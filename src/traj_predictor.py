"""Pedestrian trajectory prediction at runtime (numpy only, no torch).

Loads the gru_deploy.npz written by
``tools/perception/train_traj_gru.py --deploy`` and runs the same model:
a GRU that predicts a RESIDUAL on cv_est (the track's own velocity
extrapolated in a straight line). With too little history, or no model file,
it returns cv_est itself, so it can never do worse than the CBF's current
constant-velocity assumption by failing.

Feed it one observation per track per frame (any rate). Each track's history
is resampled to the model rate (10 Hz) over the last ``hist_n`` samples, the
same as trajectory_baselines.segments(). A track with a hole longer than
GAP_S starts over, matching how the training windows were cut.

Frames: the model is translation- and rotation-invariant (trained with random
rotations), so any consistent frame works. The training data was in the ODOM
frame, ego-compensated. Feed it ODOM or world positions, never base-frame
positions from a moving robot.

    pred = TrajPredictor('config/traj_gru_deploy.npz')
    pred.observe(track_id, t, x, y, vx, vy)
    out = pred.predict(track_id)      # dict(path=[F,2] absolute, sigma=[F] or None, used_model=bool)
    v_eff, sig = pred.cbf_velocity(track_id, horizon_s=1.0)
"""
import json
from collections import deque

import numpy as np

GAP_S = 0.35          # same as trajectory_baselines.GAP_S
STALE_S = 2.0         # forget tracks not observed for this long


def _sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


class NumpyTrajGRU:
    """Forward pass of train_traj_gru.TrajGRU from its exported state_dict."""

    def __init__(self, path):
        d = np.load(path, allow_pickle=False)
        self.meta = json.loads(str(d['meta']))
        self.w = {k: d[k].astype(np.float64) for k in d.files if k != 'meta'}
        self.fut_n = int(self.meta['fut_n'])
        self.layers = int(self.meta['layers'])
        self.nll = bool(self.meta['nll'])
        self.rate = float(self.meta['rate'])
        self.hist_n = int(self.meta['hist_n'])

    def __call__(self, x):
        """x: [B, H, 6] raw features -> (residual [B, F, 2], sigma [B, F] or None)."""
        w = self.w
        h_seq = np.maximum(x / w['scale'] @ w['inp.weight'].T + w['inp.bias'], 0.0)
        for layer in range(self.layers):
            Wi, Wh = w['gru.weight_ih_l%d' % layer], w['gru.weight_hh_l%d' % layer]
            bi, bh = w['gru.bias_ih_l%d' % layer], w['gru.bias_hh_l%d' % layer]
            H = Wh.shape[1]
            h = np.zeros((x.shape[0], H))
            outs = []
            gi_all = h_seq @ Wi.T + bi
            for t in range(h_seq.shape[1]):
                gi, gh = gi_all[:, t], h @ Wh.T + bh
                r = _sigmoid(gi[:, :H] + gh[:, :H])
                z = _sigmoid(gi[:, H:2 * H] + gh[:, H:2 * H])
                n = np.tanh(gi[:, 2 * H:] + r * gh[:, 2 * H:])
                h = (1 - z) * n + z * h
                outs.append(h)
            h_seq = np.stack(outs, 1)
        last = h_seq[:, -1]
        o = np.maximum(last @ w['head.0.weight'].T + w['head.0.bias'], 0.0) @ w['head.2.weight'].T + w['head.2.bias']
        o = o.reshape(-1, self.fut_n, 3 if self.nll else 2)
        if not self.nll:
            return o, None
        return o[..., :2], np.logaddexp(0.0, o[..., 2]) + 0.01      # softplus + 1 cm floor


def features(hist, dt):
    """Same as train_traj_gru.features: [N, H, 6] rel position, estimator velocity, finite difference."""
    rel = hist[:, :, :2] - hist[:, -1:, :2]
    fd = np.diff(hist[:, :, :2], axis=1, prepend=hist[:, :1, :2]) / dt
    return np.concatenate([rel, hist[:, :, 2:4], fd], 2)


class TrajPredictor:
    def __init__(self, model_path=None, rate=10.0, hist_n=11, fut_n=30):
        self.model = None
        if model_path:
            try:
                self.model = NumpyTrajGRU(model_path)
            except (OSError, KeyError, ValueError):
                self.model = None
        if self.model is not None:
            rate, hist_n, fut_n = self.model.rate, self.model.hist_n, self.model.fut_n
        self.rate, self.dt, self.hist_n, self.fut_n = rate, 1.0 / rate, hist_n, fut_n
        self.span_s = (hist_n - 1) * self.dt
        self._obs = {}          # id -> deque of (t, x, y, vx, vy)

    def observe(self, tid, t, x, y, vx, vy):
        q = self._obs.get(tid)
        if q is None or (q and t - q[-1][0] > GAP_S):
            q = self._obs[tid] = deque()
        if q and t <= q[-1][0]:
            return
        q.append((float(t), float(x), float(y), float(vx), float(vy)))
        while len(q) > 2 and t - q[1][0] >= self.span_s:
            q.popleft()

    def forget_stale(self, now):
        for tid in [k for k, q in self._obs.items() if not q or now - q[-1][0] > STALE_S]:
            del self._obs[tid]

    def history(self, tid):
        """[hist_n, 4] resampled at the model rate ending at the latest sample, or None if too short."""
        q = self._obs.get(tid)
        if not q or q[-1][0] - q[0][0] < self.span_s - 1e-6:
            return None
        a = np.array(q)
        g = a[-1, 0] - self.span_s + np.arange(self.hist_n) * self.dt
        return np.stack([np.interp(g, a[:, 0], a[:, c]) for c in range(1, 5)], 1)

    def predict(self, tid):
        q = self._obs.get(tid)
        if not q:
            return None
        t, x, y, vx, vy = q[-1]
        steps = np.arange(1, self.fut_n + 1)[:, None] * self.dt
        base = np.array([vx, vy]) * steps
        hist = self.history(tid)
        if self.model is None or hist is None:
            return dict(path=base + [x, y], sigma=None, used_model=False, t=t)
        res, sig = self.model(features(hist[None], self.dt))
        p0 = hist[-1, :2]
        return dict(path=p0 + base + res[0], sigma=None if sig is None else sig[0], used_model=True, t=t)

    def cbf_velocity(self, tid, horizon_s=1.0):
        """Mean velocity over the next horizon_s implied by the prediction, and the sigma there.

        The CBF row is n.(u - v_i) + gamma*h/(2d) >= 0 with one constant v_i, so
        the prediction enters as its average velocity to the chosen horizon.
        """
        out = self.predict(tid)
        if out is None:
            return None, None
        k = max(1, min(self.fut_n, int(round(horizon_s * self.rate))))
        q = self._obs[tid][-1]
        v = (out['path'][k - 1] - [q[1], q[2]]) / (k * self.dt)
        return (float(v[0]), float(v[1])), (None if out['sigma'] is None else float(out['sigma'][k - 1]))
