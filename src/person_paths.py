"""Predicted walking paths for the GUI's Reality view (display only).

server_x3 calls annotate() on the C3 and lidar person tracks it already sends.
With X3_PERSON_PATH unset or 'off' nothing is created and the payload does not
change. Otherwise every moving track gets

    path_rel   [[dfwd, dleft], ...]  N_POINTS points over HORIZON_S (point 0 is
               [0, 0]): offsets from the track's position now, in base axes
               (x forward, y left) at the robot's current heading
    path_sigma [s0, ..., sN]         1-sigma position spread (m) at each point
    path_alts  [[[dfwd, dleft], ...], ...]   0 or 2 faint alternatives (a
               sharper left and right bend), only when the turn is uncertain

Offsets rather than positions, because each source places its avatar slightly
differently (C3 is capture-time corrected, the GUI subtracts OAK_MOUNT_X for
camera tracks); the GUI anchors the offsets at the avatar it draws.

Modes
  cv    straight line at the tracker's velocity (today's GUI drawing, as data)
  ctrv  constant speed with a turn rate fitted over the last second of
        history, decaying with time constant TAU_S so it bends without
        spiralling; straight when the turn evidence is weak
  gru   config/traj_gru_deploy.npz (traj_predictor.NumpyTrajGRU), batched

Every mode works in the ODOM frame: histories are odom positions, so the
robot's own motion does not look like the person turning.
"""
import math
import time
from collections import deque

import numpy as np

MODES = ("off", "cv", "ctrv", "gru")
HORIZON_S = 2.0
N_POINTS = 16          # = reality-mode.js TRAIL_N, one point per 2/15 s
RATE_HZ = 10.0         # history resample rate (the GRU's)
HIST_N = 11            # 1.0 s of history at RATE_HZ
GAP_S = 0.35           # a hole longer than this restarts the history
STALE_S = 2.0
MIN_SPEED_MPS = 0.15   # reality-mode.js MOVING_MPS: slower tracks get no path

# CTRV turn-rate fit (tuned on the TUNE runs, see
# notes/plans/curved_person_trajectories_2026-11-27.md).
CTRV = dict(
    fit_s=0.8,          # heading series length used for the turn-rate fit
    min_fit_speed=0.25, # every velocity sample in the fit must be faster than this
    gate_t=2.0,         # |slope| / standard error must exceed this, else straight
    min_w=0.15,         # rad/s; smaller fitted turn rates are treated as straight
    max_w=1.2,          # rad/s clamp
    tau_s=1.0,          # turn rate decays as exp(-t / tau)
    shrink=0.8,         # multiply the fitted rate (fits over-read noisy turns)
)
FAN_SE = 0.25           # rad/s: show alternatives when the turn-rate std error exceeds this
FAN_DW = 0.6            # rad/s: alternative bends at +-FAN_DW around the main path
SIGMA0, SIGMA_K = 0.08, 0.22   # heuristic spread for cv / ctrv: s0 + k * t * speed


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def turn_rate(hist, dt, p=CTRV):
    """(w, se) from the estimator-velocity heading over the last fit_s.

    hist: [n, 4] (x, y, vx, vy) at dt. Weighted least-squares slope of the
    unwrapped heading; se is its standard error. (0, inf) when too slow.
    """
    m = max(3, int(round(p["fit_s"] / dt)) + 1)
    v = hist[-m:, 2:4]
    sp = np.hypot(v[:, 0], v[:, 1])
    if len(v) < 3 or sp.min() < p["min_fit_speed"]:
        return 0.0, math.inf
    ang = np.unwrap(np.arctan2(v[:, 1], v[:, 0]))
    t = np.arange(len(ang)) * dt
    t = t - t.mean()
    sxx = float((t * t).sum())
    w = float((t * (ang - ang.mean())).sum() / sxx)
    res = ang - ang.mean() - w * t
    se = math.sqrt(max(float((res * res).sum()) / max(len(ang) - 2, 1), 1e-12) / sxx)
    return w, se


def gated_rate(w, se, p=CTRV):
    if not math.isfinite(se) or abs(w) < p["min_w"] or abs(w) < p["gate_t"] * se:
        return 0.0
    return float(np.clip(w * p["shrink"], -p["max_w"], p["max_w"]))


def bend(t, w0, tau):
    """Heading change at times t for turn rate w0 decaying with tau."""
    return w0 * tau * (1.0 - np.exp(-t / tau)) if tau > 0 else w0 * t


def ctrv_path(p0, v0, w0, times, tau, sub=5):
    """Positions at `times` (starting with 0) for speed |v0|, heading from v0, damped turn w0."""
    s = math.hypot(v0[0], v0[1])
    th0 = math.atan2(v0[1], v0[0])
    tf = np.linspace(0.0, times[-1], (len(times) - 1) * sub + 1)
    th = th0 + bend(tf, w0, tau)
    # trapezoid integration of constant-speed heading
    step = np.diff(tf)[:, None] * s * 0.5 * (np.column_stack([np.cos(th[:-1]), np.sin(th[:-1])])
                                             + np.column_stack([np.cos(th[1:]), np.sin(th[1:])]))
    pos = np.vstack([[0.0, 0.0], np.cumsum(step, 0)]) + p0
    return np.column_stack([np.interp(times, tf, pos[:, 0]), np.interp(times, tf, pos[:, 1])])


def bend_path(path, times, dw, tau):
    """Rotate each increment of `path` by the heading change a turn rate dw adds."""
    inc = np.diff(path, axis=0)
    a = bend(0.5 * (times[1:] + times[:-1]), dw, tau)
    c, s = np.cos(a), np.sin(a)
    rot = np.column_stack([c * inc[:, 0] - s * inc[:, 1], s * inc[:, 0] + c * inc[:, 1]])
    return np.vstack([path[:1], path[:1] + np.cumsum(rot, 0)])


class PersonPaths:
    def __init__(self, mode, model_path=None, horizon_s=HORIZON_S, n_points=N_POINTS, ctrv=None):
        if mode not in MODES or mode == "off":
            raise ValueError("mode must be one of cv, ctrv, gru")
        self.mode = mode
        self.p = dict(CTRV, **(ctrv or {}))
        self.times = np.linspace(0.0, horizon_s, n_points)
        self.dt = 1.0 / RATE_HZ
        self.span_s = (HIST_N - 1) * self.dt
        self.model = None
        if mode == "gru":
            from traj_predictor import NumpyTrajGRU
            self.model = NumpyTrajGRU(model_path)   # raises if missing: the caller logs and falls back
            self.dt = 1.0 / self.model.rate
            self.hist_n = self.model.hist_n
            self.span_s = (self.hist_n - 1) * self.dt
            self._fut_t = np.arange(1, self.model.fut_n + 1) * self.dt
        self.hist_n = int(round(self.span_s / self.dt)) + 1
        self._obs = {}
        self.stats = {"ticks": 0, "paths": 0, "curved": 0, "fans": 0, "ms_last": 0.0, "ms_max": 0.0}

    # ---- history ---------------------------------------------------------
    def observe(self, key, t, x, y, vx, vy):
        q = self._obs.get(key)
        if q is None or (q and t - q[-1][0] > GAP_S):
            q = self._obs[key] = deque()
        if q and t <= q[-1][0]:
            return False
        q.append((float(t), float(x), float(y), float(vx), float(vy)))
        while len(q) > 2 and t - q[1][0] >= self.span_s:
            q.popleft()
        return True

    def forget_stale(self, now):
        for k in [k for k, q in self._obs.items() if not q or now - q[-1][0] > STALE_S]:
            del self._obs[k]

    def history(self, key):
        q = self._obs.get(key)
        if not q or q[-1][0] - q[0][0] < self.span_s - 1e-6:
            return None
        a = np.array(q)
        g = a[-1, 0] - self.span_s + np.arange(self.hist_n) * self.dt
        return np.stack([np.interp(g, a[:, 0], a[:, c]) for c in range(1, 5)], 1)

    # ---- prediction --------------------------------------------------------
    def predict_many(self, keys):
        """{key: dict(path [n,2] odom, sigma [n], alts list, w, used_hist)} for keys with history."""
        out, gru_keys, gru_hist = {}, [], []
        for key in keys:
            q = self._obs.get(key)
            if not q:
                continue
            _, x, y, vx, vy = q[-1]
            p0, v0 = np.array([x, y]), np.array([vx, vy])
            speed = math.hypot(vx, vy)
            hist = self.history(key)
            w, se = turn_rate(hist, self.dt, self.p) if hist is not None else (0.0, math.inf)
            r = dict(p0=p0, speed=speed, w_fit=w, se=se, used_hist=hist is not None, w=0.0)
            if self.mode == "ctrv":
                r["w"] = gated_rate(w, se, self.p)
            r["path"] = ctrv_path(p0, v0, r["w"], self.times, self.p["tau_s"])
            r["sigma"] = SIGMA0 + SIGMA_K * self.times * max(speed, 0.3)
            if self.mode == "gru" and hist is not None:
                gru_keys.append(key)
                gru_hist.append(hist)
            out[key] = r
        if gru_keys:
            from traj_predictor import features
            H = np.stack(gru_hist)
            res, sig = self.model(features(H, self.dt))
            for i, key in enumerate(gru_keys):
                r = out[key]
                v0 = H[i, -1, 2:4]
                fut = H[i, -1, :2] + v0 * self._fut_t[:, None] + res[i]
                ft = np.concatenate([[0.0], self._fut_t])
                fp = np.vstack([H[i, -1, :2], fut])
                r["path"] = np.column_stack([np.interp(self.times, ft, fp[:, 0]),
                                             np.interp(self.times, ft, fp[:, 1])])
                if sig is not None:
                    r["sigma"] = np.interp(self.times, ft, np.concatenate([[sig[i, 0]], sig[i]]))
        for r in out.values():
            r["alts"] = []
            if self.mode != "cv" and r["used_hist"] and r["speed"] > 0.3 and r["se"] > FAN_SE:
                r["alts"] = [bend_path(r["path"], self.times, d, self.p["tau_s"]) for d in (FAN_DW, -FAN_DW)]
        return out

    # ---- server hook -------------------------------------------------------
    def annotate(self, tracks, source, pose, now=None, stamp_key=None):
        """Copies of `tracks` (dicts with odom_x/odom_y/odom_vx/odom_vy and id) with
        path_rel / path_sigma / path_alts added for moving ones. pose: robot odom pose."""
        t_start = time.perf_counter()
        now = time.monotonic() if now is None else now
        keys, rows = [], []
        for tr in tracks:
            try:
                x, y = float(tr["odom_x"]), float(tr["odom_y"])
                vx, vy = float(tr.get("odom_vx", 0.0)), float(tr.get("odom_vy", 0.0))
            except (KeyError, TypeError, ValueError):
                rows.append((None, tr))
                continue
            key = (source, tr.get("id"))
            ts = tr.get(stamp_key) if stamp_key else None
            self.observe(key, float(ts) if ts is not None else now, x, y, vx, vy)
            keys.append(key)
            rows.append((key, tr))
        preds = self.predict_many([k for k in keys if k in self._obs]) if pose is not None else {}
        th = float(pose["theta"]) if pose is not None else 0.0
        c, s = math.cos(th), math.sin(th)

        def rel(path, p0):
            d = path - p0
            return [[round(c * a + s * b, 3), round(-s * a + c * b, 3)] for a, b in d]

        out = []
        for key, tr in rows:
            r = preds.get(key)
            if r is None or r["speed"] < MIN_SPEED_MPS:
                out.append(tr)
                continue
            self.stats["paths"] += 1
            self.stats["curved"] += bool(self.mode == "gru" and r["used_hist"] or r["w"])
            self.stats["fans"] += bool(r["alts"])
            out.append(dict(tr, path_rel=rel(r["path"], r["p0"]),
                            path_sigma=[round(float(v), 3) for v in r["sigma"]],
                            path_alts=[rel(a, r["p0"]) for a in r["alts"]]))
        self.forget_stale(now)
        ms = (time.perf_counter() - t_start) * 1e3
        self.stats["ticks"] += 1
        self.stats["ms_last"] = round(ms, 3)
        self.stats["ms_max"] = round(max(self.stats["ms_max"], ms), 3)
        return out
