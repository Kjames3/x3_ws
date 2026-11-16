"""Shadow mode for the trajectory GRU: predict live, log, never act.

Fed from server_x3's telemetry loop with the velocity estimator's output and
the robot's /odom pose. For every person track it runs traj_predictor (the
numpy GRU, falling back to cv_est with too little history) and appends one
NDJSON line per tick with, per track:

    odom position/velocity now, the GRU path and sigma at HORIZONS_S, the
    cv_est path at the same horizons, used_model, and cbf_v / cbf_sigma: the
    mean velocity to CBF_HORIZON_S that would replace the constant velocity in
    the CBF's moving-obstacle row (rotated back into base axes), next to the
    estimator velocity the CBF uses today.

Control is untouched. src/tools/perception/traj_shadow_score.py grades the
logged predictions against where each track actually went (from the same log).

Inputs match training exactly: tools/perception/trajectory_baselines.py read
/pedestrian_states, which server_x3.publish_pedestrians fills with
position (z, -x) and velocity (vx, vy) from each estimate (is_person False
skipped), then rotated into odom with the robot pose at that time.
"""
import json
import math
import time

import numpy as np

from traj_predictor import TrajPredictor

HORIZONS_S = (0.5, 1.0, 2.0, 3.0)
CBF_HORIZON_S = 1.0
PERIOD_S = 0.2              # at most 5 Hz: the telemetry loop is latency-sensitive
ODOM_STALE_S = 0.5          # skip ticks without a fresh /odom (odom-frame positions need it)


def base_from_estimate(est):
    """(px, py, vx, vy) in base axes exactly as publish_pedestrians fills /pedestrian_states."""
    return (float(est.get("z", 1.0)), -float(est.get("x", 0.0)),
            float(est.get("vx", 0.0)), float(est.get("vy", 0.0)))


def _r(v, n=3):
    return None if v is None else round(float(v), n)


class TrajShadow:
    def __init__(self, model_path, log_path=None, period_s=PERIOD_S, horizons_s=HORIZONS_S):
        self.pred = TrajPredictor(model_path)
        self.model_loaded = self.pred.model is not None
        self.enabled = True             # server_x3 clears this if update() ever raises
        self.period_s = period_s
        self.horizons_s = tuple(h for h in horizons_s if h * self.pred.rate <= self.pred.fut_n)
        self._last = -1e9
        self._log = open(log_path, "a", buffering=1) if log_path else None
        self.stats = {"ticks": 0, "skipped_stale_odom": 0, "predictions": 0, "used_model": 0}
        if self._log:
            self._write({"type": "meta", "model": str(model_path), "model_loaded": self.model_loaded,
                         "rate": self.pred.rate, "hist_n": self.pred.hist_n, "fut_n": self.pred.fut_n,
                         "horizons_s": list(self.horizons_s), "cbf_horizon_s": CBF_HORIZON_S,
                         "frame": "odom", "t_clock": "monotonic"})

    def _write(self, row):
        if self._log is None:
            return
        try:
            self._log.write(json.dumps(row, separators=(",", ":")) + "\n")
        except (OSError, ValueError):
            self._log = None

    def update(self, estimates, pose, now=None, odom_age_s=0.0):
        """estimates: velocity_estimator.get_estimates(); pose: {x, y, theta} in odom.
        Returns the row it logged (or None when throttled / skipped)."""
        now = time.monotonic() if now is None else now
        if pose is None or odom_age_s > ODOM_STALE_S:
            if now - self._last >= self.period_s:
                self._last = now
                self.stats["ticks"] += 1
                self.stats["skipped_stale_odom"] += 1
            return None
        x0, y0, th = float(pose["x"]), float(pose["y"]), float(pose["theta"])
        c, s = math.cos(th), math.sin(th)
        R = np.array([[c, -s], [s, c]])
        # Every tick feeds the history (a deque append); only predict + log is throttled.
        seen = []
        for est in estimates:
            if est.get("is_person") is False:
                continue
            tid = int(est.get("id", 0))
            px, py, vx, vy = base_from_estimate(est)
            wx, wy = R @ (px, py) + (x0, y0)
            wvx, wvy = R @ (vx, vy)
            self.pred.observe(tid, now, wx, wy, wvx, wvy)
            seen.append((tid, px, py, vx, vy, wx, wy, wvx, wvy))
        if now - self._last < self.period_s:
            return None
        self._last = now
        self.stats["ticks"] += 1
        tracks = []
        for tid, px, py, vx, vy, wx, wy, wvx, wvy in seen:
            out = self.pred.predict(tid)
            if out is None:
                continue
            k = [max(1, int(round(h * self.pred.rate))) for h in self.horizons_s]
            path = out["path"]
            sig = out["sigma"]
            cv = [(wx + wvx * h, wy + wvy * h) for h in self.horizons_s]
            # TrajPredictor.cbf_velocity's formula on the path already computed
            # (calling it would run the GRU a second time).
            kc = max(1, min(self.pred.fut_n, int(round(CBF_HORIZON_S * self.pred.rate))))
            v_eff = (path[kc - 1] - (wx, wy)) / (kc * self.pred.dt)
            v_sig = None if sig is None else sig[kc - 1]
            cbf_v_base = R.T @ np.asarray(v_eff)             # odom -> base axes, like the CBF row
            self.stats["predictions"] += 1
            self.stats["used_model"] += bool(out["used_model"])
            tracks.append({
                "id": tid, "x": _r(wx), "y": _r(wy), "vx": _r(wvx), "vy": _r(wvy),
                "base": [_r(px), _r(py)], "est_v_base": [_r(vx), _r(vy)],
                "used_model": bool(out["used_model"]),
                "gru": [[_r(path[i - 1][0]), _r(path[i - 1][1])] for i in k],
                "sigma": None if sig is None else [_r(sig[i - 1]) for i in k],
                "cv": [[_r(a), _r(b)] for a, b in cv],
                "cbf_v": [_r(cbf_v_base[0]), _r(cbf_v_base[1])],
                "cbf_sigma": _r(v_sig),
            })
        self.pred.forget_stale(now)
        row = {"type": "tick", "t": round(now, 4), "pose": [_r(x0), _r(y0), _r(th, 4)], "tracks": tracks}
        if tracks:                      # empty rooms are not logged
            self._write(row)
        return row

    def close(self):
        if self._log is not None:
            self._log.close()
            self._log = None
