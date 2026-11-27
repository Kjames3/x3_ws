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
  gru   a traj_predictor.NumpyTrajGRU, batched. server_x3 loads
        config/traj_gru_person_paths.npz (trained 2026-11-27 on Livox + C3 tracks
        from the runs before 2026-11-10; X3_PERSON_PATH_MODEL overrides), with
        its path shape smoothed over SMOOTH_S so it does not wag

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
SOURCE_GAP_S = {}      # per-source override, e.g. {"c3": 0.7} (C3 updates at 3-6 Hz); untested live
STALE_S = 2.0
MIN_SPEED_MPS = 0.15   # reality-mode.js MOVING_MPS: slower tracks get no path

# CTRV turn-rate fit (tuned on the TUNE runs, see
# notes/plans/curved_person_trajectories_2026-11-27.md).
CTRV = dict(
    hist_s=1.0,         # history kept for the fit (cv / ctrv; the GRU's is its own)
    fit_s=0.5,          # heading series length used for the turn-rate fit
    halves=False,       # also require both halves of the fit window to turn the same way
    min_fit_speed=0.25, # every velocity sample in the fit must be faster than this
    gate_t=2.0,         # |slope| / standard error must exceed this, else straight
    min_w=0.15,         # rad/s; smaller fitted turn rates are treated as straight
    max_w=1.2,          # rad/s clamp
    tau_s=0.7,          # turn rate decays as exp(-t / tau): total bend <= w * tau
    shrink=0.5,         # multiply the fitted rate (outside sustained circling,
                        # recent turning barely predicts the next 2 s)
)
FAN_W = 0.4             # rad/s: show alternatives when the person is visibly turning (|fitted
                        # rate| above this) ... direction precision is ~50 % outside circling
FAN_SE = 0.25           # ... or when the turn-rate std error exceeds this
FAN_DW = 0.6            # rad/s: alternative bends at +-FAN_DW around the main path
# Per mode: path-shape smoothing time constant (s). 0.15 for the GRU, chosen on the TUNE
# runs as the smallest value bringing its frame-to-frame wag of the 2 s endpoint
# under 1.4x unsmoothed cv's (0.124 -> 0.083 m vs cv 0.081; FDE@2s +3 %).
SMOOTH_S = {"gru": 0.15}
GRU_BEND = False
POLY_DEG = 3            # GRU path: cubic in time through the person (0 = raw). On TUNE it
                        # costs +0.5 % FDE@2s and removes the step-to-step wiggle
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
    w, se = _slope(ang, dt)
    if p.get("halves") and len(ang) >= 6:
        h = len(ang) // 2
        w1, w2 = _slope(ang[:h + 1], dt)[0], _slope(ang[h:], dt)[0]
        if w1 * w2 <= 0 or min(abs(w1), abs(w2)) < p["min_w"]:
            return w, math.inf          # a weave, not a turn: the gate keeps it straight
    return w, se


def _slope(ang, dt):
    t = np.arange(len(ang)) * dt
    t = t - t.mean()
    sxx = float((t * t).sum())
    w = float((t * (ang - ang.mean())).sum() / sxx)
    res = ang - ang.mean() - w * t
    return w, math.sqrt(max(float((res * res).sum()) / max(len(ang) - 2, 1), 1e-12) / sxx)


def gated_rate(w, se, p=CTRV):
    if not math.isfinite(se) or abs(w) < p["min_w"] or abs(w) < p["gate_t"] * se:
        return 0.0
    return float(np.clip(w * p["shrink"], -p["max_w"], p["max_w"]))


def poly_shape(t, path, deg):
    """Least-squares polynomial in time through path[0] (degree deg, no constant term):
    keeps the GRU's speed profile and bend, drops its step-to-step wiggle."""
    A = np.column_stack([t ** k for k in range(1, deg + 1)])
    coef, *_ = np.linalg.lstsq(A, path - path[0], rcond=None)
    return path[0] + A @ coef


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


def learnable(tr):
    """Rows that are measurements, the same filter the evaluation's loader applies
    (trajectory_baselines.tracks_from_ndjson): not the lidar tracker's coasting
    rows, not unconfirmed lidar tracks, not lidar rows without a fresh lidar hit
    (lidar_age_s >= 0.15), not C3's unmeasured (predicted) rows."""
    return (tr.get("mode") != "coasting" and tr.get("confirmed") is not False
            and tr.get("lidar_age_s", 0.0) < 0.15 and tr.get("measured") is not False)


class PersonPaths:
    def __init__(self, mode, model_path=None, horizon_s=HORIZON_S, n_points=N_POINTS, ctrv=None,
                 smooth_s=None, poly_deg=None, bend=None, source_models=None):
        if mode not in MODES or mode == "off":
            raise ValueError("mode must be one of cv, ctrv, gru")
        self.mode = mode
        self.smooth_s = SMOOTH_S.get(mode, 0.0) if smooth_s is None else smooth_s
        self.poly_deg = POLY_DEG if poly_deg is None else poly_deg
        # gru only: also apply the CTRV gated turn to the GRU path (its gain is
        # along-track, CTRV's is cross-track)
        self.bend = GRU_BEND if bend is None else bend
        self._smooth = {}        # key -> (obs stamp, path relative to p0)
        self._cache = {}         # key -> last prediction (reused until a new observation)
        self.p = dict(CTRV, **(ctrv or {}))
        self.times = np.linspace(0.0, horizon_s, n_points)
        self.dt = 1.0 / RATE_HZ
        self.span_s = self.p["hist_s"] if mode != "gru" else (HIST_N - 1) * self.dt
        self.model = None
        self._source_models = {}
        if mode == "gru":
            from traj_predictor import NumpyTrajGRU
            self.model = NumpyTrajGRU(model_path)   # raises if missing: the caller logs and falls back
            self.dt = 1.0 / self.model.rate
            self.hist_n = self.model.hist_n
            self.span_s = (self.hist_n - 1) * self.dt
            # A source whose tracks look like another model's training data gets that
            # model (v3 estimates: the OAK GRU, config/traj_gru_deploy.npz). Same
            # rate and history length required; a model that fails to load is skipped.
            for src, path in (source_models or {}).items():
                try:
                    m = NumpyTrajGRU(path)
                except (OSError, KeyError, ValueError):
                    continue
                if m.rate == self.model.rate and m.hist_n == self.model.hist_n:
                    self._source_models[src] = m
        self.hist_n = int(round(self.span_s / self.dt)) + 1
        self._obs = {}
        self._seen = {}          # key -> annotate() clock when last passed in
        self._last_sig = {}      # key -> raw values last observed (sources without stamps)
        self.stats = {"ticks": 0, "paths": 0, "curved": 0, "fans": 0, "ms_last": 0.0, "ms_max": 0.0}

    # ---- history ---------------------------------------------------------
    def observe(self, key, t, x, y, vx, vy):
        q = self._obs.get(key)
        gap = SOURCE_GAP_S.get(key[0], GAP_S) if isinstance(key, tuple) else GAP_S
        # a hole, or a clock jump backwards (a source switching stamp clocks), restarts it
        if q is None or (q and (t - q[-1][0] > gap or t < q[-1][0] - 1.0)):
            q = self._obs[key] = deque()
        if q and t <= q[-1][0]:
            return False
        q.append((float(t), float(x), float(y), float(vx), float(vy)))
        while len(q) > 2 and t - q[1][0] >= self.span_s:
            q.popleft()
        return True

    def forget_stale(self, now):
        """Drop tracks not passed to annotate() for STALE_S (on annotate()'s clock).
        Offline callers that only observe() pass their stamp clock as now."""
        for k in [k for k, q in self._obs.items()
                  if not q or now - self._seen.get(k, q[-1][0]) > STALE_S]:
            del self._obs[k]
            self._seen.pop(k, None)
            self._smooth.pop(k, None)
            self._cache.pop(k, None)
            self._last_sig.pop(k, None)

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
        out, gru_keys, gru_hist, fresh = {}, [], [], []
        for key in keys:
            q = self._obs.get(key)
            if not q:
                continue
            c = self._cache.get(key)
            if c is not None and c["t"] == q[-1][0]:
                out[key] = c            # no new observation since: the telemetry loop
                continue                # runs faster than the trackers
            fresh.append(key)
            _, x, y, vx, vy = q[-1]
            p0, v0 = np.array([x, y]), np.array([vx, vy])
            speed = math.hypot(vx, vy)
            hist = self.history(key)
            w, se = turn_rate(hist, self.dt, self.p) if hist is not None else (0.0, math.inf)
            r = dict(p0=p0, speed=speed, w_fit=w, se=se, used_hist=hist is not None, w=0.0, t=q[-1][0], key=key)
            if self.mode == "ctrv":
                r["w"] = gated_rate(w, se, self.p)
            r["path"] = ctrv_path(p0, v0, r["w"], self.times, self.p["tau_s"])
            r["sigma"] = SIGMA0 + SIGMA_K * self.times * max(speed, 0.3)
            if self.mode == "gru" and hist is not None:
                gru_keys.append(key)
                gru_hist.append(hist)
            out[key] = r
        groups = {}
        for key, hist in zip(gru_keys, gru_hist):
            groups.setdefault(self._source_models.get(key[0], self.model) if isinstance(key, tuple)
                              else self.model, []).append((key, hist))
        for model, items in groups.items():
            from traj_predictor import features
            keys_m = [k for k, _ in items]
            H = np.stack([h for _, h in items])
            res, sig = model(features(H, self.dt))
            fut_t = np.arange(1, model.fut_n + 1) * self.dt
            for i, key in enumerate(keys_m):
                r = out[key]
                v0 = H[i, -1, 2:4]
                fut = H[i, -1, :2] + v0 * fut_t[:, None] + res[i]
                ft = np.concatenate([[0.0], fut_t])
                fp = np.vstack([H[i, -1, :2], fut])
                if self.poly_deg:
                    fp = poly_shape(ft, fp, self.poly_deg)
                r["path"] = np.column_stack([np.interp(self.times, ft, fp[:, 0]),
                                             np.interp(self.times, ft, fp[:, 1])])
                if sig is not None:
                    r["sigma"] = np.interp(self.times, ft, np.concatenate([[sig[i, 0]], sig[i]]))
                if self.bend:
                    r["w"] = gated_rate(r["w_fit"], r["se"], self.p)
                    if r["w"]:
                        r["path"] = bend_path(r["path"], self.times, r["w"], self.p["tau_s"])
        for key in fresh:
            r = out[key]
            self._cache[key] = r
            self._smooth_path(r)
            r["alts"] = []
            if self.mode != "cv" and r["used_hist"] and r["speed"] > 0.3 and (
                    abs(r["w_fit"]) > FAN_W or r["se"] > FAN_SE):
                r["alts"] = [bend_path(r["path"], self.times, d, self.p["tau_s"]) for d in (FAN_DW, -FAN_DW)]
        return out

    def _smooth_path(self, r):
        """Exponential smoothing of the path shape (relative to the person) over
        observations, time constant smooth_s. Repeated calls on the same
        observation return the same path, so the 20 Hz telemetry loop re-reading a
        10-15 Hz tracker does not smooth twice."""
        if self.smooth_s <= 0:
            return
        rel = r["path"] - r["p0"]
        prev = self._smooth.get(r["key"])
        if prev is not None:
            dt = r["t"] - prev[0]
            if dt <= 0:
                rel = prev[1]
            elif dt < GAP_S:
                rel = prev[1] + (1.0 - math.exp(-dt / self.smooth_s)) * (rel - prev[1])
        self._smooth[r["key"]] = (r["t"], rel)
        r["path"] = r["p0"] + rel

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
            self._seen[key] = now
            # The lidar tracker's coasting rows and C3's unmeasured (predicted) rows
            # are the trackers' own constant-velocity extrapolation: drawing from them
            # is fine, learning a turn from them is not. A source without stamps
            # (v3) passes a `_sig` of its raw values: an unchanged estimate re-read
            # by the 20 Hz loop is not a new observation.
            sig = tr.get("_sig")
            fresh = sig is None or self._last_sig.get(key) != sig
            if learnable(tr) and fresh:
                ts = tr.get(stamp_key) if stamp_key else None
                self.observe(key, float(ts) if ts is not None else now, x, y, vx, vy)
                if sig is not None:
                    self._last_sig[key] = sig
            keys.append(key)
            rows.append((key, tr))
        # Slow tracks get no path, so skip their fit / GRU pass entirely.
        moving = [k for k in keys if k in self._obs and math.hypot(*self._obs[k][-1][3:5]) >= MIN_SPEED_MPS]
        preds = self.predict_many(moving) if pose is not None else {}
        th = float(pose["theta"]) if pose is not None else 0.0
        c, s = math.cos(th), math.sin(th)

        def rel(path, p0):
            d = path - p0
            # plain floats: server_x3 encodes with orjson, which rejects numpy scalars
            return [[round(float(c * a + s * b), 3), round(float(-s * a + c * b), 3)] for a, b in d]

        out = []
        for key, tr in rows:
            r = preds.get(key)
            if r is None or r["speed"] < MIN_SPEED_MPS:
                out.append(tr)
                continue
            self.stats["paths"] += 1
            self.stats["curved"] += bool(self.mode == "gru" and r["used_hist"] or r["w"])
            self.stats["fans"] += bool(r["alts"])
            # Offsets from the newest OBSERVED position (a coasting track has moved on
            # since; the GUI anchors the offsets at the avatar, so this is display-exact).
            out.append(dict(tr, path_rel=rel(r["path"], r["p0"]),
                            path_sigma=[round(float(v), 3) for v in r["sigma"]],
                            path_alts=[rel(a, r["p0"]) for a in r["alts"]],
                            # False: too little history, the straight fallback
                            path_model=bool(r["used_hist"])))
        self.forget_stale(now)
        ms = (time.perf_counter() - t_start) * 1e3
        self.stats["ticks"] += 1
        self.stats["ms_last"] = round(ms, 3)
        self.stats["ms_max"] = round(max(self.stats["ms_max"], ms), 3)
        return out


# ---- server_x3 glue ----------------------------------------------------------

def from_env(model_path, env=None, log=None):
    """PersonPaths for X3_PERSON_PATH (cv | ctrv | gru), or None when unset / 'off'.
    A missing GRU model logs and falls back to ctrv rather than taking the server down."""
    import os
    mode = (env if env is not None else os.environ.get("X3_PERSON_PATH", "off")).strip().lower()
    if mode in ("", "off", "0", "none"):
        return None
    if mode not in MODES:
        if log:
            log.warning("X3_PERSON_PATH=%r unknown (cv|ctrv|gru|off): person paths off", mode)
        return None
    try:
        from pathlib import Path
        # v3 (camera-only, C3 off) tracks are the OAK GRU's own domain; the Livox/C3
        # model is 14 % worse than cv there (walk-g-r7, 2026-11-27).
        return PersonPaths(mode, model_path,
                           source_models={"v3": str(Path(model_path).with_name("traj_gru_deploy.npz"))})
    except Exception as e:  # noqa: BLE001
        if log:
            log.warning("X3_PERSON_PATH=gru: model %s failed (%s); using ctrv", model_path, e)
        return PersonPaths("ctrv")


def estimates_with_odom(estimates, pose):
    """velocity_estimator estimates (camera: x right, z forward; vx fwd, vy left in base
    axes) with odom_x/odom_y/odom_vx/odom_vy added, the same transform traj_shadow uses."""
    c, s = math.cos(pose["theta"]), math.sin(pose["theta"])
    out = []
    for e in estimates:
        px, py = float(e.get("z", 1.0)), -float(e.get("x", 0.0))
        vx, vy = float(e.get("vx", 0.0)), float(e.get("vy", 0.0))
        out.append(dict(e, odom_x=pose["x"] + c * px - s * py, odom_y=pose["y"] + s * px + c * py,
                        odom_vx=c * vx - s * vy, odom_vy=s * vx + c * vy, _sig=(px, py, vx, vy)))
    return out


_ODOM_KEYS = ("odom_x", "odom_y", "odom_vx", "odom_vy")


_PATH_KEYS = ("path_rel", "path_sigma", "path_alts", "path_model")


def borrow_lidar_paths(c3_tracks, lidar_tracks):
    """C3 tracks (the front avatars) take the path of the lidar track seeded from them.

    C3 updates at 3-6 Hz with gaps, so only ~35 % of moving C3 display time has
    the 1 s history a model path needs (the rest is the straight fallback); the
    Livox tracks have it ~86 % of the time (REPORT ndjson replay, 2026-11-27).
    The GUI draws the C3 avatar and hides the co-located lidar one, so the
    better path would otherwise never be seen. Linked = a confirmed, measured
    (not coasting) lidar track whose camera_id is this C3 id (side cameras are
    'left:N' / 'right:N' / 'rear:N' and never match). Offsets are
    person-relative, so they transplant onto the C3 avatar."""
    linked = {}
    for t in lidar_tracks:
        cid = t.get("camera_id")
        if (cid is not None and "path_rel" in t and t.get("confirmed", True)
                and t.get("mode") != "coasting"):
            linked[str(cid)] = t
    out = []
    for c in c3_tracks:
        l = linked.get(str(c.get("id")))
        if l is not None and (l.get("path_model") or "path_rel" not in c):
            if not c.get("path_model") or l.get("path_model"):
                c = dict(c, **{k: l[k] for k in _PATH_KEYS if k in l})
        out.append(c)
    return out


def annotate_telemetry(paths, pose, c3_tracks, lidar_persons, velocity_estimates, c3_active=False):
    """(c3_tracks, lidar_persons, velocity_estimates) with paths added, or the same
    objects untouched when paths is None (X3_PERSON_PATH off) or there is no pose.

    v3 estimates are annotated only when C3 is not running (c3_active False): the
    GUI draws v3 people only then (scene-view.js avatarEntries). Static blobs and
    is_person False estimates never get a path."""
    if paths is None or pose is None:
        return c3_tracks, lidar_persons, velocity_estimates
    c3 = paths.annotate(c3_tracks, "c3", pose, stamp_key="state_stamp_s") if c3_tracks else c3_tracks
    lp = paths.annotate(lidar_persons, "lidar", pose, stamp_key="stamp") if lidar_persons else lidar_persons
    if c3 and lp:
        c3 = borrow_lidar_paths(c3, lp)
    ve = velocity_estimates
    people = [e for e in velocity_estimates or []
              if e.get("category") != "static" and e.get("is_person") is not False]
    if people and not c3_active:
        ann = {id(e): a for e, a in zip(people, paths.annotate(estimates_with_odom(people, pose), "v3", pose))}
        # hand back the original dicts plus the path fields only (no odom_* leak into the payload)
        ve = [dict(e, **{k: v for k, v in ann[id(e)].items() if k.startswith("path_")})
              if id(e) in ann and "path_rel" in ann[id(e)] else e for e in velocity_estimates]
    return c3, lp, ve
