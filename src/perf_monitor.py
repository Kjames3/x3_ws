"""
perf_monitor.py — live performance tracking for the X3 perception stack.

Tracks two things, continuously, on the running robot:

  1. LATENCY — rolling percentiles (p50/p95/p99) for every instrumented stage:
     OAK-D NN decode, detection end-to-end age, velocity-estimator cycle,
     MLP forward pass, and the effective rate of each.

  2. ACCURACY — with **no manual labels**:

     * OAK-D detections: labels are not available online, so we track the
       quality proxies that actually move when the detector degrades —
       mean/low-tail confidence, detections per frame, fraction of boxes that
       get a valid depth back-projection, empty-frame rate, and *flicker*
       (a label that disappears for 1-2 frames and returns, i.e. a missed
       detection sandwiched between two hits). A separately-computed labelled
       score (from an offline eval run) can be folded in via
       `record_labeled_eval()` so the same snapshot carries both.

     * Velocity MLP: labels come for free from the future. The model is causal
       — it predicts (vx, vy) from a window ending at time t — so a few hundred
       ms later we can central-difference the track's WORLD-frame positions
       around t, rotate into the robot frame the model predicted in, and score
       the prediction against it. That yields live MAE / RMSE / bias / speed
       and heading error with zero annotation. Same trick as the auto-labeling
       described in VELOCITY_SELF_TRAINING_PLAN.md, used here for measurement
       rather than training.

Everything is bounded (fixed-size deques), lock-guarded, and cheap enough to
call from the hot loops. Snapshots are exported to the GUI via telemetry and
appended to a JSONL file under src/logs/perf/ for offline trending.

Usage:
    from perf_monitor import get_monitor
    perf = get_monitor()

    with perf.timer("vel.infer_ms"):
        pred = model(x)

    perf.record_detection_frame(dets, e2e_ms=12.4)
    perf.record_velocity_sample(tid, t, wx, wy, theta, vx, vy)
    snap = perf.snapshot()
"""

import json
import logging
import math
import os
import threading
import time
from collections import defaultdict, deque
from contextlib import contextmanager
from datetime import datetime
from pathlib import Path

logger = logging.getLogger(__name__)

_SRC_DIR = Path(__file__).parent.resolve()
LOG_DIR = _SRC_DIR / "logs" / "perf"

# ── Tuning ────────────────────────────────────────────────────────────────────
WINDOW_N = 600           # samples kept per latency series (~60 s at 10 Hz)
DET_WINDOW_N = 300       # detection frames kept for the quality proxies
VEL_WINDOW_N = 600       # scored velocity predictions kept

# Self-supervised velocity label: central difference over ±LABEL_HORIZON_S of
# world-frame track position. Long enough to beat centroid jitter, short enough
# that the true velocity is roughly constant across the interval.
LABEL_HORIZON_S = 0.35
LABEL_TOLERANCE_S = 0.12  # accepted slack when picking the ±horizon neighbours
TRACK_STREAM_N = 40       # per-track position samples retained (~4 s at 10 Hz)
MOVING_THRESH_MS = 0.15   # |v_gt| above this counts as "moving" for the split

SNAPSHOT_LOG_INTERVAL_S = 10.0


def _pct(sorted_vals, q):
    """Nearest-rank percentile on an already-sorted list."""
    if not sorted_vals:
        return 0.0
    k = min(len(sorted_vals) - 1, max(0, int(round(q * (len(sorted_vals) - 1)))))
    return float(sorted_vals[k])


class Series:
    """Fixed-size rolling numeric series with percentile summary."""

    __slots__ = ("vals", "total", "name")

    def __init__(self, name, maxlen=WINDOW_N):
        self.name = name
        self.vals = deque(maxlen=maxlen)
        self.total = 0

    def add(self, v):
        self.vals.append(float(v))
        self.total += 1

    def summary(self):
        if not self.vals:
            return {"n": 0}
        s = sorted(self.vals)
        return {
            "n": len(s),
            "total": self.total,
            "mean": round(sum(s) / len(s), 2),
            "p50": round(_pct(s, 0.50), 2),
            "p95": round(_pct(s, 0.95), 2),
            "p99": round(_pct(s, 0.99), 2),
            "max": round(s[-1], 2),
        }


class RateCounter:
    """Events/second over a sliding wall-clock window."""

    def __init__(self, window_s=5.0):
        self.window_s = window_s
        self.stamps = deque()

    def tick(self, t=None):
        t = t if t is not None else time.monotonic()
        self.stamps.append(t)
        cutoff = t - self.window_s
        while self.stamps and self.stamps[0] < cutoff:
            self.stamps.popleft()

    def rate(self):
        if len(self.stamps) < 2:
            return 0.0
        span = self.stamps[-1] - self.stamps[0]
        return round((len(self.stamps) - 1) / span, 2) if span > 0 else 0.0


class PerfMonitor:
    """Thread-safe registry of latency series and accuracy estimators."""

    def __init__(self, log_to_file=None):
        self._lock = threading.Lock()
        self._series = {}                       # name -> Series
        self._rates = {}                        # name -> RateCounter
        self._last_stamp = {}                   # name -> monotonic, for interval series

        # ── detection quality state ──
        self._det_frames = deque(maxlen=DET_WINDOW_N)   # per-frame dicts
        self._det_presence = defaultdict(lambda: deque(maxlen=DET_WINDOW_N))
        self._det_flickers = defaultdict(int)
        self._det_frame_count = 0
        self._labeled_eval = None               # last offline labelled score

        # ── velocity self-supervised scoring state ──
        self._vel_streams = {}                  # tid -> deque of samples
        self._vel_scored = deque(maxlen=VEL_WINDOW_N)   # scored prediction dicts
        self._vel_pending = 0
        self._vel_dropped = 0

        self._started = time.monotonic()
        self._log_path = None
        self._last_log = 0.0
        env = os.environ.get("X3_PERF_LOG", "1")
        self._log_enabled = env not in ("0", "false", "no") if log_to_file is None else log_to_file

    # ── latency ───────────────────────────────────────────────────────────────

    def observe(self, name, value):
        """Record one sample (ms, or any scalar) into the named series."""
        with self._lock:
            s = self._series.get(name)
            if s is None:
                s = self._series[name] = Series(name)
            s.add(value)

    @contextmanager
    def timer(self, name):
        """`with perf.timer("vel.infer_ms"): ...` — records elapsed ms."""
        t0 = time.perf_counter()
        try:
            yield
        finally:
            self.observe(name, (time.perf_counter() - t0) * 1000.0)

    def mark(self, name, t=None):
        """
        Record an event for rate tracking, and the interval since the previous
        mark as `<name>_interval_ms`. Use for loop cadence / frame arrival.
        """
        t = t if t is not None else time.monotonic()
        with self._lock:
            rc = self._rates.get(name)
            if rc is None:
                rc = self._rates[name] = RateCounter()
            rc.tick(t)
            prev = self._last_stamp.get(name)
            self._last_stamp[name] = t
        if prev is not None:
            gap = (t - prev) * 1000.0
            if gap < 10000.0:      # ignore restarts / long idles
                self.observe(name + "_interval_ms", gap)

    # ── detection quality (label-free proxies) ────────────────────────────────

    def record_detection_frame(self, detections, e2e_ms=None, source="oak"):
        """
        Call once per inference frame with the detector's output list
        (dicts with 'label', 'conf', and optionally 'xyz_m').

        Tracks: detections/frame, confidence mean and low tail, fraction of
        boxes with a valid 3D back-projection, empty-frame rate, and per-label
        flicker (present → absent for 1-2 frames → present again), which is the
        strongest label-free signal of intermittent missed detections.
        """
        dets = list(detections or [])
        confs = [float(d.get("conf", 0.0)) for d in dets]
        with_depth = sum(1 for d in dets if d.get("xyz_m"))
        now = time.monotonic()

        with self._lock:
            self._det_frame_count += 1
            self._det_frames.append({
                "t": now,
                "n": len(dets),
                "confs": confs,
                "with_depth": with_depth,
            })
            labels = {d.get("label") for d in dets if d.get("label")}
            # keep presence history bounded to labels actually seen recently
            for lab in set(self._det_presence) | labels:
                hist = self._det_presence[lab]
                hist.append(1 if lab in labels else 0)
                # flicker = 1..2 absent frames wedged between two present frames
                if len(hist) >= 3 and hist[-1] == 1:
                    if hist[-2] == 0 and hist[-3] == 1:
                        self._det_flickers[lab] += 1
                    elif len(hist) >= 4 and hist[-2] == 0 and hist[-3] == 0 and hist[-4] == 1:
                        self._det_flickers[lab] += 1

        self.mark(f"{source}.det")
        if e2e_ms is not None:
            self.observe(f"{source}.det_e2e_ms", e2e_ms)

    def record_labeled_eval(self, precision=None, recall=None, map50=None,
                            n_images=None, source="offline"):
        """
        Fold in a labelled detection score computed offline (annotated set, or a
        rosbag replayed against a stronger reference model). Kept alongside the
        live proxies so one snapshot answers "is it accurate?" and "is it
        degrading right now?".
        """
        with self._lock:
            self._labeled_eval = {
                "precision": precision,
                "recall": recall,
                "map50": map50,
                "n_images": n_images,
                "source": source,
                "at": datetime.now().isoformat(timespec="seconds"),
            }

    def _detection_summary(self):
        frames = list(self._det_frames)
        if not frames:
            return {"n_frames": 0}
        n_det = [f["n"] for f in frames]
        all_conf = [c for f in frames for c in f["confs"]]
        total_boxes = sum(n_det)
        total_depth = sum(f["with_depth"] for f in frames)
        conf_sorted = sorted(all_conf)
        span = frames[-1]["t"] - frames[0]["t"]
        flick = sum(self._det_flickers.values())
        return {
            "n_frames": len(frames),
            "det_per_frame": round(total_boxes / len(frames), 2),
            "empty_frame_pct": round(100.0 * sum(1 for n in n_det if n == 0) / len(frames), 1),
            "conf_mean": round(sum(all_conf) / len(all_conf), 3) if all_conf else 0.0,
            "conf_p10": round(_pct(conf_sorted, 0.10), 3),
            "conf_p50": round(_pct(conf_sorted, 0.50), 3),
            "depth_valid_pct": round(100.0 * total_depth / total_boxes, 1) if total_boxes else 0.0,
            "flicker_total": flick,
            # Per-frame is the stable form; the per-minute rate needs a window
            # wide enough not to explode on a near-zero span (replay/startup).
            "flicker_per_100f": round(100.0 * flick / len(frames), 2),
            "flicker_per_min": round(60.0 * flick / span, 2) if span >= 2.0 else None,
            "flicker_labels": dict(sorted(self._det_flickers.items(),
                                          key=lambda kv: -kv[1])[:5]),
            "labeled": self._labeled_eval,
        }

    # ── velocity MLP accuracy (self-supervised) ───────────────────────────────

    def record_velocity_sample(self, track_id, t, world_x, world_y, robot_theta,
                               pred_vx=None, pred_vy=None):
        """
        Record one frame of a track: its WORLD-frame position, the robot heading
        at that instant, and (optionally) the model's prediction for that frame.

        Predictions are scored lazily — once the stream contains samples about
        LABEL_HORIZON_S on *both* sides of t, the ground-truth velocity is the
        central difference of world position over that interval, rotated into
        the robot frame the prediction was made in.
        """
        with self._lock:
            stream = self._vel_streams.get(track_id)
            if stream is None:
                stream = self._vel_streams[track_id] = deque(maxlen=TRACK_STREAM_N)
            stream.append({
                "t": float(t), "wx": float(world_x), "wy": float(world_y),
                "th": float(robot_theta),
                "pvx": None if pred_vx is None else float(pred_vx),
                "pvy": None if pred_vy is None else float(pred_vy),
                "scored": False,
            })
            self._score_stream(track_id, stream)
            self._reap_streams(float(t))

    def _score_stream(self, track_id, stream):
        """Score any pending prediction that now has enough future context."""
        if len(stream) < 3:
            return
        t_now = stream[-1]["t"]
        pending = 0
        for i, s in enumerate(stream):
            if s["scored"] or s["pvx"] is None:
                continue
            if t_now - s["t"] < LABEL_HORIZON_S:
                pending += 1
                continue          # future not here yet
            before = self._nearest(stream, s["t"] - LABEL_HORIZON_S, 0, i)
            after = self._nearest(stream, s["t"] + LABEL_HORIZON_S, i, len(stream))
            if before is None or after is None:
                s["scored"] = True          # unlabelable (track too short / gap)
                self._vel_dropped += 1
                continue
            dt = after["t"] - before["t"]
            if dt <= 1e-3:
                s["scored"] = True
                self._vel_dropped += 1
                continue
            # World-frame ground truth, then rotate into the robot frame at t
            # (model's output frame: x forward, y left).
            gvx_w = (after["wx"] - before["wx"]) / dt
            gvy_w = (after["wy"] - before["wy"]) / dt
            c, sn = math.cos(s["th"]), math.sin(s["th"])
            gt_vx = c * gvx_w + sn * gvy_w
            gt_vy = -sn * gvx_w + c * gvy_w
            s["scored"] = True
            self._vel_scored.append({
                "tid": track_id,
                "t": s["t"],
                "pvx": s["pvx"], "pvy": s["pvy"],
                "gvx": gt_vx, "gvy": gt_vy,
            })
        self._vel_pending = pending

    @staticmethod
    def _nearest(stream, t_target, lo, hi):
        """Sample in stream[lo:hi] closest to t_target, within tolerance."""
        best, best_d = None, LABEL_TOLERANCE_S
        for j in range(lo, hi):
            d = abs(stream[j]["t"] - t_target)
            if d <= best_d:
                best, best_d = stream[j], d
        return best

    def _reap_streams(self, t_now):
        for tid in [k for k, v in self._vel_streams.items()
                    if v and t_now - v[-1]["t"] > 3.0]:
            del self._vel_streams[tid]

    def _velocity_summary(self):
        rows = list(self._vel_scored)
        if not rows:
            return {"n": 0, "pending": self._vel_pending, "unlabelable": self._vel_dropped}
        n = len(rows)
        ex = [r["pvx"] - r["gvx"] for r in rows]
        ey = [r["pvy"] - r["gvy"] for r in rows]
        mag = [math.hypot(a, b) for a, b in zip(ex, ey)]
        gt_speed = [math.hypot(r["gvx"], r["gvy"]) for r in rows]
        pr_speed = [math.hypot(r["pvx"], r["pvy"]) for r in rows]
        speed_err = [p - g for p, g in zip(pr_speed, gt_speed)]

        moving = [i for i, g in enumerate(gt_speed) if g >= MOVING_THRESH_MS]
        head_err = []
        for i in moving:
            if pr_speed[i] < 1e-3:
                continue
            r = rows[i]
            dot = (r["pvx"] * r["gvx"] + r["pvy"] * r["gvy"]) / (pr_speed[i] * gt_speed[i])
            head_err.append(math.degrees(math.acos(max(-1.0, min(1.0, dot)))))

        # R² of predicted vs true, pooled over both axes — 1.0 perfect, <=0 means
        # the model is no better than always predicting the mean.
        obs = [r["gvx"] for r in rows] + [r["gvy"] for r in rows]
        res = ex + ey
        mu = sum(obs) / len(obs)
        ss_tot = sum((o - mu) ** 2 for o in obs)
        ss_res = sum(e * e for e in res)
        r2 = round(1.0 - ss_res / ss_tot, 3) if ss_tot > 1e-9 else None

        def _mae(v):
            return round(sum(abs(x) for x in v) / len(v), 3)

        return {
            "n": n,
            "pending": self._vel_pending,
            "unlabelable": self._vel_dropped,
            "mae_vx": _mae(ex),
            "mae_vy": _mae(ey),
            "rmse": round(math.sqrt(sum(m * m for m in mag) / n), 3),
            "bias_vx": round(sum(ex) / n, 3),
            "bias_vy": round(sum(ey) / n, 3),
            "speed_mae": _mae(speed_err),
            "speed_bias": round(sum(speed_err) / n, 3),
            "heading_mae_deg": round(sum(head_err) / len(head_err), 1) if head_err else None,
            "moving_pct": round(100.0 * len(moving) / n, 1),
            "gt_speed_mean": round(sum(gt_speed) / n, 3),
            "r2": r2,
        }

    # ── export ────────────────────────────────────────────────────────────────

    def snapshot(self):
        """Full metrics dict — safe to serialise straight into telemetry."""
        with self._lock:
            lat = {k: v.summary() for k, v in sorted(self._series.items())}
            rates = {k: v.rate() for k, v in sorted(self._rates.items())}
            det = self._detection_summary()
            vel = self._velocity_summary()
            uptime = round(time.monotonic() - self._started, 1)
        snap = {
            "uptime_s": uptime,
            "latency": lat,
            "rates_hz": rates,
            "detection": det,
            "velocity_mlp": vel,
        }
        self._maybe_log(snap)
        return snap

    def brief(self):
        """Compact dict for the 2 Hz telemetry lane / GUI badge."""
        with self._lock:
            det = self._detection_summary()
            vel = self._velocity_summary()
            nn = self._series.get("oak.nn_decode_ms")
            e2e = self._series.get("oak.det_e2e_ms")
            cyc = self._series.get("vel.cycle_ms")
            inf = self._series.get("vel.infer_ms")
            det_rate = self._rates.get("oak.det")
            vel_rate = self._rates.get("vel.cycle")
            g = lambda s, k: (s.summary().get(k, 0.0) if s else 0.0)
            out = {
                "det_hz": det_rate.rate() if det_rate else 0.0,
                "det_nn_p95_ms": g(nn, "p95"),
                "det_e2e_p95_ms": g(e2e, "p95"),
                "det_conf_mean": det.get("conf_mean", 0.0),
                "det_per_frame": det.get("det_per_frame", 0.0),
                "det_depth_valid_pct": det.get("depth_valid_pct", 0.0),
                "det_flicker_per_100f": det.get("flicker_per_100f", 0.0),
                "det_flicker_per_min": det.get("flicker_per_min"),
                "det_labeled": det.get("labeled"),
                "vel_hz": vel_rate.rate() if vel_rate else 0.0,
                "vel_cycle_p95_ms": g(cyc, "p95"),
                "vel_infer_p95_ms": g(inf, "p95"),
                "vel_n_scored": vel.get("n", 0),
                "vel_rmse": vel.get("rmse"),
                "vel_mae_vx": vel.get("mae_vx"),
                "vel_mae_vy": vel.get("mae_vy"),
                "vel_speed_bias": vel.get("speed_bias"),
                "vel_heading_mae_deg": vel.get("heading_mae_deg"),
                "vel_r2": vel.get("r2"),
            }
        # brief() is the call the server makes on every telemetry tick, so it is
        # what keeps the JSONL trend file advancing (throttled to one line per
        # SNAPSHOT_LOG_INTERVAL_S; the full snapshot is only built when due).
        self._maybe_log_lazy()
        return out

    def _maybe_log_lazy(self):
        if not self._log_enabled:
            return
        if time.monotonic() - self._last_log < SNAPSHOT_LOG_INTERVAL_S:
            return
        self.snapshot()

    def _maybe_log(self, snap):
        if not self._log_enabled:
            return
        now = time.monotonic()
        if now - self._last_log < SNAPSHOT_LOG_INTERVAL_S:
            return
        self._last_log = now
        try:
            if self._log_path is None:
                LOG_DIR.mkdir(parents=True, exist_ok=True)
                stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                self._log_path = LOG_DIR / f"perf_{stamp}.jsonl"
                logger.info(f"PerfMonitor: logging snapshots to {self._log_path}")
            line = dict(snap)
            line["wall"] = datetime.now().isoformat(timespec="seconds")
            with open(self._log_path, "a") as f:
                f.write(json.dumps(line) + "\n")
        except Exception as e:
            logger.warning(f"PerfMonitor: snapshot log failed: {e}")
            self._log_enabled = False

    def reset(self):
        """Clear all accumulated stats (e.g. when swapping models)."""
        with self._lock:
            self._series.clear()
            self._rates.clear()
            self._last_stamp.clear()
            self._det_frames.clear()
            self._det_presence.clear()
            self._det_flickers.clear()
            self._det_frame_count = 0
            self._vel_streams.clear()
            self._vel_scored.clear()
            self._vel_pending = 0
            self._vel_dropped = 0
            self._started = time.monotonic()


_monitor = None
_monitor_lock = threading.Lock()


def get_monitor():
    """Process-wide PerfMonitor singleton."""
    global _monitor
    if _monitor is None:
        with _monitor_lock:
            if _monitor is None:
                _monitor = PerfMonitor()
    return _monitor
