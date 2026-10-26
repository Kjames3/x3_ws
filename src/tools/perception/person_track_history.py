"""Per-person position history -> velocity, for the A/B test's predictive mode.

The A/B test used to take each person's velocity from the latest readout alone.
Those single-frame estimates are noisy: one walker read 0.06 -> 0.65 -> 1.07 ->
0.01 m/s within ~1.5 s, and a standing person up to 0.44 m/s (2026-10-26 run).
Projecting that forward made predictive mode react to noise.

Here each person's position is kept in the map frame (robot motion removed)
and the velocity is the least-squares slope over the last WINDOW_S.  It is
returned in the robot frame, the frame the test's avoidance logic uses.  Until a
track has MIN_SPAN_S of history its velocity is unknown and reported as zero, so
predictive mode treats a new person like reactive mode does instead of acting on
a guess.

Estimates arrive in the camera frame used by the server's readout: ``x``
right, ``z`` forward.  Robot frame is x forward, y left: (rx, ry) = (z, -x).
"""

import math

WINDOW_S = 1.0         # fit window
MIN_SPAN_S = 0.4       # history needed before a velocity is trusted
MIN_SAMPLES = 4
MAX_SPEED = 2.5        # m/s; a faster fit is a bad association, not a person
LOST_S = 0.5           # a track unseen this long is dropped...
REATTACH_M = 0.35      # ...unless a new ID appears this close to where it was


def camera_to_robot(est):
    """(rx, ry) in base_link from a readout estimate."""
    return float(est.get("z", est.get("y", 0.0))), -float(est.get("x", 0.0))


class PersonTrackHistory:
    def __init__(self, window_s=WINDOW_S):
        self.window_s = window_s
        self._tracks = {}      # id -> list of (t, mx, my)
        self._last_seen = {}   # id -> t
        self._last_raw = {}    # id -> (x, z) as received

    def update(self, estimates, pose, t):
        """Add one readout.  ``pose`` = (x, y, yaw) of base_link in the map frame."""
        px, py, pyaw = pose
        c, s = math.cos(pyaw), math.sin(pyaw)
        ests = [e for e in estimates if e.get("id") is not None]
        # Known IDs first, so a new ID can only inherit a track nobody claimed.
        ests.sort(key=lambda e: e["id"] not in self._tracks)
        for est in ests:
            tid = est["id"]
            raw = (est.get("x"), est.get("z", est.get("y")))
            if tid not in self._tracks:
                rx, ry = camera_to_robot(est)
                self._tracks[tid] = self._reattach(px + rx * c - ry * s,
                                                   py + rx * s + ry * c, t)
            hist = self._tracks[tid]
            # The readout repeats the last estimate between estimator updates.
            # Compare the raw values: in the map frame a repeat would look like
            # the person moving whenever the robot does.
            if self._last_raw.get(tid) != raw:
                rx, ry = camera_to_robot(est)
                hist.append((t, px + rx * c - ry * s, py + rx * s + ry * c))
                self._last_raw[tid] = raw
            while hist and t - hist[0][0] > self.window_s:
                hist.pop(0)
            self._last_seen[tid] = t
        for tid in [k for k, ts in self._last_seen.items() if t - ts > LOST_S]:
            self._tracks.pop(tid, None)
            self._last_seen.pop(tid, None)
            self._last_raw.pop(tid, None)

    def _reattach(self, mx, my, t):
        """History of a recently lost track near (mx, my): the estimator's IDs churn."""
        best, best_d = None, REATTACH_M
        for tid, hist in self._tracks.items():
            if not hist or t - self._last_seen.get(tid, -1e9) <= 0.0:
                continue           # still live under its own ID this readout
            d = math.hypot(hist[-1][1] - mx, hist[-1][2] - my)
            if d < best_d:
                best, best_d = tid, d
        if best is None:
            return []
        self._last_seen.pop(best, None)
        self._last_raw.pop(best, None)
        return self._tracks.pop(best)

    def velocity_map(self, tid):
        """(vx, vy) in the map frame, or None while the history is too short."""
        hist = self._tracks.get(tid)
        if not hist or len(hist) < MIN_SAMPLES or hist[-1][0] - hist[0][0] < MIN_SPAN_S:
            return None
        t0 = hist[0][0]
        ts = [h[0] - t0 for h in hist]
        tm = sum(ts) / len(ts)
        den = sum((x - tm) ** 2 for x in ts)
        if den <= 0.0:
            return None
        vx = sum((x - tm) * h[1] for x, h in zip(ts, hist)) / den
        vy = sum((x - tm) * h[2] for x, h in zip(ts, hist)) / den
        if math.hypot(vx, vy) > MAX_SPEED:
            return None
        return vx, vy

    def tracked(self, estimates, pose):
        """Copies of ``estimates`` with vx/vy replaced by the fitted velocity.

        Velocity is in robot axes, like the estimator's own vx/vy.  Adds
        ``vel_ok`` (False = not enough history, velocity reported as 0) and keeps
        the estimator's values as ``vx_raw`` / ``vy_raw`` for the CSV.
        """
        _, _, pyaw = pose
        c, s = math.cos(pyaw), math.sin(pyaw)
        out = []
        for est in estimates:
            e = dict(est)
            e["vx_raw"], e["vy_raw"] = est.get("vx", 0.0), est.get("vy", 0.0)
            v = self.velocity_map(est.get("id"))
            if v is None:
                e["vx"], e["vy"], e["vel_ok"] = 0.0, 0.0, False
            else:
                vx, vy = v
                e["vx"], e["vy"] = vx * c + vy * s, -vx * s + vy * c
                e["vel_ok"] = True
            e["speed"] = math.hypot(e["vx"], e["vy"])
            out.append(e)
        return out
