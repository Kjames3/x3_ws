"""Capture-time pose alignment and the velocity MLP's 10 Hz input contract."""
from bisect import bisect_left
from collections import deque
import math
import threading

import numpy as np

MODEL_PERIOD_S = 0.1  # Thor-Magni training CSVs are native 10 Hz.
MAX_SAMPLE_GAP_S = 0.25


def resample_positions(samples, count=10):
    """Return a complete, latest-anchored 10 Hz window, or None.

    No padding/extrapolation and no interpolation across gaps over 250 ms.
    Samples are (monotonic capture seconds, global xyz). Missing observations
    must not be appended as if measured.
    """
    if len(samples) < 2:
        return None
    ts = np.asarray([s[0] for s in samples], dtype=float)
    xyz = np.asarray([s[1] for s in samples], dtype=float)
    if not np.isfinite(ts).all() or not np.isfinite(xyz).all():
        return None
    grid = ts[-1] - np.arange(count - 1, -1, -1) * MODEL_PERIOD_S
    if grid[0] < ts[0] - 1e-7 or np.any(np.diff(ts) <= 0):
        return None
    first = max(0, np.searchsorted(ts, grid[0], side='right') - 1)
    if np.any(np.diff(ts[first:]) > MAX_SAMPLE_GAP_S + 1e-7):
        return None
    return np.column_stack([np.interp(grid, ts, xyz[:, i]) for i in range(3)])


class PoseHistory:
    """Bounded odometry history; interpolate only within supported timestamps."""
    def __init__(self):
        self._samples = deque(maxlen=400)
        self._lock = threading.Lock()

    def add(self, stamp, x, y, yaw):
        if not all(math.isfinite(v) for v in (stamp, x, y, yaw)):
            return
        with self._lock:
            if self._samples and stamp < self._samples[-1][0]:
                self._samples.clear()  # clock reset: never splice time domains
            if self._samples and stamp == self._samples[-1][0]:
                return
            self._samples.append((stamp, x, y, yaw))

    def at(self, stamp, max_gap_s=0.15):
        with self._lock:
            samples = list(self._samples)
        if not samples or not math.isfinite(stamp):
            return None
        i = bisect_left([s[0] for s in samples], stamp)
        if i < len(samples) and samples[i][0] == stamp:
            return dict(zip(('x', 'y', 'theta'), samples[i][1:]))
        if i == 0 or i == len(samples):
            return None
        a, b = samples[i - 1], samples[i]
        if b[0] - a[0] > max_gap_s:
            return None
        f = (stamp - a[0]) / (b[0] - a[0])
        dyaw = math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3]))
        return {'x': a[1] + f * (b[1] - a[1]),
                'y': a[2] + f * (b[2] - a[2]), 'theta': a[3] + f * dyaw}


def ros_stamp_ns(ros_now_ns, mono_now, capture_t):
    """ROS time (ns) of a monotonic capture instant, or None when it is unusable.

    ``ros_now_ns`` and ``mono_now`` must be read together; the capture is placed
    at its age before that moment. Rejects negative or implausibly old ages.
    """
    if capture_t is None or not math.isfinite(capture_t):
        return None
    age = mono_now - capture_t
    if not 0.0 <= age <= 5.0:
        return None
    return int(ros_now_ns - age * 1e9)

