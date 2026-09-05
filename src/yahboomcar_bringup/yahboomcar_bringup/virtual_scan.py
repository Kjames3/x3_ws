"""Synthesize a horizontal LaserScan from the tilting lidar's deskewed clouds.

Continuous sweeping gates /scan off permanently (lidar_3d_processor_node only
forwards a scan captured level and settled), which removes the obstacle feed
the CBF and Nav2 both read.  This module rebuilds that feed from the 3D cloud
instead of the raw scan.

The geometry is not symmetric, and that drives every decision here.  The tilt
axis is +Y, so a beam at bearing psi leaves the laser along

    (cos(psi) * cos(theta),  sin(psi),  -cos(psi) * sin(theta))

Beams at psi = +/-90 deg are ON the tilt axis: they are invariant under theta
and stay in the laser's horizontal plane for the whole sweep.  Beams near
psi = 0 or 180 deg get the full pitch.  So lateral bearings are refreshed every
scan while forward and rear bearings are only usable near a level crossing.
A thin horizontal slice therefore starves exactly the sector that matters for
driving forward, which is why `synthesize` projects a height BAND by column
instead: any return whose height falls in the robot's collision envelope counts
as an obstacle at its horizontal distance, whatever pitch produced it.

`ScanAccumulator` then holds each bearing's most recent return so a bearing
that is only illuminated twice per sweep still reports something between
passes.  It stamps every bin with the age of the measurement rather than
silently presenting stale data as current -- a frozen obstacle set with no age
signal is the F7 failure (see notes/, project_lidar_3d_audit): the CBF pushed
at 30 Hz against a ghost it could no longer re-measure.
"""

import numpy as np

#: Returns below this height are floor, not obstacles.  The measured floor
#: spread on the deskewed cloud is p10 -0.062 m to p90 +0.089 m about a
#: +0.003 m median, so a band starting under ~0.10 m re-detects the floor as
#: an obstacle at the sweep's downward intercept.  This is the same trap the
#: velocity estimator hit before ground-plane removal.
DEFAULT_Z_MIN = 0.12
#: The chassis is ~0.20 m tall and the laser plane sits at 0.313 m.  Anything
#: above this passes over the robot.
DEFAULT_Z_MAX = 0.40


def synthesize(points, n_bins=360, z_min=DEFAULT_Z_MIN, z_max=DEFAULT_Z_MAX,
               range_min=0.15, range_max=6.0):
    """Column-project a body-frame cloud to per-bearing nearest ranges.

    `points` are (N, 3) in a frame whose origin is the robot and whose z is
    height above the floor -- base_footprint.  Returns (ranges, counts) where
    `ranges` is `n_bins` nearest horizontal distances with inf for bearings
    that got no return, and `counts` is how many returns each bin saw.
    """
    points = np.asarray(points, dtype=float)
    ranges = np.full(n_bins, np.inf)
    counts = np.zeros(n_bins, dtype=int)
    if points.size == 0:
        return ranges, counts
    d = np.hypot(points[:, 0], points[:, 1])
    keep = ((points[:, 2] >= z_min) & (points[:, 2] <= z_max)
            & (d >= range_min) & (d <= range_max) & np.isfinite(points).all(axis=1))
    if not np.any(keep):
        return ranges, counts
    d = d[keep]
    bearing = np.arctan2(points[keep, 1], points[keep, 0])
    idx = np.floor((bearing + np.pi) / (2 * np.pi) * n_bins).astype(int) % n_bins
    # minimum.at is the reduction that gives nearest-per-bin in one pass.
    np.minimum.at(ranges, idx, d)
    np.add.at(counts, idx, 1)
    return ranges, counts


class ScanAccumulator:
    """Most-recent range per bearing, with an explicit per-bin age.

    `max_age` expires a bin outright: an obstacle that has moved away must
    stop being reported, and a sector the sweep has stopped illuminating must
    read "unknown" rather than keeping its last value forever.
    """

    def __init__(self, n_bins=360, max_age=1.0, **kw):
        self.n_bins = n_bins
        self.max_age = max_age
        self.kw = kw
        self.ranges = np.full(n_bins, np.inf)
        self.stamps = np.full(n_bins, -np.inf)

    def update(self, points, now):
        fresh, counts = synthesize(points, self.n_bins, **self.kw)
        hit = counts > 0
        # Overwrite rather than take the min against history: a bearing that
        # was measured again is authoritative, including when the new reading
        # is FARTHER (the obstacle left).  Min-with-history would make every
        # transient return permanent.
        self.ranges[hit] = fresh[hit]
        self.stamps[hit] = now
        return hit

    def read(self, now):
        """(ranges, ages) with expired bins set to inf / inf."""
        ages = now - self.stamps
        ranges = np.where(ages <= self.max_age, self.ranges, np.inf)
        return ranges, np.where(np.isfinite(self.stamps), ages, np.inf)

    def coverage(self, now):
        ranges, _ = self.read(now)
        return float(np.mean(np.isfinite(ranges)))
