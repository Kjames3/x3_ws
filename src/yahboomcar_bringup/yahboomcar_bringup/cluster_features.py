"""Shape features of one person candidate, for a lidar person classifier.

Pure numpy, no ROS, so the offline dataset tools, the tests and (later) livox_scan_node
can all share one definition.

A candidate (person_candidates.extract) is only a summary row.  ``crop`` takes back the
points it covers: every point between Z_MIN_M and Z_MAX_M inside its XY bounds (+PAD_M).
For a split piece that box is its core, so a wall it was cut from contributes only the
points inside the same box, which is what the extractor saw.

``features`` describes the cropped points from the sensor's point of view.  Everything
that depends on the sensor (its height, its vertical FOV) is a parameter, so the same
code can describe the Livox at 0.31 m and SiT's Velodynes at their own height:

  * range and point count, and n * r^2, which is roughly the visible surface area
    (angular point density is constant, so counts fall as 1/r^2);
  * height: lowest/highest point, mean, spread, and the share of points in five
    height bands (legs, hips, torso, shoulders, head);
  * width across the line of sight and depth along it, overall and for the legs
    (below 0.8 m) and the upper body (above 0.9 m) separately: a person is ~0.3-0.6 m
    wide and only ~0.1-0.3 m deep as the lidar sees them;
  * how much of the FOV window at that range the cluster fills: the top of the window
    (where the upper FOV edge meets the body's near surface) and its bottom (where the
    lowest beam reaches; the Livox does not see the floor nearer than ~2.35 m).  A
    person spans the window; a chair or a table does not.

No reflectivity: it is sensor-specific (Livox vs Velodyne) and is handled separately.
"""
import numpy as np

Z_MIN_M = 0.15
Z_MAX_M = 2.2
PAD_M = 0.02
BANDS_M = (0.15, 0.5, 0.9, 1.3, 1.7, 2.2)
LEGS_TOP_M = 0.8
UPPER_BOTTOM_M = 0.9

# Livox Mid-360 at 0.31 m (see person_candidates.py).
LIVOX = {'sensor_z': 0.31, 'fov_top_tan': 1.28, 'fov_bottom_tan': 0.1317}

NAMES = (
    'range', 'n', 'log_n', 'area',
    'z_lo', 'z_hi', 'height', 'z_mean', 'z_std',
    'band_0', 'band_1', 'band_2', 'band_3', 'band_4',
    'width', 'depth', 'width_legs', 'depth_legs', 'width_upper', 'depth_upper',
    'window_top', 'top_fill', 'window_bottom', 'bottom_gap',
)


def crop(x, y, z, cand, pad=PAD_M):
    """Mask of the points one candidate covers.  cand: a person_candidates row
    (x, y, x_lo, x_hi, y_lo, y_hi, z_lo, z_hi, n)."""
    return ((z > Z_MIN_M) & (z < Z_MAX_M)
            & (x >= cand[2] - pad) & (x <= cand[3] + pad)
            & (y >= cand[4] - pad) & (y <= cand[5] + pad))


def _extent(a):
    return float(np.percentile(a, 95) - np.percentile(a, 5)) if len(a) >= 3 else 0.0


def features(x, y, z, origin=(0.0, 0.0), sensor=LIVOX):
    """x, y, z: the candidate's points (z above the floor), any planar frame.
    origin: the sensor's XY in that frame.  Returns float32 array, order NAMES."""
    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    z = np.asarray(z, dtype=np.float64)
    n = len(x)
    out = np.zeros(len(NAMES), dtype=np.float32)
    if n == 0:
        return out
    dx, dy = x - origin[0], y - origin[1]
    cx, cy = float(np.median(dx)), float(np.median(dy))
    r = max(float(np.hypot(cx, cy)), 1e-3)
    ux, uy = cx / r, cy / r                   # along the line of sight
    along = dx * ux + dy * uy
    across = -dx * uy + dy * ux
    legs, upper = z < LEGS_TOP_M, z > UPPER_BOTTOM_M
    hist = np.histogram(z, bins=BANDS_M)[0] / n
    r_near = float(np.hypot(dx, dy).min())
    top = min(Z_MAX_M, sensor['sensor_z'] + r_near * sensor['fov_top_tan'])
    bottom = max(Z_MIN_M, sensor['sensor_z'] - r * sensor['fov_bottom_tan'])
    z_lo, z_hi = float(z.min()), float(z.max())
    out[:] = (
        r, n, np.log1p(n), n * r * r / 100.0,
        z_lo, z_hi, z_hi - z_lo, z.mean(), z.std(),
        *hist,
        _extent(across), _extent(along),
        _extent(across[legs]), _extent(along[legs]),
        _extent(across[upper]), _extent(along[upper]),
        top, z_hi / top, bottom, z_lo - bottom,
    )
    return out
