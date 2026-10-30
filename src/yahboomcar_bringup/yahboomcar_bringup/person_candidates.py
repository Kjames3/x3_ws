"""Person-sized clusters in one Livox frame, for the camera-seeded lidar person tracker.

Pure numpy (scipy optional), no ROS, so tests can import it.  The rules are the GUI
association's (src/web/person-cloud-tracker.js) run on the full 10 Hz frame instead of
the 5 Hz, 6000-point display cloud:

  * keep points 0.15-2.2 m above the floor (drops the floor and the ceiling);
  * connect occupied cells of an 18 cm XY grid (8-neighbour).  XY connectivity bridges
    the gaps of the non-repetitive pattern along a body;
  * components are built over the WHOLE scene before any size test, so a wall is
    rejected as a wall instead of being cropped into person-sized pieces;
  * a candidate needs >= 8 points, >= 35 cm of vertical extent, and an XY span of
    12-90 cm.

A candidate is only a shape.  Clusters alone never make a person: the tracker
(src/lidar_person_tracker.py) starts a track only where a camera detection lands.
"""
import numpy as np

try:
    from scipy import ndimage as _ndimage
except Exception:          # the fallback below is slower but needs nothing
    _ndimage = None

CELL_M = 0.18
Z_MIN_M = 0.15
Z_MAX_M = 2.2
RANGE_MAX_M = 8.0
MIN_POINTS = 8
MIN_HEIGHT_M = 0.35
MAX_SPAN_M = 0.90
MIN_SPAN_M = 0.12

# Order of the float32 fields of /livox/person_candidates (PointCloud2, frame odom).
FIELDS = ('x', 'y', 'x_lo', 'x_hi', 'y_lo', 'y_hi', 'z_lo', 'z_hi', 'n')


def _label(occ):
    """8-connected component labels of a 2D bool grid (0 = empty)."""
    if _ndimage is not None:
        lab, _ = _ndimage.label(occ, structure=np.ones((3, 3), dtype=bool))
        return lab
    lab = np.zeros(occ.shape, dtype=np.int32)
    h, w = occ.shape
    n = 0
    for i0, j0 in zip(*np.nonzero(occ)):
        if lab[i0, j0]:
            continue
        n += 1
        lab[i0, j0] = n
        stack = [(i0, j0)]
        while stack:
            i, j = stack.pop()
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    a, b = i + di, j + dj
                    if 0 <= a < h and 0 <= b < w and occ[a, b] and not lab[a, b]:
                        lab[a, b] = n
                        stack.append((a, b))
    return lab


def extract(x, y, z, origin=(0.0, 0.0), cell=CELL_M, z_min=Z_MIN_M, z_max=Z_MAX_M,
            range_max=RANGE_MAX_M, min_points=MIN_POINTS, min_height=MIN_HEIGHT_M,
            max_span=MAX_SPAN_M, min_span=MIN_SPAN_M):
    """x, y, z: point coordinates (any planar frame, z = height above the floor).
    origin: the robot's XY in that frame, for the range limit.
    Returns an (N, 9) float32 array, columns FIELDS: median XY, XY/Z bounds, count."""
    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    z = np.asarray(z, dtype=np.float64)
    dx, dy = x - origin[0], y - origin[1]
    keep = (np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & (z > z_min) & (z < z_max)
            & (dx * dx + dy * dy < range_max * range_max))
    x, y, z = x[keep], y[keep], z[keep]
    if len(x) < min_points:
        return np.zeros((0, len(FIELDS)), dtype=np.float32)
    ix = np.floor((x - origin[0] + range_max) / cell).astype(np.int64)
    iy = np.floor((y - origin[1] + range_max) / cell).astype(np.int64)
    size = int(np.ceil(2 * range_max / cell)) + 1
    occ = np.zeros((size, size), dtype=bool)
    occ[ix, iy] = True
    lab = _label(occ)[ix, iy]
    order = np.argsort(lab, kind='stable')
    lab, x, y, z = lab[order], x[order], y[order], z[order]
    starts = np.flatnonzero(np.r_[True, lab[1:] != lab[:-1]])
    ends = np.r_[starts[1:], len(lab)]
    out = []
    for s, e in zip(starts, ends):
        if e - s < min_points:
            continue
        xs, ys, zs = x[s:e], y[s:e], z[s:e]
        x_lo, x_hi, y_lo, y_hi = xs.min(), xs.max(), ys.min(), ys.max()
        z_lo, z_hi = zs.min(), zs.max()
        span = max(x_hi - x_lo, y_hi - y_lo)
        if span > max_span or span < min_span or z_hi - z_lo < min_height:
            continue
        out.append((np.median(xs), np.median(ys), x_lo, x_hi, y_lo, y_hi, z_lo, z_hi, e - s))
    if not out:
        return np.zeros((0, len(FIELDS)), dtype=np.float32)
    return np.asarray(out, dtype=np.float32)
