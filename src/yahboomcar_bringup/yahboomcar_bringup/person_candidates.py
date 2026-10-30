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
    12-90 cm;
  * a component too wide to be a person is split once more on its points above
    SPLIT_Z_M only.  In the first lab trial (2026-10-30) a person crossing ~0.5 m in
    front of a bench line was merged into it in 2 of 3 frames: ~230 points within
    0.3 m of their centre, almost nothing at 0.3-0.5 m, furniture (never above
    ~1 m) beyond, and a few stray returns in the gap bridged the 18 cm grid.  Above
    1.1 m the furniture is gone.  A piece from the split must also be ISOLATED: few
    points in a ring round it at the same heights, which a wall cropped into a
    person-sized piece never is (the wall carries on through the ring).

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
SPLIT_Z_M = 1.1          # split oversized components on their points above this height
SPLIT_MIN_POINTS = 12
CORE_R_M = 0.3           # a split piece is the points within this of its upper-body centre
RING_M = (0.4, 0.55)     # isolation ring round that centre
RING_MAX_FRAC = 0.2      # ring points (above SPLIT_Z_M) / core points (above SPLIT_Z_M)

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


def _split(xs, ys, zs, cell, max_span, min_span):
    """Person-sized, isolated pieces of an oversized component, found on its points
    above SPLIT_Z_M.  Each piece is reported as every point (any height) within
    CORE_R_M of its upper-body centre."""
    up = zs > SPLIT_Z_M
    if up.sum() < SPLIT_MIN_POINTS:
        return []
    ux, uy = xs[up], ys[up]
    ix = np.floor(ux / cell).astype(np.int64)
    iy = np.floor(uy / cell).astype(np.int64)
    ix0, iy0 = ix.min(), iy.min()
    occ = np.zeros((ix.max() - ix0 + 1, iy.max() - iy0 + 1), dtype=bool)
    occ[ix - ix0, iy - iy0] = True
    lab = _label(occ)[ix - ix0, iy - iy0]
    out = []
    for k in np.unique(lab):
        m = lab == k
        if m.sum() < SPLIT_MIN_POINTS:
            continue
        px, py = ux[m], uy[m]
        if max(px.max() - px.min(), py.max() - py.min()) > max_span:
            continue
        cx, cy = np.median(px), np.median(py)
        du = np.hypot(ux - cx, uy - cy)
        core_up = (du < CORE_R_M).sum()
        ring_up = ((du >= RING_M[0]) & (du < RING_M[1])).sum()
        if core_up < SPLIT_MIN_POINTS or ring_up > RING_MAX_FRAC * core_up:
            continue
        core = np.hypot(xs - cx, ys - cy) < CORE_R_M
        cxs, cys, czs = xs[core], ys[core], zs[core]
        span = max(cxs.max() - cxs.min(), cys.max() - cys.min())
        if span < min_span:
            continue
        out.append((np.median(cxs), np.median(cys), cxs.min(), cxs.max(), cys.min(), cys.max(),
                    czs.min(), czs.max(), core.sum()))
    return out


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
        if span > max_span:
            out.extend(_split(xs, ys, zs, cell, max_span, min_span))
            continue
        if span < min_span or z_hi - z_lo < min_height:
            continue
        out.append((np.median(xs), np.median(ys), x_lo, x_hi, y_lo, y_hi, z_lo, z_hi, e - s))
    if not out:
        return np.zeros((0, len(FIELDS)), dtype=np.float32)
    return np.asarray(out, dtype=np.float32)
