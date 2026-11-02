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
    person-sized piece never is (the wall carries on through the ring);
  * within NEAR_SPLIT_R_M of the robot the split is repeated at NEAR_SPLIT_Z_M: the
    Livox's +52 deg upper FOV edge cuts a person at 0.65-0.8 m off at ~1.1-1.3 m,
    leaving nothing above 1.1 m (circle run, 2026-10-30).  Such a piece must reach
    near the top of the FOV at its range, which a chair back or a table does not.

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
SPLIT_MIN_POINTS = 8
CORE_R_M = 0.3           # a split piece is the points within this of its upper-body centre
RING_M = (0.4, 0.55)     # isolation ring round that centre
RING_MAX_FRAC = 0.2      # ring points (above SPLIT_Z_M) / core points (above SPLIT_Z_M)
NEAR_SPLIT_R_M = 1.5     # second split pass for pieces centred this close to the robot
NEAR_SPLIT_Z_M = 0.9     # ... on points above this (the lab bench tops out at ~0.86 m)
FOV_TOP_TAN = 1.28       # tan(52 deg), the Livox's upper FOV edge
SENSOR_Z_M = 0.31        # Livox optical centre above the floor
NEAR_TOP_MARGIN_M = 0.15 # a near piece must reach to within this of the FOV top (or 1.3 m)

# Head/shoulder peak split (opt-in, extract(..., peaks=True)); see _peak_split.
PEAK_TRY_SPAN_M = 0.55   # components at least this wide are checked for several people
PEAK_MAX_SPAN_M = 2.5    # ... and at most this wide: long walls cost time (uncapped: +6 points
                         # crowd recall, groups merged into a wall; 2x the time)
PEAK_CELL_M = 0.08       # height-map cell
PEAK_NMS_M = 0.3         # peaks closer than this are one person
PEAK_MIN_Z_M = 1.2       # a head/shoulder peak is at least this high (or near the FOV top)
PEAK_MAX_Z_M = 2.1       # a piece reaching this high is structure (walls, shelves, door frames)
PEAK_MIN_AREA = 300.0    # n * r^2 (points x m^2): enough surface for its range
PEAK_MAX_CANDIDATES = 40 # highest local maxima considered per component (bounds the cost)
SADDLE_DROP_M = 0.15     # two peaks are two people only if the height dips this much between them
ASSIGN_R_M = 0.45        # a peak takes the component's points within this (XY)
PEAK_RING_M = 0.6        # isolation: unassigned upper points within this of a peak
PEAK_CORE_M = 0.3        # compact: a piece's upper points lie within this of its peak ...
PEAK_CORE_FRAC = 0.85    # ... at least this share of them (a cut-up wall spreads to ASSIGN_R_M)

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


def _split(xs, ys, zs, cell, max_span, min_span, split_z=SPLIT_Z_M, origin=None):
    """Person-sized, isolated pieces of an oversized component, found on its points
    above split_z.  Each piece is reported as every point (any height) within
    CORE_R_M of its upper-body centre.  With origin, only pieces centred within
    NEAR_SPLIT_R_M of it that reach near the FOV top (the near pass)."""
    up = zs > split_z
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
        if origin is not None and np.hypot(cx - origin[0], cy - origin[1]) > NEAR_SPLIT_R_M:
            continue
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
        if origin is not None:
            # The top of the FOV meets the body on its NEAR surface.
            r = np.hypot(cxs - origin[0], cys - origin[1]).min()
            if czs.max() < min(1.3, SENSOR_Z_M + r * FOV_TOP_TAN - NEAR_TOP_MARGIN_M):
                continue
        out.append((np.median(cxs), np.median(cys), cxs.min(), cxs.max(), cys.min(), cys.max(),
                    czs.min(), czs.max(), core.sum()))
    return out


def _peak_split(xs, ys, zs, origin, max_span, min_span, min_points, min_height):
    """Several people in one component, found by their heads/shoulders.

    Height map (max z per PEAK_CELL_M cell) -> local maxima above PEAK_MIN_Z_M (lower
    near the robot, where the FOV top cuts people) -> taken highest first, a peak is a
    new person only if it is PEAK_NMS_M from every accepted one AND the height dips by
    SADDLE_DROP_M on the straight line to each accepted peak within 1.2 m (two heads have
    shoulders or a gap between them; a head and its own shoulder, or a flat shelf top, do
    not).  Each peak takes the points within ASSIGN_R_M (nearest peak wins).  A piece is
    kept if person-sized and isolated: few UPPER points (above SPLIT_Z_M) within
    PEAK_RING_M of it that no piece took, so a neighbour's shoulder does not count
    against it but a wall or shelf that carries on does.  A piece must also look like a
    person to a sensor (offline, 2026-11-02: these keep ~93% of real and synthetic
    people's pieces and drop ~2/3 of the pieces cut from clutter): compact (PEAK_CORE_FRAC
    of its upper points within PEAK_CORE_M), top between PEAK_MIN_Z_M (or the FOV top
    near the robot) and PEAK_MAX_Z_M, and n * r^2 >= PEAK_MIN_AREA.  Returns [] unless
    >= 2 pieces survive (the caller then keeps its usual path)."""
    if _ndimage is None or len(xs) < 2 * min_points:
        return []
    r0 = np.hypot(xs.mean() - origin[0], ys.mean() - origin[1])
    zmin = min(PEAK_MIN_Z_M, SENSOR_Z_M + max(r0 - 0.3, 0.3) * FOV_TOP_TAN - NEAR_TOP_MARGIN_M)
    up = (zs > zmin) & (zs < PEAK_MAX_Z_M)
    # cheap exit: two heads need head-height points at least PEAK_NMS_M apart
    if up.sum() < 2 or max(np.ptp(xs[up]), np.ptp(ys[up])) < PEAK_NMS_M:
        return []
    c = PEAK_CELL_M
    # Height map from the points near head height only: a peak is above zmin and a dip
    # must fall SADDLE_DROP_M below it, so any cell lower than zmin - 0.2 is a dip
    # whatever its exact height (empty = 0).  Max per cell by sorting (np.maximum.at is
    # slow on walls of ~10k points).
    hm = zs > zmin - SADDLE_DROP_M - 0.05
    hx, hy, hz = xs[hm], ys[hm], zs[hm]
    ix = np.floor((hx - xs.min()) / c).astype(np.int64)
    iy = np.floor((hy - ys.min()) / c).astype(np.int64)
    W = int(np.floor((ys.max() - ys.min()) / c)) + 1
    h = np.zeros((int(np.floor((xs.max() - xs.min()) / c)) + 1, W))
    lin = ix * W + iy
    o = np.lexsort((hz, lin))
    last = np.r_[lin[o][1:] != lin[o][:-1], True]
    h.flat[lin[o][last]] = hz[o][last]
    hd = _ndimage.maximum_filter(h, size=3, mode='constant')       # +-1 cell: fills sparse gaps
    # peaks at structure height are dropped before the pairwise checks (their pieces
    # would be rejected anyway, and a wall's flat top is a long row of equal maxima)
    loc = (h > zmin) & (h < PEAK_MAX_Z_M) & (h >= _ndimage.maximum_filter(h, size=5, mode='constant'))
    pi, pj = np.nonzero(loc)
    if len(pi) < 2:
        return []
    order = np.argsort(-h[pi, pj])[:PEAK_MAX_CANDIDATES]
    peaks = []
    for k in order:
        i, j = pi[k], pj[k]
        ok = True
        for (a, b) in peaks:
            d = np.hypot(i - a, j - b) * c
            if d < PEAK_NMS_M:
                ok = False
                break
            if d > 1.2:
                continue
            f = np.arange(max(2, int(d / (c / 2)))) / (max(2, int(d / (c / 2))) - 1)
            li = np.rint(i + (a - i) * f).astype(np.int64)
            lj = np.rint(j + (b - j) * f).astype(np.int64)
            if hd[li, lj].min() > min(h[i, j], h[a, b]) - SADDLE_DROP_M:
                ok = False
                break
        if ok:
            peaks.append((i, j))
    if len(peaks) < 2:
        return []
    P = np.array([[xs.min() + (i + 0.5) * c, ys.min() + (j + 0.5) * c] for i, j in peaks])
    # only points within PEAK_RING_M of some peak matter (a wall's far end does not)
    box = ((xs > P[:, 0].min() - PEAK_RING_M) & (xs < P[:, 0].max() + PEAK_RING_M)
           & (ys > P[:, 1].min() - PEAK_RING_M) & (ys < P[:, 1].max() + PEAK_RING_M))
    xs, ys, zs = xs[box], ys[box], zs[box]
    d = np.hypot(xs[:, None] - P[None, :, 0], ys[:, None] - P[None, :, 1])
    near = d.argmin(1)
    assigned = d[np.arange(len(xs)), near] < ASSIGN_R_M
    free_up = ~assigned & (zs > SPLIT_Z_M)
    out = []
    for k in range(len(P)):
        m = assigned & (near == k)
        if m.sum() < min_points:
            continue
        px, py, pz = xs[m], ys[m], zs[m]
        span = max(px.max() - px.min(), py.max() - py.min())
        if span > max_span or span < min_span or pz.max() - pz.min() < min_height:
            continue
        rk = max(float(np.hypot(P[k, 0] - origin[0], P[k, 1] - origin[1])), 0.3)
        top = min(PEAK_MIN_Z_M, SENSOR_Z_M + max(rk - 0.3, 0.3) * FOV_TOP_TAN - NEAR_TOP_MARGIN_M)
        if pz.max() >= PEAK_MAX_Z_M or pz.max() < top or m.sum() * rk * rk < PEAK_MIN_AREA:
            continue
        upm = pz > SPLIT_Z_M
        up = upm.sum()
        if up and (d[m, k][upm] < PEAK_CORE_M).mean() < PEAK_CORE_FRAC:
            continue
        ring = (free_up & (d[:, k] < PEAK_RING_M)).sum()
        if ring > RING_MAX_FRAC * max(up, SPLIT_MIN_POINTS):
            continue
        out.append((np.median(px), np.median(py), px.min(), px.max(), py.min(), py.max(),
                    pz.min(), pz.max(), m.sum()))
    return out if len(out) >= 2 else []


def extract(x, y, z, origin=(0.0, 0.0), cell=CELL_M, z_min=Z_MIN_M, z_max=Z_MAX_M,
            range_max=RANGE_MAX_M, min_points=MIN_POINTS, min_height=MIN_HEIGHT_M,
            max_span=MAX_SPAN_M, min_span=MIN_SPAN_M, peaks=False):
    """x, y, z: point coordinates (any planar frame, z = height above the floor).
    origin: the robot's XY in that frame, for the range limit.
    peaks: also split components of several people by head/shoulder peaks
    (_peak_split; offline-evaluated 2026-11-02, off by default until shadow-tested).
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
        multi = []
        if peaks and PEAK_TRY_SPAN_M <= span <= PEAK_MAX_SPAN_M:
            multi = _peak_split(xs, ys, zs, origin, max_span, min_span, min_points, min_height)
        if span > max_span:
            pieces = _split(xs, ys, zs, cell, max_span, min_span)
            dx, dy = xs - origin[0], ys - origin[1]
            if (dx * dx + dy * dy).min() < NEAR_SPLIT_R_M ** 2:
                for q in _split(xs, ys, zs, cell, max_span, min_span, NEAR_SPLIT_Z_M, origin):
                    if all(np.hypot(q[0] - o[0], q[1] - o[1]) > CORE_R_M for o in pieces):
                        pieces.append(q)
            # the isolated pieces of the usual split always stay; peak pieces add the
            # people it could not separate
            pieces += [q for q in multi if all(np.hypot(q[0] - o[0], q[1] - o[1]) > CORE_R_M for o in pieces)]
            out.extend(pieces)
            continue
        if multi:
            out.extend(multi)
            continue
        if span < min_span or z_hi - z_lo < min_height:
            continue
        out.append((np.median(xs), np.median(ys), x_lo, x_hi, y_lo, y_hi, z_lo, z_hi, e - s))
    if not out:
        return np.zeros((0, len(FIELDS)), dtype=np.float32)
    return np.asarray(out, dtype=np.float32)
