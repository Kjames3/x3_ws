"""C3 candidate tracker: constant-velocity Kalman filter in the odom frame.

Diagnostic only. Nothing here is wired to the CBF or the server. Arms B and C of
notes/triz_perception_mapping_experiment_plan.md (Experiment 1) share this
tracker and differ only in the measurements fed to it:

  B: v3's depth-blob centroids (VelocityEstimator._extract_depth_centroids)
  C: robust torso depth inside frozen-detector person boxes (person_measurement)

State is planar [x, y, vx, vy] in odom, advanced with the actual dt between
acquisition stamps. Association is global (Hungarian) on Mahalanobis distance.
Every output carries identity, covariance, observation age and whether this
step was measured or only predicted, so stale predictions stay visible.
"""
import math

import numpy as np
from scipy.optimize import linear_sum_assignment

# Frozen before scoring; replay writes this dict into every summary.json.
DEFAULT_CONFIG = {
    'accel_sigma_mps2': 0.5,        # white-acceleration process noise; tuned on trial05
    'init_speed_sigma_mps': 1.0,    # velocity is unknown at birth
    'gate_chi2': 9.21,              # chi2(2 dof) 0.99
    'confirm_hits': 3,
    'confirm_window': 5,            # hits must land within this many updates
    'tentative_max_misses': 2,
    'confirmed_max_age_s': 1.0,     # expiry without a measurement
    'max_dt_s': 0.5,                # a longer gap resets instead of predicting
    'max_tracks': 10,
    'depth_noise_coef': 0.0025,    # legacy baseline; configurable for Pro W evaluation
    'measurement_spread_scale': 1.0,  # spatial depth spread is not centroid uncertainty
    'meas_sigma_floor_m': 0.25,     # blob centroid jitter floor; tuned on trial05
}


# Depth accepted for a person measurement (m). 4.0 matches the v3 estimator's
# band; people at or past it are detected but give no measurement.
# c3_depth_cap_test.py varies the upper limit offline.
DEPTH_RANGE_M = (0.5, 4.0)
# mask_measurement subsamples a silhouette only while this many valid-depth samples remain.
SUBSAMPLE_MIN_VALID = 200


def measurement_cov_camera(z_m, floor_sigma_m=0.03, extra_sigma_m=0.0,
                           depth_noise_coef=0.0025, spread_scale=1.0):
    """Planar (forward, left) covariance for a point at depth z.

    ``depth_noise_coef * Z^2`` is a configurable range-dependent term.
    Defaults preserve the legacy estimator. ``spread_scale`` controls how much
    within-object depth spread counts as centroid uncertainty; pixel noise and
    object shape spread are not automatically centroid error. Tune on held-out
    measurements before selecting a Pro W coefficient.
    """
    if any(not math.isfinite(v) or v < 0 for v in
           (floor_sigma_m, extra_sigma_m, depth_noise_coef, spread_scale)):
        raise ValueError('measurement uncertainty parameters must be finite and nonnegative')
    extra = extra_sigma_m * spread_scale
    s_depth = math.hypot(max(floor_sigma_m, depth_noise_coef * z_m * z_m), extra)
    s_lat = math.hypot(floor_sigma_m, extra)
    return np.diag([s_depth ** 2, s_lat ** 2])


def camera_point_to_odom(point_cam, odom_from_camera):
    """Optical-frame (x right, y down, z forward) point -> odom xy.

    ``odom_from_camera`` is the pair-index record (translation + quaternion)
    looked up at the depth acquisition stamp.
    """
    x, y, z, w = odom_from_camera['quaternion_xyzw']
    rot = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    p = rot @ np.asarray(point_cam, float) + np.asarray(odom_from_camera['translation_xyz'])
    return p[:2], rot


def rotate_cov_to_odom(cov_fl, rot):
    """Rotate a (forward, left) camera-planar covariance into odom xy."""
    forward = rot[:2, 2]
    left = -rot[:2, 0]
    basis = np.column_stack([forward / (np.linalg.norm(forward) or 1),
                             left / (np.linalg.norm(left) or 1)])
    return basis @ cov_fl @ basis.T


# Histogram depth mode (opt-in, see _nearest_mode_depth). Bins are 5 cm; a peak
# is smoothed over three bins because a person spans 20-30 cm of depth.
HIST_BIN_M = 0.05
HIST_MIN_FRACTION = 0.15     # a peak must hold this share of the valid pixels
HIST_ROW_FRACTION = 0.6      # histogram reads only the top 60 % of the clipped box
DEPTH_MODES = ('window', 'nearest_mode')


def clip_box_to_image(box, shape):
    """Clip an (x1, y1, x2, y2) box to a (rows, cols) image; None if nothing is left.

    The detector box covers the whole body even when the head is above the
    frame, so y1 can be hundreds of pixels negative. Fractions of the unclipped
    box slide the torso window up past the image top and onto whatever lies
    behind the person.
    """
    rows, cols = shape[:2]
    x1, y1, x2, y2 = (float(v) for v in box)
    x1, x2 = max(0.0, x1), min(float(cols), x2)
    y1, y2 = max(0.0, y1), min(float(rows), y2)
    if x2 <= x1 or y2 <= y1:
        return None
    return x1, y1, x2, y2


def _nearest_mode_depth(region, min_valid_px, min_fraction=HIST_MIN_FRACTION, ground=None):
    """Depth of the nearest substantial peak of the in-range depth histogram.

    Returns (valid_count, selected_pixel_mask) or None. ``ground`` is an
    optional bool mask (same shape as ``region``) of floor pixels to ignore.
    The peak is a local maximum of the 3-bin smoothed counts holding at least
    ``min_fraction`` of the valid pixels, so a flat wall spiking one bin does not
    hide a person spread over several, and the rising flank of a peak is never
    taken for the peak. The result is the median of the pixels in those 3 bins.
    """
    ok = (region >= DEPTH_RANGE_M[0]) & (region <= DEPTH_RANGE_M[1])
    if ground is not None:
        ok &= ~ground
    n_valid = int(ok.sum())
    if n_valid < min_valid_px:
        return None
    lo = DEPTH_RANGE_M[0]
    n_bins = int(math.ceil((DEPTH_RANGE_M[1] - lo) / HIST_BIN_M)) + 1
    idx = np.clip(((region[ok] - lo) / HIST_BIN_M).astype(int), 0, n_bins - 1)
    counts = np.bincount(idx, minlength=n_bins).astype(float)
    smooth = np.convolve(counts, np.ones(3), mode='same')
    need = max(float(min_valid_px), min_fraction * n_valid)
    for b in range(n_bins):
        left = smooth[b - 1] if b > 0 else -1.0
        right = smooth[b + 1] if b + 1 < n_bins else -1.0
        if smooth[b] >= need and smooth[b] >= left and smooth[b] > right:
            sel = ok & (np.abs(((region - lo) / HIST_BIN_M).astype(int) - b) <= 1)
            return n_valid, sel
    return None


def person_measurement(depth_m, box, min_valid_px=30, min_valid_fraction=0.2, mask=None,
                       clip_box=True, depth_mode='window', ground_mask=None):
    """Robust torso depth inside a person box on the depth pixel grid.

    depth_mode 'window' (default): the central 40 % of the width and 20-55 % of
    the height, where the box is least likely to contain background or floor.
    'nearest_mode' (opt-in): the nearest substantial peak of the depth histogram
    over the central 60 % of the width and the top HIST_ROW_FRACTION of the box
    (the person is in front of the wall or partition behind them; the lower rows
    are where a desk or the floor in front shows up). u, v are then the centroid
    of the pixels in that peak, not the window centre. ``ground_mask`` (bool,
    depth shape) drops floor pixels from the histogram when the caller has one.

    clip_box (default True) clips the box to the image before any window is
    computed; False reproduces the old behaviour of windowing the unclipped box.
    A segmentation ``mask`` takes precedence over either box mode.
    Returns None when the support is too thin, otherwise pixel centre, depth and quality.
    """
    if mask is not None and mask.shape == depth_m.shape:
        m = mask_measurement(depth_m, mask, min_valid_px)
        if m is not None:
            return m
    if depth_mode not in DEPTH_MODES:
        raise ValueError(f'depth_mode must be one of {DEPTH_MODES}, got {depth_mode!r}')
    rows, cols = depth_m.shape
    if clip_box:
        box = clip_box_to_image(box, depth_m.shape)
        if box is None:
            return None
    x1, y1, x2, y2 = box
    w, h = x2 - x1, y2 - y1
    if depth_mode == 'nearest_mode':
        u0, u1 = int(x1 + 0.2 * w), int(math.ceil(x1 + 0.8 * w))
        v0, v1 = int(y1), int(math.ceil(y1 + HIST_ROW_FRACTION * h))
    else:
        u0, u1 = int(x1 + 0.3 * w), int(math.ceil(x1 + 0.7 * w))
        v0, v1 = int(y1 + 0.2 * h), int(math.ceil(y1 + 0.55 * h))
    u0, u1 = max(0, u0), min(cols, u1)
    v0, v1 = max(0, v0), min(rows, v1)
    if u1 <= u0 or v1 <= v0:
        return None
    region = depth_m[v0:v1, u0:u1]
    if depth_mode == 'nearest_mode':
        ground = ground_mask[v0:v1, u0:u1] if ground_mask is not None else None
        found = _nearest_mode_depth(region, min_valid_px, ground=ground)
        if found is None:
            return None
        n_valid, sel = found
        if sel.sum() < min_valid_px:
            return None
        vals = region[sel]
        vs, us = np.nonzero(sel)
        z = float(np.median(vals))
        spread = float(np.percentile(vals, 84) - np.percentile(vals, 16)) / 2
        return {'u': u0 + float(us.mean()) + 0.5, 'v': v0 + float(vs.mean()) + 0.5, 'z': z,
                'valid_fraction': n_valid / region.size, 'inlier_fraction': sel.sum() / n_valid,
                'spread_m': spread}
    valid = region[(region >= DEPTH_RANGE_M[0]) & (region <= DEPTH_RANGE_M[1])]
    fraction = valid.size / region.size
    if valid.size < min_valid_px or fraction < min_valid_fraction:
        return None
    median = float(np.median(valid))
    mad = float(np.median(np.abs(valid - median))) * 1.4826
    keep = valid[np.abs(valid - median) <= max(0.15, 2.5 * mad)]
    if keep.size < min_valid_px:
        return None
    z = float(np.median(keep))
    spread = float(np.percentile(keep, 84) - np.percentile(keep, 16)) / 2
    return {'u': (u0 + u1) / 2.0, 'v': (v0 + v1) / 2.0, 'z': z,
            'valid_fraction': fraction, 'inlier_fraction': keep.size / valid.size,
            'spread_m': spread}


def mask_measurement(depth_m, mask, min_valid_px=30):
    """Robust depth over a segmentation silhouette (no background by construction).

    Same outputs as person_measurement; u, v are the inlier-pixel centroid.
    Returns None when the silhouette has too little valid depth, so the caller
    can fall back to the box.
    """
    # A big silhouette is ~150k pixels and every step below is O(pixels); on
    # the Jetson that was most of an 80 ms tracker update. Seg masks are a
    # 160x120 grid upscaled 4x, so every 4th pixel keeps the shape. Go finer
    # only when that leaves too few pixels with valid depth (a far person
    # mostly past the depth limit): step 1 is the original, exact computation.
    for step in (4, 2, 1):
        vs, us = np.nonzero(mask[::step, ::step])
        vals = depth_m[::step, ::step][vs, us]
        ok = (vals >= DEPTH_RANGE_M[0]) & (vals <= DEPTH_RANGE_M[1])
        if step == 1 or ok.sum() >= SUBSAMPLE_MIN_VALID:
            break
    if vs.size == 0 or ok.sum() < min_valid_px:
        return None
    # Sample (i, j) stands for the step x step block starting at (i*step, j*step).
    vs = vs * step + (step - 1) / 2.0
    us = us * step + (step - 1) / 2.0
    vs, us, vals = vs[ok], us[ok], vals[ok]
    median = float(np.median(vals))
    mad = float(np.median(np.abs(vals - median))) * 1.4826
    keep = np.abs(vals - median) <= max(0.15, 2.5 * mad)
    if keep.sum() < min_valid_px:
        return None
    k = vals[keep]
    return {'u': float(us[keep].mean()), 'v': float(vs[keep].mean()), 'z': float(np.median(k)),
            'valid_fraction': float(ok.mean()), 'inlier_fraction': float(keep.mean()),
            'spread_m': float(np.percentile(k, 84) - np.percentile(k, 16)) / 2,
            'mask': True}


class KalmanTracker:
    def __init__(self, config=None):
        self.config = dict(DEFAULT_CONFIG, **(config or {}))
        self.tracks = []
        self.next_id = 0
        self.last_t = None

    def reset(self):
        self.tracks = []
        self.last_t = None

    def _predict(self, dt):
        q = self.config['accel_sigma_mps2'] ** 2
        f = np.eye(4)
        f[0, 2] = f[1, 3] = dt
        g = np.array([[dt * dt / 2, 0], [0, dt * dt / 2], [dt, 0], [0, dt]])
        for t in self.tracks:
            t['x'] = f @ t['x']
            t['P'] = f @ t['P'] @ f.T + q * g @ g.T

    def update(self, t_s, measurements):
        """Advance to ``t_s`` and fuse ``[(xy, R), ...]`` in odom.

        Returns one output dict per live track (tentative included, flagged).
        """
        cfg = self.config
        if self.last_t is not None:
            dt = t_s - self.last_t
            if dt <= 0 or dt > cfg['max_dt_s']:
                # Time reversal or a gap: predicting across it would invent motion.
                self.reset()
            else:
                self._predict(dt)
        self.last_t = t_s
        h = np.hstack([np.eye(2), np.zeros((2, 2))])
        n_t, n_m = len(self.tracks), len(measurements)
        pairs = []
        if n_t and n_m:
            cost = np.full((n_t, n_m), 1e6)
            for i, t in enumerate(self.tracks):
                for j, (z, r) in enumerate(measurements):
                    s = h @ t['P'] @ h.T + r
                    d = np.asarray(z) - h @ t['x']
                    m2 = float(d @ np.linalg.solve(s, d))
                    if m2 <= cfg['gate_chi2']:
                        cost[i, j] = m2
            rows, cols = linear_sum_assignment(cost)
            pairs = [(i, j) for i, j in zip(rows, cols) if cost[i, j] < 1e6]
        matched_t = {i for i, _ in pairs}
        matched_m = {j for _, j in pairs}
        for i, j in pairs:
            t = self.tracks[i]
            z, r = measurements[j]
            s = h @ t['P'] @ h.T + r
            k = t['P'] @ h.T @ np.linalg.inv(s)
            t['x'] = t['x'] + k @ (np.asarray(z) - h @ t['x'])
            t['P'] = (np.eye(4) - k @ h) @ t['P']
            t['measurement_index'] = int(j)
            t['last_meas_t'] = t_s
            t['hits'] += 1
            t['misses'] = 0
            t['recent'].append(True)
        for i, t in enumerate(self.tracks):
            if i not in matched_t:
                t['measurement_index'] = None
                t['misses'] += 1
                t['recent'].append(False)
            t['recent'] = t['recent'][-cfg['confirm_window']:]
            if not t['confirmed'] and sum(t['recent']) >= cfg['confirm_hits']:
                t['confirmed'] = True
        alive = []
        for t in self.tracks:
            if t['confirmed']:
                if t_s - t['last_meas_t'] <= cfg['confirmed_max_age_s']:
                    alive.append(t)
            elif t['misses'] <= cfg['tentative_max_misses']:
                alive.append(t)
        self.tracks = alive
        sv = cfg['init_speed_sigma_mps'] ** 2
        for j, (z, r) in enumerate(measurements):
            if j in matched_m or len(self.tracks) >= cfg['max_tracks']:
                continue
            p = np.zeros((4, 4))
            p[:2, :2] = r
            p[2, 2] = p[3, 3] = sv
            self.tracks.append({'id': self.next_id, 'x': np.array([z[0], z[1], 0.0, 0.0]),
                                'P': p, 'last_meas_t': t_s, 'hits': 1, 'misses': 0,
                                'recent': [True], 'confirmed': False, 'measurement_index': j})
            self.next_id += 1
        out = []
        for t in self.tracks:
            x = t['x']
            out.append({'id': t['id'], 'x': float(x[0]), 'y': float(x[1]),
                        'vx': float(x[2]), 'vy': float(x[3]),
                        'speed': float(math.hypot(x[2], x[3])),
                        'cov': t['P'].tolist(),
                        'obs_age_s': float(t_s - t['last_meas_t']),
                        'measured': t['last_meas_t'] == t_s,
                        'measurement_index': t['measurement_index'],
                        'confirmed': t['confirmed']})
        return out
