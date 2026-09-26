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
    'meas_sigma_floor_m': 0.25,     # blob centroid jitter floor; tuned on trial05
}


def measurement_cov_camera(z_m, floor_sigma_m=0.03, extra_sigma_m=0.0):
    """Planar (forward, left) covariance for a point at depth z.

    Depth sigma follows the characterized OAK stereo model sigma ~ 0.0025 Z^2
    with ``floor_sigma_m`` as the floor for segmentation/centroid jitter.
    """
    s_depth = math.hypot(max(floor_sigma_m, 0.0025 * z_m * z_m), extra_sigma_m)
    s_lat = math.hypot(floor_sigma_m, extra_sigma_m)
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


def person_measurement(depth_m, box, min_valid_px=30, min_valid_fraction=0.2):
    """Robust torso depth inside a person box on the depth pixel grid.

    Uses the central 40 % of the width and 20-55 % of the height, where the
    box is least likely to contain background or floor. Returns None when the
    support is too thin, otherwise pixel centre, median depth and quality.
    """
    x1, y1, x2, y2 = box
    w, h = x2 - x1, y2 - y1
    u0, u1 = int(x1 + 0.3 * w), int(math.ceil(x1 + 0.7 * w))
    v0, v1 = int(y1 + 0.2 * h), int(math.ceil(y1 + 0.55 * h))
    rows, cols = depth_m.shape
    u0, u1 = max(0, u0), min(cols, u1)
    v0, v1 = max(0, v0), min(rows, v1)
    if u1 <= u0 or v1 <= v0:
        return None
    region = depth_m[v0:v1, u0:u1]
    valid = region[(region >= 0.5) & (region <= 4.0)]
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
            t['last_meas_t'] = t_s
            t['hits'] += 1
            t['misses'] = 0
            t['recent'].append(True)
        for i, t in enumerate(self.tracks):
            if i not in matched_t:
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
                                'recent': [True], 'confirmed': False})
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
                        'confirmed': t['confirmed']})
        return out
