"""Live, diagnostic-only C3 person tracker (arm C) on the OAK's on-device YOLO.

Each new OAK detection packet is turned into robust torso-depth measurements
(``c3_person_tracker.person_measurement``), placed in odom with the current
robot pose and fused by the constant-velocity Kalman tracker with the arm C
config frozen in the 2026-09-25 evaluation. Nothing here reaches the CBF.

Differences from the offline replay, on purpose kept visible:
  - camera -> base uses the OAK mount offsets only (no -0.29 deg pitch, no TF);
  - the pose is the latest odom when the packet is processed, not interpolated
    to the depth stamp;
  - boxes come from the OAK's yolo26 blob, not the host yolo26n replay boxes.

Outputs are robot-local (fwd, left) so the GUI can draw them without a pose.
"""
import logging
import math
import threading
import time

import numpy as np

import c3_person_tracker as pt

logger = logging.getLogger(__name__)

# Arm C, frozen after r1 (artifacts/c3-eval-2026-09-25/RESULTS.md).
ARM_C_CONFIG = dict(pt.DEFAULT_CONFIG, accel_sigma_mps2=1.5, meas_sigma_floor_m=0.05)
PREDICT_S = 0.5       # CV prediction is only better than "hold" out to ~0.5-0.7 s
POLL_S = 0.02
POSE_MAX_AGE_S = 0.5


class C3Live:
    def __init__(self, oak, pose_fn, mount_x, mount_z=0.0):
        """pose_fn() -> {'x','y','theta'} in odom, or None when stale."""
        self.oak = oak
        self.pose_fn = pose_fn
        self.mount_x = mount_x
        self.tracker = pt.KalmanTracker(ARM_C_CONFIG)
        self._lock = threading.Lock()
        self._tracks = []
        self._stats = {'updates': 0, 'update_ms_avg': 0.0, 'update_ms_max': 0.0, 'hz': 0.0}
        self._running = False
        self._thread = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._run, name='c3-live', daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def get_tracks(self):
        with self._lock:
            return list(self._tracks), {k: v for k, v in self._stats.items() if not k.startswith('_')}

    def _measurements(self, detections, depth, intr, pose):
        fx, fy, cx, cy, w, h = intr
        sx, sy = depth.shape[1] / w, depth.shape[0] / h
        fx, fy, cx, cy = fx * sx, fy * sy, cx * sx, cy * sy
        c, s = math.cos(pose['theta']), math.sin(pose['theta'])
        rot = np.array([[c, -s], [s, c]])
        floor = ARM_C_CONFIG['meas_sigma_floor_m']
        out = []
        for d in detections:
            if d.get('label') != 'person' or not d.get('bbox'):
                continue
            m = pt.person_measurement(depth, d['bbox'])
            if m is None:
                continue
            fwd = self.mount_x + m['z']
            left = -(m['u'] - cx) * m['z'] / fx
            xy = rot @ np.array([fwd, left]) + np.array([pose['x'], pose['y']])
            cov = rot @ pt.measurement_cov_camera(m['z'], floor, m['spread_m']) @ rot.T
            out.append((xy, cov))
        return out

    def _to_local(self, tracks, pose):
        c, s = math.cos(pose['theta']), math.sin(pose['theta'])
        out = []
        for t in tracks:
            if not t['confirmed']:
                continue
            dx, dy = t['x'] - pose['x'], t['y'] - pose['y']
            vx, vy = t['vx'], t['vy']
            cov = np.asarray(t['cov'])
            # Position covariance after PREDICT_S of constant velocity.
            f = np.hstack([np.eye(2), PREDICT_S * np.eye(2)])
            p_pred = f @ cov @ f.T
            out.append({
                'id': t['id'],
                'fwd': round(c * dx + s * dy, 3), 'left': round(-s * dx + c * dy, 3),
                'vx': round(c * vx + s * vy, 3), 'vy': round(-s * vx + c * vy, 3),
                'speed': round(t['speed'], 3),
                'pos_sigma_m': round(math.sqrt(max(np.linalg.eigvalsh(cov[:2, :2]).max(), 0)), 3),
                'pred_sigma_m': round(math.sqrt(max(np.linalg.eigvalsh(p_pred).max(), 0)), 3),
                'predict_s': PREDICT_S,
                'obs_age_s': round(t['obs_age_s'], 3),
                'measured': t['measured'],
            })
        return out

    def _run(self):
        last_t = None
        while self._running:
            time.sleep(POLL_S)
            try:
                # The packet stamp is the only reliable "new packet" signal;
                # a repeat would reach the tracker with dt=0 and reset it.
                stamp = getattr(self.oak, '_latest_detections_t', 0.0)
                if stamp == 0.0 or stamp == last_t:
                    continue
                last_t = stamp
                detections, _ = self.oak.get_detection_observation()
                pose = self.pose_fn()
                depth = self.oak.get_raw_depth_frame()
                intr = self.oak.get_depth_intrinsics()
                if pose is None or depth is None or intr is None:
                    self.tracker.reset()
                    with self._lock:
                        self._tracks = []
                    continue
                t0 = time.perf_counter()
                meas = self._measurements(detections or [], depth, intr, pose)
                tracks = self._to_local(self.tracker.update(stamp, meas), pose)
                ms = (time.perf_counter() - t0) * 1e3
                with self._lock:
                    self._tracks = tracks
                    st = self._stats
                    st['updates'] += 1
                    st['update_ms_avg'] = round(0.95 * st['update_ms_avg'] + 0.05 * ms, 3)
                    st['update_ms_max'] = round(max(st['update_ms_max'] * 0.999, ms), 3)
                    if st.get('_prev') is not None:
                        st['hz'] = round(0.9 * st['hz'] + 0.1 / max(stamp - st['_prev'], 1e-3), 2)
                    st['_prev'] = stamp
            except Exception as e:  # diagnostic: never take the server down
                logger.warning('C3 live update failed: %s', e)
                time.sleep(0.5)
