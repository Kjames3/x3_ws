"""Live, diagnostic-only C3 person tracker (arm C) on the OAK's on-device YOLO.

Each new OAK detection packet is turned into robust torso-depth measurements
(``c3_person_tracker.person_measurement``), placed in odom with the current
robot pose and fused by the constant-velocity Kalman tracker with the arm C
config frozen in the 2026-09-25 evaluation. Nothing here reaches the CBF.

Timing: NN packets reach the host a few hundred ms after their frame was
captured. The detection is placed with the robot pose interpolated to the
frame's capture time (a ~2 s pose history sampled here) and measured on the
depth frame taken with it. Using "latest pose / latest depth" instead made a
still person swing ~2 m sideways during a 2 rad/s turn (2026-09-28).

Differences from the offline replay, on purpose kept visible:
  - camera -> base uses the OAK mount offsets only (no -0.29 deg pitch, no TF);
  - boxes come from the OAK's yolo26 blob, not the host yolo26n replay boxes.

Outputs are robot-local (fwd, left) so the GUI can draw them without a pose.
"""
import bisect
import json
import logging
import math
import threading
import time
from collections import deque

import numpy as np

import c3_person_tracker as pt

logger = logging.getLogger(__name__)

# Arm C, frozen after r1 (artifacts/c3-eval-2026-09-25/RESULTS.md).
ARM_C_CONFIG = dict(pt.DEFAULT_CONFIG, accel_sigma_mps2=1.5, meas_sigma_floor_m=0.05)
PREDICT_S = 0.5       # CV prediction is only better than "hold" out to ~0.5-0.7 s
POLL_S = 0.02
POSE_MAX_AGE_S = 0.5
POSE_HISTORY_S = 2.0


class C3Live:
    def __init__(self, oak, pose_fn, mount_x, mount_z=0.0, log_path=None):
        """pose_fn() -> {'x','y','theta'} in odom, or None when stale.

        log_path: optional NDJSON debug log of raw measurements and the pose
        stream (25 Hz), so placement can be recomputed offline with any
        timing offset.
        """
        self._log = open(log_path, 'a', buffering=1) if log_path else None
        self._log_pose_n = 0
        self.oak = oak
        self.pose_fn = pose_fn
        self.mount_x = mount_x
        self.tracker = pt.KalmanTracker(ARM_C_CONFIG)
        self._lock = threading.Lock()
        self._tracks = []
        self._stats = {'updates': 0, 'update_ms_avg': 0.0, 'update_ms_max': 0.0, 'hz': 0.0}
        # Where person detections are lost between the OAK and a ring (cumulative).
        #   person_packets / ring_packets: packets with a person box / with >=1 ring
        #   dets / meas: person boxes / boxes that gave a torso-depth measurement
        #   reset_pose / reset_depth: packets dropped (tracker reset) for stale
        #   odom or missing depth; reset_gap: packet gap > max_dt_s reset the tracker
        #   timed: packets placed with capture-time pose + depth (else latest)
        self._diag = dict(person_packets=0, ring_packets=0, dets=0, meas=0,
                          reset_pose=0, reset_depth=0, reset_gap=0, timed=0,
                          latency_ms_avg=0.0)
        self._running = False
        self._thread = None
        self._poses = deque()   # (monotonic, x, y, unwrapped theta), sampled every poll
        self._meas_log = []

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._run, name='c3-live', daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._log is not None:
            self._log.close()
            self._log = None

    def _write(self, rec):
        if self._log is not None:
            try:
                self._log.write(json.dumps(rec) + '\n')
            except (OSError, ValueError):
                self._log = None

    def get_tracks(self):
        with self._lock:
            st = {k: v for k, v in self._stats.items() if not k.startswith('_')}
            st['diag'] = dict(self._diag)
            return list(self._tracks), st

    def _measurements(self, detections, depth, intr, pose, masks=None):
        fx, fy, cx, cy, w, h = intr
        sx, sy = depth.shape[1] / w, depth.shape[0] / h
        fx, fy, cx, cy = fx * sx, fy * sy, cx * sx, cy * sy
        c, s = math.cos(pose['theta']), math.sin(pose['theta'])
        rot = np.array([[c, -s], [s, c]])
        floor = ARM_C_CONFIG['meas_sigma_floor_m']
        out = []
        for i, d in enumerate(detections):
            if d.get('label') != 'person' or not d.get('bbox'):
                continue
            mask = masks[i] if masks is not None and i < len(masks) else None
            m = pt.person_measurement(depth, d['bbox'], mask=mask)
            if m is None:
                continue
            fwd = self.mount_x + m['z']
            left = -(m['u'] - cx) * m['z'] / fx
            self._meas_log.append({'bbox': d['bbox'], 'u': m['u'], 'z': round(m['z'], 4),
                                   'spread': round(m['spread_m'], 4), 'fwd': round(fwd, 4),
                                   'mask': bool(m.get('mask')),
                                   'left': round(left, 4)})
            xy = rot @ np.array([fwd, left]) + np.array([pose['x'], pose['y']])
            cov = rot @ pt.measurement_cov_camera(m['z'], floor, m['spread_m']) @ rot.T
            out.append((xy, cov))
        return out

    def _record_pose(self, now):
        pose = self.pose_fn()
        if pose is None:
            self._poses.clear()
            return
        th = pose['theta']
        if self._poses:   # unwrap so interpolation never crosses the +/-pi seam
            prev = self._poses[-1][3]
            th = prev + math.atan2(math.sin(th - prev), math.cos(th - prev))
        self._poses.append((now, pose['x'], pose['y'], th))
        self._log_pose_n += 1
        if self._log is not None and self._log_pose_n % 2 == 0:
            self._write({'k': 'pose', 't': round(now, 4), 'x': round(pose['x'], 4),
                         'y': round(pose['y'], 4), 'th': round(th, 5)})
        while self._poses and now - self._poses[0][0] > POSE_HISTORY_S:
            self._poses.popleft()

    def _pose_at(self, t):
        """Pose interpolated at monotonic t, or None if t is outside the history."""
        ps = self._poses
        if len(ps) < 2 or not (ps[0][0] <= t <= ps[-1][0] + POLL_S * 2):
            return None
        ts = [p[0] for p in ps]
        i = bisect.bisect_left(ts, t)
        if i >= len(ps):
            _, x, y, th = ps[-1]
            return {'x': x, 'y': y, 'theta': th}
        if i == 0 or ts[i] == t:
            _, x, y, th = ps[i]
            return {'x': x, 'y': y, 'theta': th}
        (t0, x0, y0, h0), (t1, x1, y1, h1) = ps[i - 1], ps[i]
        a = (t - t0) / (t1 - t0)
        return {'x': x0 + a * (x1 - x0), 'y': y0 + a * (y1 - y0), 'theta': h0 + a * (h1 - h0)}

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
                self._record_pose(time.monotonic())
                # The packet stamp is the only reliable "new packet" signal;
                # a repeat would reach the tracker with dt=0 and reset it.
                stamp = getattr(self.oak, '_latest_detections_t', 0.0)
                if stamp == 0.0 or stamp == last_t:
                    continue
                last_t = stamp
                if hasattr(self.oak, 'get_detections_with_masks'):
                    detections, masks = self.oak.get_detections_with_masks()
                else:
                    (detections, _), masks = self.oak.get_detection_observation(), None
                detections = detections or []
                people = [d for d in detections if d.get('label') == 'person' and d.get('bbox')]
                dg = self._diag
                cap_t = getattr(self.oak, 'get_detection_capture_time', lambda: None)()
                near = (self.oak.get_depth_near(cap_t)
                        if cap_t is not None and hasattr(self.oak, 'get_depth_near') else None)
                pose_cap = self._pose_at(cap_t) if cap_t is not None else None
                if cap_t is not None:
                    # Receipt minus capture: how late the packet is (EMA, ms).
                    lat = self.oak._latest_detections_t - cap_t
                    dg['latency_ms_avg'] = round(0.9 * dg['latency_ms_avg'] + 100.0 * lat, 1)
                    stamp = cap_t   # tracker time = capture time: honest dt between frames
                if near is not None and pose_cap is not None:
                    depth, pose = near[1], pose_cap
                    dg['timed'] += 1
                else:
                    pose = self.pose_fn()
                    depth = self.oak.get_raw_depth_frame()
                intr = self.oak.get_depth_intrinsics()
                if people:
                    dg['person_packets'] += 1
                    dg['dets'] += len(people)
                if pose is None or depth is None or intr is None:
                    dg['reset_pose' if pose is None else 'reset_depth'] += 1
                    self.tracker.reset()
                    with self._lock:
                        self._tracks = []
                    continue
                t0 = time.perf_counter()
                self._meas_log = []
                meas = self._measurements(detections, depth, intr, pose, masks)
                self._write({'k': 'pkt', 'recv': round(self.oak._latest_detections_t, 4),
                             'cap': None if cap_t is None else round(cap_t, 4),
                             'timed': near is not None and pose_cap is not None,
                             'depth_t': None if near is None else round(near[0], 4),
                             'pose': {k: round(v, 5) for k, v in pose.items()},
                             'intr': [round(v, 3) for v in intr[:4]] + list(intr[4:]),
                             'depth_shape': list(depth.shape), 'meas': self._meas_log})
                dg['meas'] += len(meas)
                prev_t = self.tracker.last_t
                if prev_t is not None and stamp - prev_t > ARM_C_CONFIG['max_dt_s']:
                    dg['reset_gap'] += 1   # update() will reset rather than predict
                # Tracking runs at capture time, but the GUI draws rings around
                # the robot as it is NOW: convert with the newest pose.
                now_pose = self.pose_fn() or pose
                if now_pose is not pose:
                    now_pose = dict(now_pose)
                    now_pose['theta'] = pose['theta'] + math.atan2(
                        math.sin(now_pose['theta'] - pose['theta']),
                        math.cos(now_pose['theta'] - pose['theta']))
                tracks = self._to_local(self.tracker.update(stamp, meas), now_pose)
                if people and tracks:
                    dg['ring_packets'] += 1
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
