"""
velocity_estimator.py — Real-time pedestrian velocity estimation for Yahboom X3.

Integrates with existing drivers_x3.py:
  - AstraCamera.get_depth_frame() for depth centroid extraction
  - YDLidarDriver.get_points_xy() for LiDAR cluster features
  - Outputs per-obstacle (vx, vy) estimates at 10Hz

Drop this file into ~/x3_ws/src/ alongside drivers_x3.py.
"""

import torch
import numpy as np
import cv2
import joblib
import threading
import time
import logging
import os
from pathlib import Path
from collections import deque

logger = logging.getLogger(__name__)

# ── Config ────────────────────────────────────────────────────────────────────
# Paths resolve relative to this file's location (~/x3_ws/src/) so they work
# regardless of username on the Jetson.
_SRC_DIR      = Path(__file__).parent.resolve()
MODEL_PATH    = str(_SRC_DIR / "velocity_mlp.torchscript")
SCALER_X_PATH = str(_SRC_DIR / "scaler_X.pkl")
SCALER_Y_PATH = str(_SRC_DIR / "scaler_y.pkl")

WINDOW_SIZE   = 10          # frames of history (matches training T=10)
INFER_HZ      = 10          # target inference rate
MIN_BLOB_AREA = 500         # pixels — ignore tiny depth blobs
MAX_OBSTACLES = 5           # track at most N obstacles simultaneously
MAX_RANGE_M   = 5.0         # ignore detections beyond this distance


class ObstacleTracker:
    """
    Lightweight centroid tracker — matches detections to existing tracks
    by nearest-centroid distance. No Kalman, no external deps.
    """
    def __init__(self, max_distance=0.8, max_age=10):
        self.tracks   = {}   # track_id -> {'centroid': (x,y), 'age': int, 'history': deque}
        self.next_id  = 0
        self.max_dist = max_distance   # metres
        self.max_age  = max_age        # frames before track is dropped

    def update(self, centroids_m):
        """
        centroids_m: list of (x, y, z) in metres relative to robot.
        Returns dict: track_id -> {'centroid': (x,y,z), 'history': deque}
        """
        # Age all tracks
        for tid in list(self.tracks):
            self.tracks[tid]['age'] += 1
            if self.tracks[tid]['age'] > self.max_age:
                del self.tracks[tid]

        # Match detections to tracks
        unmatched = list(range(len(centroids_m)))
        for tid, track in self.tracks.items():
            if not unmatched:
                break
            tx, ty = track['centroid'][:2]
            dists  = [np.hypot(centroids_m[i][0]-tx, centroids_m[i][1]-ty)
                      for i in unmatched]
            best   = int(np.argmin(dists))
            if dists[best] < self.max_dist:
                idx = unmatched.pop(best)
                cx, cy, cz = centroids_m[idx]
                track['centroid'] = (cx, cy, cz)
                track['age']      = 0
                track['history'].append((cx, cy, cz))

        # Create new tracks for unmatched detections
        for idx in unmatched:
            if len(self.tracks) >= MAX_OBSTACLES:
                break
            tid = self.next_id
            self.next_id += 1
            cx, cy, cz = centroids_m[idx]
            self.tracks[tid] = {
                'centroid': (cx, cy, cz),
                'age':      0,
                'history':  deque(maxlen=WINDOW_SIZE),
            }
            self.tracks[tid]['history'].append((cx, cy, cz))

        return {tid: {'centroid': t['centroid'], 'history': t['history']}
                for tid, t in self.tracks.items()}


class VelocityEstimator:
    """
    Loads the TorchScript MLP and runs inference at INFER_HZ.
    Designed to run alongside the existing server_x3.py broadcast loop.

    Usage:
        estimator = VelocityEstimator(camera, lidar)
        estimator.start()
        ...
        estimates = estimator.get_estimates()
        # estimates: list of {'id': int, 'x': float, 'y': float,
        #                      'vx': float, 'vy': float, 'speed': float}
        estimator.stop()
    """

    def __init__(self, camera, lidar, robot_pose_fn=None, model_path=None):
        """
        camera:        AstraCamera instance (or ROS2Bridge)
        lidar:         YDLidarDriver instance (or ROS2Bridge)
        robot_pose_fn: callable returning {'x': m, 'y': m, 'theta': rad}
                       for ego-motion compensation. Pass None to skip.
        model_path:    optional string path to the TorchScript model file
        """
        self.camera        = camera
        self.lidar         = lidar
        self.robot_pose_fn = robot_pose_fn
        self.model_path    = model_path or MODEL_PATH

        self._model     = None
        self._scaler_X  = None
        self._scaler_y  = None
        self._tracker   = ObstacleTracker()
        self._estimates = []
        self._lock      = threading.Lock()
        self._running   = False
        self._thread    = None

        self._load_model()

    def _load_model(self):
        try:
            self._model    = torch.jit.load(self.model_path, map_location='cpu')
            self._model.eval()
            self._scaler_X = joblib.load(SCALER_X_PATH)
            self._scaler_y = joblib.load(SCALER_Y_PATH)
            logger.info(f"VelocityEstimator: model {self.model_path} and scalers loaded")
        except Exception as e:
            logger.error(f"VelocityEstimator: failed to load model {self.model_path}: {e}")

    def _extract_depth_centroids(self, depth_frame, raw_depth_frame=None):
        """
        Extract obstacle centroids. If raw_depth_frame is provided, we perform
        thresholding directly on the raw physical depth values in meters to avoid 
        colorization and min-max scaling artifacts. Otherwise, we fall back to
        the colorised BGR thresholding logic.

        Returns list of (x_m, y_m, Z) relative to camera centre.
        """
        if raw_depth_frame is not None:
            # Create a binary mask where depth is between 0.5m and 4.0m
            mask = ((raw_depth_frame >= 0.5) & (raw_depth_frame <= 4.0) & (~np.isnan(raw_depth_frame))).astype(np.uint8) * 255
            
            # Morphological cleanup
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            centroids = []
            h, w = raw_depth_frame.shape[:2]
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < MIN_BLOB_AREA:
                    continue
                M = cv2.moments(cnt)
                if M['m00'] == 0:
                    continue
                cx = M['m10'] / M['m00']
                cy = M['m01'] / M['m00']
                
                # Mask out this contour's region to get its raw depth values
                cnt_mask = np.zeros_like(mask)
                cv2.drawContours(cnt_mask, [cnt], -1, 255, -1)
                depth_vals = raw_depth_frame[cnt_mask == 255]
                valid_depths = depth_vals[(depth_vals >= 0.5) & (depth_vals <= 4.0) & (~np.isnan(depth_vals))]
                
                if len(valid_depths) > 0:
                    Z = float(np.median(valid_depths))
                else:
                    Z = 1.0  # fallback if no valid depth
                
                # Convert pixel centroid to physical metres using camera model
                # Astra Pro SC: 640x480, fx ≈ 554
                fx = 554.0
                x_m = (cx - w / 2.0) * Z / fx
                y_m = (cy - h / 2.0) * Z / fx
                centroids.append((x_m, y_m, Z))
                
            return centroids[:MAX_OBSTACLES]

        if depth_frame is None:
            return []

        # Convert to grayscale — bright pixels are near objects
        gray = cv2.cvtColor(depth_frame, cv2.COLOR_BGR2GRAY)

        # Threshold: keep pixels brighter than 120 (near objects)
        _, thresh = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY)

        # Morphological cleanup
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN,  kernel)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        centroids = []
        h, w = depth_frame.shape[:2]

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_BLOB_AREA:
                continue
            M  = cv2.moments(cnt)
            if M['m00'] == 0:
                continue
            cx = M['m10'] / M['m00']
            cy = M['m01'] / M['m00']

            # Calculate actual Z (depth) in meters using the raw depth frame
            Z = 1.0
            fx = 554.0
            x_m = (cx - w / 2.0) * Z / fx
            y_m = (cy - h / 2.0) * Z / fx

            centroids.append((x_m, y_m, Z))

        return centroids[:MAX_OBSTACLES]

    def _build_window_features(self, history):
        """
        Build a (1, 40) feature vector from a deque of T (x,y,z) positions.
        Matches the sliding window format used during training:
          [rel_x, rel_y, dx, dy] × T frames, flattened.
        """
        hist = list(history)
        # Pad with first entry if history shorter than window
        while len(hist) < WINDOW_SIZE:
            hist.insert(0, hist[0] if hist else (0.0, 0.0, 1.0))

        hist = hist[-WINDOW_SIZE:]
        features = []
        for i, (x, y, z) in enumerate(hist):
            if i == 0:
                dx, dy = 0.0, 0.0
            else:
                dx = x - hist[i-1][0]
                dy = y - hist[i-1][1]
            features.extend([x, y, dx, dy])

        return np.array(features, dtype=np.float32).reshape(1, -1)

    def _inference_loop(self):
        dt = 1.0 / INFER_HZ
        logger.info("VelocityEstimator: inference loop started")

        while self._running:
            t0 = time.monotonic()

            try:
                # 1. Get depth and raw depth frames
                depth_frame = self.camera.get_depth_frame() if hasattr(self.camera, 'get_depth_frame') else None
                raw_depth_frame = self.camera.get_raw_depth_frame() if hasattr(self.camera, 'get_raw_depth_frame') else None

                # 2. Extract centroids
                centroids = self._extract_depth_centroids(depth_frame, raw_depth_frame)

                # 3. Update tracker
                tracks = self._tracker.update(centroids)

                # 4. Run MLP inference on each track with full history
                estimates = []
                for tid, track in tracks.items():
                    if len(track['history']) < 2:
                        continue  # need at least 2 frames for displacement

                    features = self._build_window_features(track['history'])

                    # Normalize using training scalers
                    features_scaled = self._scaler_X.transform(features)
                    x_tensor = torch.tensor(features_scaled, dtype=torch.float32)

                    with torch.no_grad():
                        pred_scaled = self._model(x_tensor).numpy()

                    pred_ms = self._scaler_y.inverse_transform(pred_scaled)
                    vx, vy  = float(pred_ms[0, 0]), float(pred_ms[0, 1])
                    speed   = float(np.sqrt(vx**2 + vy**2))

                    cx, cy, cz = track['centroid']
                    estimates.append({
                        'id':    tid,
                        'x':     round(cx, 3),
                        'y':     round(cy, 3),
                        'z':     round(cz, 3),
                        'vx':    round(vx, 3),
                        'vy':    round(vy, 3),
                        'speed': round(speed, 3),
                    })

                with self._lock:
                    self._estimates = estimates

            except Exception as e:
                logger.error(f"VelocityEstimator: inference error: {e}")

            # Sleep remainder of cycle
            elapsed = time.monotonic() - t0
            sleep_t = max(0.0, dt - elapsed)
            time.sleep(sleep_t)

    def start(self):
        if self._model is None:
            logger.error("VelocityEstimator: cannot start — model not loaded")
            return
        self._running = True
        self._thread  = threading.Thread(target=self._inference_loop, daemon=True)
        self._thread.start()
        logger.info("VelocityEstimator: started")

    def stop(self):
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        logger.info("VelocityEstimator: stopped")

    def get_estimates(self):
        """Return latest list of obstacle velocity estimates. Thread-safe."""
        with self._lock:
            return list(self._estimates)