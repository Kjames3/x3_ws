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
import json
import threading
import time
import logging
import os
import math
from pathlib import Path
from collections import deque

logger = logging.getLogger(__name__)

# ── Config ────────────────────────────────────────────────────────────────────
# Paths resolve relative to this file's location (~/x3_ws/src/) so they work
# regardless of username on the Jetson.
_SRC_DIR           = Path(__file__).parent.resolve()
MODEL_PATH         = str(_SRC_DIR / "velocity_mlp.torchscript")
SCALER_PARAMS_PATH = str(_SRC_DIR / "scaler_params.json")
GROUND_PLANE_CONFIG_PATH = str(_SRC_DIR.parent / "config" /
                               "camera_ground_plane.json")

WINDOW_SIZE   = 10          # frames of history (matches training T=10)
INFER_HZ      = 10          # target inference rate
# A depth frame older than this is a stalled camera, not a slow one. The
# estimator runs at INFER_HZ regardless of capture, so without this it happily
# re-projects one frozen frame forever -- and because centroids are mapped into
# a GLOBAL frame using live odometry, a moving robot turns a frozen frame into
# fabricated obstacle motion. 0.5 s is 5 inference periods and ~15 frames at the
# measured ~29 fps depth rate, so it only trips on a real stall.
MAX_DEPTH_FRAME_AGE_S = 0.5
MIN_BLOB_AREA = 500         # pixels — ignore tiny depth blobs
MAX_OBSTACLES = 5           # track at most N obstacles simultaneously
MAX_RANGE_M   = 5.0         # ignore detections beyond this distance

# Safe defaults match the physically measured OAK optical-centre height and the
# level mount represented in the X3 Plus URDF. The JSON file makes a later
# empty-floor pitch calibration deployable without changing code.
DEFAULT_CAMERA_HEIGHT_M = 0.210
DEFAULT_CAMERA_PITCH_DEG = 0.0
DEFAULT_MIN_HEIGHT_M = 0.15
DEFAULT_MAX_HEIGHT_M = 2.0
# Tracks beyond this range report speed 0.0 without running the model. The MLP
# was trained against this same 1.8 m gate, so raising it puts the model
# out-of-distribution AND feeds more non-zero speeds to the CBF. Configurable so
# a measurement run can widen it on a parked robot; the DEFAULT stays 1.8.
DEFAULT_MAX_SPEED_RANGE_M = 1.8


class ObstacleTracker:
    """
    Lightweight centroid tracker — matches detections to existing tracks
    by nearest-centroid distance in global map frame coordinates.
    """
    def __init__(self, max_distance=0.8, max_age=10):
        self.tracks   = {}   # track_id -> {'centroid': (x,y,z), 'centroid_global': (x,y,z), 'age': int, 'history_global': deque, 'visible_count': int}
        self.next_id  = 0
        self.max_dist = max_distance   # metres
        self.max_age  = max_age        # frames before track is dropped
        self.alpha_z  = 0.7            # EMA filtering factor for depth Z (Idea 43)

    def reset(self):
        """Drop all tracks. Used when the input stream breaks (stalled depth or
        missing odometry): resuming with the pre-gap tracks would splice frames
        from either side of the gap into one velocity window, and the window
        features assume uniformly spaced samples."""
        self.tracks = {}

    def update(self, centroids_m, centroids_g):
        """
        centroids_m: list of (x, y, z) in metres relative to robot.
        centroids_g: list of (x, y, z) in global map frame.
        Returns dict of updated tracks.
        """
        # Age all tracks. NOTE: do NOT decrement visible_count here — a track that
        # matches every frame would net zero (decrement here, +1 on match below) and
        # stay pinned at its initial value of 1, making the visible_count >= 3
        # confirmation gate unreachable. Stale tracks are pruned via age/max_age,
        # and unmatched-frame decay is applied after matching below.
        for tid in list(self.tracks):
            self.tracks[tid]['age'] += 1
            if self.tracks[tid]['age'] > self.max_age:
                del self.tracks[tid]

        # Match detections to tracks using global coordinates (Idea 1 & 91)
        unmatched = list(range(len(centroids_g)))
        for tid, track in self.tracks.items():
            if not unmatched:
                break
            tx, ty = track['centroid_global'][:2]
            
            # Vectorized distance norm over unmatched detections (Idea 91)
            det_coords = np.array([centroids_g[i][:2] for i in unmatched], dtype=np.float32)
            dists = np.linalg.norm(det_coords - np.array([tx, ty], dtype=np.float32), axis=1)
            best = int(np.argmin(dists))
            
            if dists[best] < self.max_dist:
                idx = unmatched.pop(best)
                cx_l, cy_l, cz = centroids_m[idx]
                cx_g, cy_g, _  = centroids_g[idx]
                
                # Apply EMA filtering on depth (Idea 43)
                prev_cz = track['history_global'][-1][2] if track['history_global'] else cz
                cz_filtered = self.alpha_z * cz + (1.0 - self.alpha_z) * prev_cz
                
                track['centroid'] = (cx_l, cy_l, cz_filtered)
                track['centroid_global'] = (cx_g, cy_g, cz_filtered)
                track['age']      = 0
                track['visible_count'] += 1  # Increment visible count (Idea 109)
                track['history_global'].append((cx_g, cy_g, cz_filtered))

        # Decay confirmation for tracks that were NOT matched this frame
        # (matched tracks had age reset to 0 above). This keeps visible_count a
        # measure of recent, consecutive visibility without pinning it at 1.
        for tid, track in self.tracks.items():
            if track['age'] > 0:
                track['visible_count'] = max(0, track['visible_count'] - 1)

        # Create new tracks for unmatched detections
        for idx in unmatched:
            if len(self.tracks) >= MAX_OBSTACLES:
                break
            tid = self.next_id
            self.next_id += 1
            cx_l, cy_l, cz = centroids_m[idx]
            cx_g, cy_g, _  = centroids_g[idx]
            
            self.tracks[tid] = {
                'centroid': (cx_l, cy_l, cz),
                'centroid_global': (cx_g, cy_g, cz),
                'age':      0,
                'visible_count': 1,  # Initialize visible count (Idea 109)
                'history_global': deque(maxlen=WINDOW_SIZE),
            }
            self.tracks[tid]['history_global'].append((cx_g, cy_g, cz))

        return {tid: {'centroid': t['centroid'],
                      'centroid_global': t['centroid_global'],
                      'history_global': t['history_global'],
                      'visible_count': t['visible_count']}  # Return visible count (Idea 109)
                for tid, t in self.tracks.items()}


class VelocityEstimator:
    """
    Loads the TorchScript MLP and runs inference at INFER_HZ.
    Designed to run alongside the existing server_x3.py broadcast loop.

    Usage:
        estimator = VelocityEstimator(camera, lidar, robot_pose_fn)
        estimator.start()
        ...
        estimates = estimator.get_estimates()
        # estimates: list of {'id': int, 'x': float, 'y': float,
        #                      'vx': float, 'vy': float, 'speed': float}
        estimator.stop()
    """

    def __init__(self, camera, lidar, robot_pose_fn=None, model_path=None, detections_fn=None):
        """
        camera:        AstraCamera instance (or ROS2Bridge)
        lidar:         YDLidarDriver instance (or ROS2Bridge)
        robot_pose_fn: callable returning {'pose': {'x': m, 'y': m, 'theta': rad},
                                           'twist': {'vx': m/s, 'vy': m/s, 'wz': rad/s}}
                       for EKF slip & ego-motion compensation.
        model_path:    optional string path to the TorchScript model file
        detections_fn: optional callable returning list of latest camera detections
                       for visual-LiDAR fusion gating.
        """
        self.camera        = camera
        self.lidar         = lidar
        self.robot_pose_fn = robot_pose_fn
        self.model_path    = model_path or MODEL_PATH
        self.detections_fn = detections_fn

        self.estimation_enabled = True
        self._model     = None
        self.scaler_X_mean = None
        self.scaler_X_scale = None
        self.scaler_X_inv_scale = None
        self.scaler_y_mean = None
        self.scaler_y_scale = None
        self._tracker   = ObstacleTracker()
        self._estimates = []
        self._lock      = threading.Lock()
        self._running   = False
        self._thread    = None
        self._last_print_time = 0.0
        self._prev_estimates = {}
        self._logged_missing_intrinsics = False
        self._logged_stale_depth = False
        self._logged_missing_pose = False
        self._height_ray_cache = {}
        self._load_ground_plane_config()

        # Pre-allocate input tensor shape (Idea 116)
        self.x_tensor_preallocated = torch.zeros((MAX_OBSTACLES, 40), dtype=torch.float32)

        self._load_model()

    def _load_ground_plane_config(self):
        values = {
            "camera_height_m": DEFAULT_CAMERA_HEIGHT_M,
            "camera_pitch_deg": DEFAULT_CAMERA_PITCH_DEG,
            "min_obstacle_height_m": DEFAULT_MIN_HEIGHT_M,
            "max_obstacle_height_m": DEFAULT_MAX_HEIGHT_M,
            "max_speed_range_m": DEFAULT_MAX_SPEED_RANGE_M,
        }
        try:
            with open(GROUND_PLANE_CONFIG_PATH, "r") as config_file:
                values.update(json.load(config_file))
        except FileNotFoundError:
            logger.warning("VelocityEstimator: ground-plane config missing; using "
                           "measured X3 Plus defaults")
        except Exception as exc:
            logger.error(f"VelocityEstimator: invalid ground-plane config: {exc}; "
                         "using measured X3 Plus defaults")

        self.camera_height_m = float(values["camera_height_m"])
        self.camera_pitch_rad = math.radians(float(values["camera_pitch_deg"]))
        self.min_obstacle_height_m = float(values["min_obstacle_height_m"])
        self.max_obstacle_height_m = float(values["max_obstacle_height_m"])
        self.max_speed_range_m = float(values["max_speed_range_m"])
        if not (self.camera_height_m > 0.0 and
                0.0 <= self.min_obstacle_height_m < self.max_obstacle_height_m):
            raise ValueError("invalid camera ground-plane dimensions")
        # 4.0 m is the depth extraction ceiling; a gate beyond it is meaningless,
        # and one at or below 0 disables speed estimation entirely.
        if not (0.0 < self.max_speed_range_m <= 4.0):
            raise ValueError("max_speed_range_m must be in (0.0, 4.0]")
        if self.max_speed_range_m > DEFAULT_MAX_SPEED_RANGE_M:
            logger.warning(
                "VelocityEstimator: max_speed_range_m=%.2f m exceeds the %.2f m "
                "gate the model was trained at; estimates beyond %.2f m are "
                "out-of-distribution and also reach the CBF",
                self.max_speed_range_m, DEFAULT_MAX_SPEED_RANGE_M,
                DEFAULT_MAX_SPEED_RANGE_M)

    def _height_band_mask(self, depth_m, fy, cy):
        """Return pixels inside the configured height band above the floor."""
        h = depth_m.shape[0]
        key = (h, round(float(fy), 6), round(float(cy), 6),
               round(self.camera_pitch_rad, 8))
        row_term = self._height_ray_cache.get(key)
        if row_term is None:
            rows = np.arange(h, dtype=np.float32)
            # Optical Y is positive down. Positive pitch means the optical axis
            # points down, adding Z*sin(pitch) to vertical distance below camera.
            row_term = (((rows - np.float32(cy)) / np.float32(fy)) *
                        np.float32(math.cos(self.camera_pitch_rad)) +
                        np.float32(math.sin(self.camera_pitch_rad)))
            if len(self._height_ray_cache) > 4:
                self._height_ray_cache.clear()
            self._height_ray_cache[key] = row_term
        height = self.camera_height_m - row_term[:, None] * depth_m
        return ((height > self.min_obstacle_height_m) &
                (height < self.max_obstacle_height_m))

    def _load_model(self):
        try:
            self._model    = torch.jit.load(self.model_path, map_location='cpu')
            self._model.eval()
            
            # JIT Trace optimization (Idea 137)
            dummy_input = torch.zeros((MAX_OBSTACLES, 40), dtype=torch.float32)
            try:
                self._model = torch.jit.trace(self._model, dummy_input)
                logger.info("VelocityEstimator: model JIT traced successfully")
            except Exception as trace_err:
                logger.warning(f"VelocityEstimator: JIT tracing failed (falling back to untraced): {trace_err}")

            # Load scaler parameters directly from JSON (Idea 72)
            with open(SCALER_PARAMS_PATH, "r") as f:
                params = json.load(f)
            
            self.scaler_X_mean = np.array(params["scaler_X"]["mean"], dtype=np.float32)
            self.scaler_X_scale = np.array(params["scaler_X"]["scale"], dtype=np.float32)
            self.scaler_X_inv_scale = (1.0 / self.scaler_X_scale).astype(np.float32)
            self.scaler_y_mean = np.array(params["scaler_y"]["mean"], dtype=np.float32)
            self.scaler_y_scale = np.array(params["scaler_y"]["scale"], dtype=np.float32)
            
            logger.info(f"VelocityEstimator: model {self.model_path} and scaler parameters loaded")
        except Exception as e:
            logger.error(f"VelocityEstimator: failed to load model {self.model_path}: {e}")

    @staticmethod
    def _scale_intrinsics(intrinsics, shape):
        """Scale ``(fx, fy, cx, cy, width, height)`` to an actual frame shape."""
        if intrinsics is None:
            return None
        fx, fy, cx, cy, intr_w, intr_h = intrinsics
        h, w = shape[:2]
        if intr_w and intr_h and (w, h) != (intr_w, intr_h):
            sx, sy = w / float(intr_w), h / float(intr_h)
            fx, cx = fx * sx, cx * sx
            fy, cy = fy * sy, cy * sy
        return float(fx), float(fy), float(cx), float(cy)

    def _extract_depth_centroids(self, depth_frame, raw_depth_frame=None,
                                 depth_intrinsics=None):
        """
        Extract obstacle centroids. If raw_depth_frame is provided, we perform
        thresholding directly on the raw physical depth values in meters to avoid 
        colorization and min-max scaling artifacts. Otherwise, we fall back to
        the colorised BGR thresholding logic.

        Returns list of (x_m, y_m, Z) relative to camera centre.
        """
        source_frame = raw_depth_frame if raw_depth_frame is not None else depth_frame
        intr_full = (self._scale_intrinsics(depth_intrinsics, source_frame.shape)
                     if source_frame is not None else None)
        if intr_full is None:
            if not self._logged_missing_intrinsics:
                logger.warning("VelocityEstimator: depth intrinsics unavailable; "
                               "skipping centroid projection")
                self._logged_missing_intrinsics = True
            return []
        self._logged_missing_intrinsics = False

        if raw_depth_frame is not None:
            # Dynamic Voxel Downsampling (Idea 143)
            # Partition the raw depth frame to extract clean close/far contours:
            # 1. Close-range (< 1.5m): stride 1 (higher resolution, tight voxel size)
            # 2. Far-range (1.5m to 4.0m): stride 3 (lower resolution, larger voxel size)
            h_orig, w_orig = raw_depth_frame.shape[:2]
            mask_orig = np.zeros((h_orig, w_orig), dtype=np.uint8)
            
            # Close-range mask (0.5m to 1.5m, high resolution)
            close_mask = (raw_depth_frame >= 0.5) & (raw_depth_frame < 1.5)
            mask_orig[close_mask] = 255

            # Downsample first, then add the far-range region on the smaller frame.
            raw_depth_frame = raw_depth_frame[::2, ::2]
            mask = mask_orig[::2, ::2]
            fx, fy, cx0, cy0 = intr_full
            fx *= 0.5
            fy *= 0.5
            cx0 *= 0.5
            cy0 *= 0.5

            # Far-range mask on the (already 2x downsampled) frame. NOTE: do NOT
            # zero out alternating rows/cols here — that leaves isolated, non-adjacent
            # set pixels that the 5x5 MORPH_OPEN erosion below erases completely,
            # making every obstacle beyond 1.5 m invisible. Keep the region contiguous.
            far_mask_ds = (raw_depth_frame >= 1.5) & (raw_depth_frame <= 4.0)
            mask[far_mask_ds] = 255

            # Remove floor/ceiling pixels before morphology and contour creation.
            # Once the floor joins a person's feet, no contour-level operation can
            # separate them again.
            mask[~self._height_band_mask(raw_depth_frame, fy, cy0)] = 0
            
            # Morphological cleanup
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            centroids = []
            h, w = raw_depth_frame.shape[:2]
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                # Get rough Z from bounding box center for adaptive area threshold
                x_b_tmp, y_b_tmp, w_b_tmp, h_b_tmp = cv2.boundingRect(cnt)
                cx_tmp = x_b_tmp + w_b_tmp / 2.0
                cy_tmp = y_b_tmp + h_b_tmp / 2.0
                # Sample depth at bounding box center (with bounds checking)
                cy_i = min(int(cy_tmp), raw_depth_frame.shape[0] - 1)
                cx_i = min(int(cx_tmp), raw_depth_frame.shape[1] - 1)
                z_ref = float(raw_depth_frame[cy_i, cx_i]) if not np.isnan(raw_depth_frame[cy_i, cx_i]) else 1.5
                z_ref = max(0.5, min(4.0, z_ref))
                adaptive_min_area = (MIN_BLOB_AREA / 4.0) * (1.5 / z_ref) ** 2
                if area < adaptive_min_area:
                    continue
                M = cv2.moments(cnt)
                if M['m00'] == 0:
                    continue
                cx = M['m10'] / M['m00']
                cy = M['m01'] / M['m00']
                
                # Crop contour bounding box to reduce mask allocation size and indexing overhead (Idea 36)
                x_b, y_b, w_b, h_b = cv2.boundingRect(cnt)
                cnt_mask = np.zeros((h_b, w_b), dtype=np.uint8)
                cv2.drawContours(cnt_mask, [cnt], -1, 255, -1, offset=(-x_b, -y_b))
                
                depth_slice = raw_depth_frame[y_b:y_b+h_b, x_b:x_b+w_b]
                depth_vals = depth_slice[cnt_mask == 255]
                valid_depths = depth_vals[(depth_vals >= 0.5) & (depth_vals <= 4.0) & (~np.isnan(depth_vals))]
                
                if len(valid_depths) == 0:
                    # No valid depth anywhere in the contour. The old behaviour
                    # substituted Z = 1.0, which invents an obstacle at 1.0 m --
                    # inside the 1.8 m gate, so it reached the model and the CBF
                    # as a close object. A contour with no depth carries no
                    # position; drop it.
                    continue
                # Decimate large depth arrays to speed up sorting (Idea 52)
                if len(valid_depths) > 200:
                    valid_depths = valid_depths[::len(valid_depths) // 200]
                Z = float(np.median(valid_depths))
                
                # Convert pixel centroid using the calibrated intrinsics of the
                # active, depth-aligned OAK stream (already scaled to this 2x
                # downsampled mask).
                x_m = (cx - cx0) * Z / fx
                y_m = (cy - cy0) * Z / fy
                
                # Visual-LiDAR Fusion Gating (Idea 146)
                if self.detections_fn is not None:
                    try:
                        detections = self.detections_fn()
                        # Filter for 'person' boxes
                        person_boxes = [d.get("bbox") for d in detections if d.get("label") == "person"]
                        if person_boxes:
                            # Project into the full-resolution depth image. This
                            # gate remains disabled below, but keeping its geometry
                            # calibrated prevents it regressing if re-enabled.
                            fx_full, fy_full, cx_full, cy_full = intr_full
                            u = int(x_m * fx_full / Z + cx_full)
                            v = int(y_m * fy_full / Z + cy_full)
                            
                            inside_any = False
                            for x1, y1, x2, y2 in person_boxes:
                                # 15px padding margin
                                if (x1 - 15) <= u <= (x2 + 15) and (y1 - 15) <= v <= (y2 + 15):
                                    inside_any = True
                                    break
                            if not inside_any:
                                # DELIBERATELY DISABLED -- and NOT because the
                                # geometry is unverified. Verified 2026-09-04:
                                # detections are scaled by oakd_driver nn_w/nn_h
                                # (480x640) and get_raw_depth_frame() is the same
                                # CAM_A-aligned 480x640 via stereo.setOutputSize,
                                # so both live in one frame; the projection above
                                # reduces to u = 2*cx of the 2x-downsampled mask,
                                # which is correct.
                                #
                                # The blocker is scope, not calibration: these
                                # centroids also feed the CBF, so discarding
                                # everything that is not a person would make
                                # collision avoidance blind to boxes, furniture
                                # and the Roomba. Enabling this needs the
                                # estimator and CBF consumers split first.
                                pass
                    except Exception as ex:
                        logger.warning(f"VelocityEstimator: failed visual-lidar gating check: {ex}")
                
                centroids.append((x_m, y_m, Z))
                
            return centroids[:MAX_OBSTACLES]

        if depth_frame is None:
            return []

        # Downsample depth_frame (Idea 81 & 136)
        depth_frame = depth_frame[::2, ::2]
        fx, fy, cx0, cy0 = intr_full
        fx *= 0.5
        fy *= 0.5
        cx0 *= 0.5
        cy0 *= 0.5

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
            if area < (MIN_BLOB_AREA / 4.0):
                continue
            M  = cv2.moments(cnt)
            if M['m00'] == 0:
                continue
            cx = M['m10'] / M['m00']
            cy = M['m01'] / M['m00']

            # Calculate actual Z (depth) in meters using the raw depth frame
            Z = 1.0
            x_m = (cx - cx0) * Z / fx
            y_m = (cy - cy0) * Z / fy

            centroids.append((x_m, y_m, Z))

        return centroids[:MAX_OBSTACLES]

    def _build_window_features(self, history_local):
        """
        Build a (1, 40) feature vector from a list of T (x,y,z) local coordinates.
        Matches the sliding window format used during training:
          [rel_x, rel_y, dx, dy] × T frames, flattened.
        Where:
          rel_x = z (depth / Robot X forward)
          rel_y = -x (camera horizontal offset inverted / Robot Y left)
        Returns: tuple (features_numpy_array, is_stopped_boolean)
        """
        hist = list(history_local)
        # Pad with first entry if history shorter than window
        while len(hist) < WINDOW_SIZE:
            hist.insert(0, hist[0] if hist else (0.0, 0.0, 1.0))

        hist = hist[-WINDOW_SIZE:]
        
        # Determine starting offset for translation normalization (Idea 48)
        rx0 = hist[0][2]      # cz
        ry0 = -hist[0][0]     # -cx
        
        # Check for Kinematic Stop-Trigger Gating (Idea 108)
        is_stopped = False
        if len(hist) >= 3:
            last_3 = hist[-3:]
            disps = []
            for j in range(1, len(last_3)):
                dx_j = last_3[j][2] - last_3[j-1][2]
                dy_j = -last_3[j][0] - (-last_3[j-1][0])
                disps.append(math.hypot(dx_j, dy_j))
            if all(d < 0.01 for d in disps):
                is_stopped = True
        
        features = []
        for i, (cx, cy, cz) in enumerate(hist):
            rx = cz
            ry = -cx
            
            # Apply translation normalization (Idea 48)
            rx_norm = rx - rx0
            ry_norm = ry - ry0
            
            if i == 0:
                dx, dy = 0.0, 0.0
            else:
                rx_prev = hist[i-1][2]
                ry_prev = -hist[i-1][0]
                dx = rx - rx_prev
                dy = ry - ry_prev
                # Clamp displacements to prevent noise spikes from causing extreme predictions (Idea 63)
                dx = np.clip(dx, -0.25, 0.25)
                dy = np.clip(dy, -0.25, 0.25)
            features.extend([rx_norm, ry_norm, dx, dy])

        return np.array(features, dtype=np.float32).reshape(1, -1), is_stopped

    def _inference_loop(self):
        dt = 1.0 / INFER_HZ
        logger.info("VelocityEstimator: inference loop started")

        while self._running:
            t0 = time.monotonic()

            try:
                # 1. Get depth and raw depth frames
                cam_src = self.camera() if callable(self.camera) else self.camera
                depth_frame = cam_src.get_depth_frame() if (cam_src is not None and hasattr(cam_src, 'get_depth_frame')) else None
                raw_depth_frame = cam_src.get_raw_depth_frame() if (cam_src is not None and hasattr(cam_src, 'get_raw_depth_frame')) else None
                depth_intrinsics = cam_src.get_depth_intrinsics() if (cam_src is not None and hasattr(cam_src, 'get_depth_intrinsics')) else None

                # Reject a stalled camera. Without this the loop keeps
                # re-projecting the last frame at INFER_HZ; the centroids are
                # frozen but the odometry used to place them in the global frame
                # is not, so a moving robot manufactures obstacle motion out of a
                # dead camera. Drop the estimates rather than publish stale ones.
                if cam_src is not None and hasattr(cam_src, 'get_depth_frame_age'):
                    try:
                        frame_age = cam_src.get_depth_frame_age()
                    except Exception:
                        frame_age = None
                    if frame_age is not None and frame_age > MAX_DEPTH_FRAME_AGE_S:
                        if not self._logged_stale_depth:
                            logger.warning(
                                "VelocityEstimator: depth frame is %.2f s old "
                                "(> %.2f s); dropping estimates until it recovers",
                                frame_age, MAX_DEPTH_FRAME_AGE_S)
                            self._logged_stale_depth = True
                        with self._lock:
                            self._estimates = []
                        self._tracker.reset()
                        elapsed = time.monotonic() - t0
                        time.sleep(max(0.001, dt - elapsed))
                        continue
                    if self._logged_stale_depth:
                        logger.info("VelocityEstimator: depth frames recovered")
                        self._logged_stale_depth = False

                # Fast-path: skip all processing when no valid depth data exists
                if raw_depth_frame is not None and raw_depth_frame.size > 0:
                    if not np.any((raw_depth_frame >= 0.5) & (raw_depth_frame <= 4.0)):
                        with self._lock:
                            self._estimates = []
                        elapsed = time.monotonic() - t0
                        time.sleep(max(0.001, (1.0 / INFER_HZ) - elapsed))
                        continue

                # 2. Extract local centroids (relative to camera/robot)
                centroids_m = self._extract_depth_centroids(
                    depth_frame, raw_depth_frame, depth_intrinsics)

                # 3. Query robot pose for global mapping & slip compensation (Idea 1 & 11)
                robot_data = None
                if self.robot_pose_fn is not None:
                    try:
                        robot_data = self.robot_pose_fn()
                    except Exception as e:
                        logger.error(f"VelocityEstimator: failed to query robot pose: {e}")

                if self.robot_pose_fn is None:
                    # No odometry source was ever configured: the global frame is
                    # just the robot frame, fixed at the origin. That is a
                    # consistent choice, not a fallback.
                    rx_rob, ry_rob, rtheta_rob = 0.0, 0.0, 0.0
                elif robot_data and "pose" in robot_data:
                    r_pose = robot_data["pose"]
                    rx_rob, ry_rob, rtheta_rob = r_pose["x"], r_pose["y"], r_pose["theta"]
                else:
                    # Odometry was configured but is unavailable this frame.
                    # Substituting the origin silently teleports every centroid by
                    # the robot's true displacement, which either destroys track
                    # association or -- when the robot happens to be near the
                    # origin -- survives association and is read as obstacle
                    # motion. Skip the frame instead of inventing a pose.
                    if not self._logged_missing_pose:
                        logger.warning("VelocityEstimator: robot pose unavailable; "
                                       "skipping frames until odometry returns")
                        self._logged_missing_pose = True
                    with self._lock:
                        self._estimates = []
                    self._tracker.reset()
                    elapsed = time.monotonic() - t0
                    time.sleep(max(0.001, dt - elapsed))
                    continue
                if self._logged_missing_pose:
                    logger.info("VelocityEstimator: robot pose recovered")
                    self._logged_missing_pose = False

                # Compute global coordinates of centroids (Idea 1, 11, & 121)
                centroids_g = []
                cos_r = math.cos(rtheta_rob)
                sin_r = math.sin(rtheta_rob)
                if centroids_m:
                    local_coords = np.array([[cz, -cx_l] for cx_l, cy_l, cz in centroids_m], dtype=np.float32)
                    R = np.array([[cos_r, -sin_r],
                                  [sin_r, cos_r]], dtype=np.float32)
                    T = np.array([rx_rob, ry_rob], dtype=np.float32)
                    global_xy = local_coords @ R.T + T
                    for idx, (cx_l, cy_l, cz) in enumerate(centroids_m):
                        centroids_g.append((float(global_xy[idx, 0]), float(global_xy[idx, 1]), cz))

                # 4. Update tracker using global coordinate matching
                tracks = self._tracker.update(centroids_m, centroids_g)

                # 5. Run MLP inference on active tracks (batched) (Idea 46)
                estimates = []
                eligible_tracks = []
                features_list = []

                for tid, track in tracks.items():
                    # Filter out tracks that do not satisfy the track initiation gate (Idea 109)
                    if track.get('visible_count', 1) < 3:
                        continue

                    # Skip tracks that are beyond the proximity threshold — they contribute zero to speed scaling
                    if track['centroid'][2] > self.max_speed_range_m:
                        cx, cy, cz = track['centroid']
                        estimates.append({
                            'id':    tid,
                            'x':     round(cx, 3),
                            'y':     round(cy, 3),
                            'z':     round(cz, 3),
                            'vx':    0.0,
                            'vy':    0.0,
                            'speed': 0.0,
                        })
                        continue

                    if len(track['history_global']) < 2:
                        cx, cy, cz = track['centroid']
                        estimates.append({
                            'id':    tid,
                            'x':     round(cx, 3),
                            'y':     round(cy, 3),
                            'z':     round(cz, 3),
                            'vx':    0.0,
                            'vy':    0.0,
                            'speed': 0.0,
                        })
                        continue

                    # Default velocity to 0.0 when estimation is disabled or model is not loaded
                    if not self.estimation_enabled or self._model is None:
                        cx, cy, cz = track['centroid']
                        estimates.append({
                            'id':    tid,
                            'x':     round(cx, 3),
                            'y':     round(cy, 3),
                            'z':     round(cz, 3),
                            'vx':    0.0,
                            'vy':    0.0,
                            'speed': 0.0,
                        })
                        continue

                    # Reconstruct relative history for this track in the robot's current frame (Idea 1 & 11)
                    hist_g_arr = np.array(list(track['history_global']), dtype=np.float32)  # shape (T, 3)
                    if len(hist_g_arr) == 0:
                        continue
                    dxy = hist_g_arr[:, :2] - np.array([rx_rob, ry_rob], dtype=np.float32)  # (T, 2)
                    R = np.array([[cos_r, sin_r], [-sin_r, cos_r]], dtype=np.float32)
                    local_xy = dxy @ R.T  # (T, 2): col0=rx_l, col1=ry_l
                    # cx_l = -ry_l, cy_l = 0.0, cz = rx_l
                    hist_local = [(-float(local_xy[i, 1]), 0.0, float(local_xy[i, 0])) for i in range(len(local_xy))]

                    feats, is_stopped = self._build_window_features(hist_local)
                    if is_stopped:  # Kinematic Stop-Trigger Gating (Idea 108)
                        cx, cy, cz = tracks[tid]['centroid']
                        estimates.append({
                            'id':    tid,
                            'x':     round(cx, 3),
                            'y':     round(cy, 3),
                            'z':     round(cz, 3),
                            'vx':    0.0,
                            'vy':    0.0,
                            'speed': 0.0,
                        })
                        continue

                    eligible_tracks.append((tid, hist_local))
                    features_list.append(feats)

                if eligible_tracks and self.estimation_enabled and self._model is not None:
                    # Stack features into a single matrix of shape (N, 40) (Idea 46)
                    features_batch = np.vstack(features_list)
                    
                    # Normalize using parameters (Idea 72)
                    features_scaled = (features_batch - self.scaler_X_mean) * self.scaler_X_inv_scale
                    
                    # Copy directly into pre-allocated PyTorch tensor (Idea 116)
                    num_tracks = len(eligible_tracks)
                    self.x_tensor_preallocated[:num_tracks].copy_(torch.from_numpy(features_scaled))
                    x_tensor = self.x_tensor_preallocated[:num_tracks]

                    with torch.no_grad():
                        pred_scaled = self._model(x_tensor).numpy()

                    # Inverse transform predictions: shape (N, 2)
                    pred_ms = pred_scaled * self.scaler_y_scale + self.scaler_y_mean
                    pred_ms = np.clip(pred_ms, -2.5, 2.5)

                    for idx, (tid, _) in enumerate(eligible_tracks):
                        vx = float(pred_ms[idx, 0])
                        vy = float(pred_ms[idx, 1])
                        speed = float(np.sqrt(vx**2 + vy**2))

                        cx, cy, cz = tracks[tid]['centroid']
                        visible_count = tracks[tid].get('visible_count', WINDOW_SIZE)
                        conf = min(1.0, visible_count / WINDOW_SIZE)
                        estimates.append({
                            'id':    tid,
                            'x':     round(cx, 3),
                            'y':     round(cy, 3),
                            'z':     round(cz, 3),
                            'vx':    round(vx * conf, 3),
                            'vy':    round(vy * conf, 3),
                            'speed': round(speed * conf, 3),
                        })

                # Idea 152: clamp velocity change between frames to max human acceleration (3 m/s²)
                max_delta = 3.0 / INFER_HZ  # max speed change per frame = 0.3 m/s at 10Hz
                for est in estimates:
                    tid = est['id']
                    if tid in self._prev_estimates:
                        prev = self._prev_estimates[tid]
                        dvx = est['vx'] - prev.get('vx', 0.0)
                        dvy = est['vy'] - prev.get('vy', 0.0)
                        if abs(dvx) > max_delta:
                            est['vx'] = prev.get('vx', 0.0) + math.copysign(max_delta, dvx)
                        if abs(dvy) > max_delta:
                            est['vy'] = prev.get('vy', 0.0) + math.copysign(max_delta, dvy)
                        est['speed'] = float(np.sqrt(est['vx']**2 + est['vy']**2))
                self._prev_estimates = {est['id']: est for est in estimates}

                with self._lock:
                    self._estimates = estimates

                # 6. Print calculated velocities (throttled to 2Hz)
                if self.estimation_enabled and estimates:
                    now_t = time.monotonic()
                    if now_t - self._last_print_time >= 0.5:
                        self._last_print_time = now_t
                        print(f"\n[Estimator] Calculated {len(estimates)} velocity estimate(s):")
                        for est in estimates:
                            print(f"  - ID {est['id']}: Centroid=(x={est['x']:.2f}, y={est['y']:.2f}, z={est['z']:.2f})m | "
                                  f"Vel=(vx={est['vx']:.2f}, vy={est['vy']:.2f})m/s | Speed={est['speed']:.2f}m/s")

                # 7. Overlay decisions on depth frame and display window if DISPLAY is set
                if "DISPLAY" in os.environ and depth_frame is not None:
                    debug_frame = depth_frame.copy()
                    h, w = debug_frame.shape[:2]
                    debug_intr = self._scale_intrinsics(depth_intrinsics, debug_frame.shape)

                    for tid, track in tracks.items() if debug_intr is not None else ():
                        cx_m, cy_m, Z = track['centroid']
                        fx, fy, cx0, cy0 = debug_intr
                        img_x = int(cx_m * fx / Z + cx0)
                        img_y = int(cy_m * fy / Z + cy0)

                        if 0 <= img_x < w and 0 <= img_y < h:
                            cv2.circle(debug_frame, (img_x, img_y), 6, (0, 0, 255), -1)

                            # Find matching estimate
                            match_est = None
                            for est in estimates:
                                if est['id'] == tid:
                                    match_est = est
                                    break

                            label = f"ID {tid}: ({cx_m:.2f}, {Z:.2f}m)"
                            if match_est is not None:
                                vx = match_est['vx']
                                vy = match_est['vy']
                                speed = match_est['speed']
                                label += f" v:({vx:.2f}, {vy:.2f}) s:{speed:.2f}m/s"

                                # Draw velocity arrow projected on the image:
                                # Horizontal component: -vy (pedestrian moving left means negative horizontal change in camera)
                                # Vertical component: -vx (pedestrian moving forward/deeper means negative vertical change in camera)
                                scale = 150.0  # pixels per m/s
                                arrow_x = int(img_x - vy * scale / Z)
                                arrow_y = int(img_y - vx * scale / Z)
                                cv2.arrowedLine(debug_frame, (img_x, img_y), (arrow_x, arrow_y), (0, 255, 0), 2, tipLength=0.3)

                            cv2.putText(debug_frame, label, (img_x + 10, img_y - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)

                    try:
                        cv2.imshow("Pedestrian Velocity Estimator Decisions", debug_frame)
                        cv2.waitKey(1)
                    except Exception:
                        pass

            except Exception as e:
                logger.error(f"VelocityEstimator: inference error: {e}")

            # Sleep remainder of cycle
            elapsed = time.monotonic() - t0
            sleep_t = max(0.001, dt - elapsed)
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
