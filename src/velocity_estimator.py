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

WINDOW_SIZE   = 10          # frames of history (matches training T=10)
INFER_HZ      = 10          # target inference rate
MIN_BLOB_AREA = 500         # pixels — ignore tiny depth blobs
MAX_OBSTACLES = 5           # track at most N obstacles simultaneously
MAX_RANGE_M   = 5.0         # ignore detections beyond this distance

# Range within which velocity is actually estimated. Must cover the consumer's
# time-to-collision horizon: ab_comparison_test uses TTC_THRESHOLD = 3.0 s, so at a
# 1.4 m/s closing speed the estimator needs to report motion from ~4 m out. This was
# previously 1.8 m, which force-zeroed the velocity of every approaching pedestrian
# until they were ~1.3 s from contact and left the predictive/TTC lane permanently
# starved (stationary obstacles still worked, since those use proximity, not TTC).
# 4.0 m matches the depth segmentation range gate below.
VELOCITY_MAX_RANGE_M = 4.0

# Depth segmentation range gate (metres).
DEPTH_NEAR_M  = 0.5
DEPTH_FAR_M   = 4.0

# Track confirmation: a track must be matched this many frames before it is reported.
MIN_VISIBLE_FRAMES = 3

# --- Ground / structure rejection -------------------------------------------------
# Camera optical centre height above the floor, from yahboomcar_X3.urdf:
#   base_footprint -> base_link  z = 0.0815
#   base_link      -> oak-d-base-frame  z = 0.185   (rpy 0 0 0, i.e. no tilt)
CAM_HEIGHT_M = 0.0815 + 0.185      # 0.2665 m
# Keep only depth pixels whose height above the floor falls in this band. Removes the
# floor (which otherwise merges into every obstacle contour and drags its centroid
# toward a near-stationary average) and the ceiling/overhangs.
HEIGHT_BAND_MIN_M = 0.15
HEIGHT_BAND_MAX_M = 2.00
# Reject a contour whose metric width exceeds this — walls and long furniture runs
# span far wider than any person, and their centroid is meaningless as an obstacle.
# Deliberately generous (a single person is ~0.5 m): two people walking abreast merge
# into one contour at ~1.2 m, and dropping a group would be far worse than keeping the
# occasional wall, which the LiDAR wall-detection handles independently anyway.
MAX_BLOB_WIDTH_M  = 2.00
# Reject a contour spanning more than this much depth — a compact obstacle is roughly
# fronto-parallel, whereas a wall receding from the camera covers a large depth range.
MAX_BLOB_DEPTH_SPAN_M = 1.50

# Fallback depth intrinsics (Orbbec Astra Pro, 640x480, fx≈fy≈554) used only when the
# active depth source cannot report its own calibration. The OAK-D Lite's real values
# differ by ~1.6x, so these are a last resort, not a default.
FALLBACK_FX = 554.0
FALLBACK_FY = 554.0


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

        # Pre-allocate input tensor shape (Idea 116)
        self.x_tensor_preallocated = torch.zeros((MAX_OBSTACLES, 40), dtype=torch.float32)

        # Depth intrinsics for the frame we actually process (post-downsample),
        # resolved from the live source each cycle and cached by (source, shape).
        self._intr_cache_key = None
        self._intr = None            # (fx, fy, cx, cy) for the downsampled frame
        self._intr_warned = False

        self._load_model()

    def _resolve_intrinsics(self, cam_src, shape):
        """Return (fx, fy, cx, cy) valid for a depth frame of `shape` (h, w).

        Asks the active source for its own calibration (OakDCamera.get_depth_intrinsics)
        and rescales it to `shape`, which is the 2x-downsampled frame this module works
        on. Falls back to Astra constants only if the source cannot report calibration.
        """
        h, w = shape[:2]
        key = (id(cam_src), w, h)
        if key == self._intr_cache_key and self._intr is not None:
            return self._intr

        fx = fy = cx = cy = None
        getter = getattr(cam_src, "get_depth_intrinsics", None)
        if callable(getter):
            try:
                intr = getter()
                if intr:
                    i_fx, i_fy, i_cx, i_cy, i_w, i_h = intr
                    # Rescale from the resolution calibration was read at to ours.
                    sx = float(w) / float(i_w)
                    sy = float(h) / float(i_h)
                    fx, fy = i_fx * sx, i_fy * sy
                    cx, cy = i_cx * sx, i_cy * sy
            except Exception as e:
                logger.warning(f"VelocityEstimator: get_depth_intrinsics failed: {e}")

        if fx is None:
            # No calibration available — scale the Astra defaults from 640x480. These
            # are correct for an actual Astra, and merely approximate for anything else.
            fx = FALLBACK_FX * (w / 640.0)
            fy = FALLBACK_FY * (h / 480.0)
            cx, cy = w / 2.0, h / 2.0
            if not self._intr_warned:
                self._intr_warned = True
                logger.warning(
                    f"VelocityEstimator: depth source exposes no intrinsics; falling back "
                    f"to Astra defaults scaled to {w}x{h} (fx={fx:.1f} fy={fy:.1f}). "
                    f"Lateral velocity may be biased."
                )
            # Deliberately do NOT cache the fallback. The OAK reads its calibration
            # asynchronously after the device opens, so an early frame can arrive before
            # it is available; caching here would pin the wrong intrinsics for the life
            # of the process and feed a wrong fy into the height-band ground rejection,
            # which can mask out real people. Retry every cycle until the real values
            # show up (a dict lookup on the driver — negligible).
            return (float(fx), float(fy), float(cx), float(cy))

        logger.info(f"VelocityEstimator: depth intrinsics @ {w}x{h} "
                    f"fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}")
        self._intr = (float(fx), float(fy), float(cx), float(cy))
        self._intr_cache_key = key
        self._intr_warned = False   # re-arm, so a later loss of calibration warns again
        return self._intr

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

    def _extract_depth_centroids(self, depth_frame, raw_depth_frame=None, intr=None):
        """
        Extract obstacle centroids. If raw_depth_frame is provided, we perform
        thresholding directly on the raw physical depth values in meters to avoid 
        colorization and min-max scaling artifacts. Otherwise, we fall back to
        the colorised BGR thresholding logic.

        Returns list of (x_m, y_m, Z) relative to camera centre.
        """
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

            # Far-range mask on the (already 2x downsampled) frame. NOTE: do NOT
            # zero out alternating rows/cols here — that leaves isolated, non-adjacent
            # set pixels that the 5x5 MORPH_OPEN erosion below erases completely,
            # making every obstacle beyond 1.5 m invisible. Keep the region contiguous.
            far_mask_ds = (raw_depth_frame >= 1.5) & (raw_depth_frame <= DEPTH_FAR_M)
            mask[far_mask_ds] = 255

            # Ground-plane / ceiling rejection. Without this the mask is a pure range
            # gate, so the floor in front of the robot is "obstacle" and merges into
            # every contour — the resulting centroid is a floor+person average that
            # barely moves when the person walks, suppressing the very motion we are
            # trying to measure. Back-project each row to a metric height and keep only
            # the band a person occupies. The camera has no tilt (URDF rpy 0 0 0), so
            # height depends only on the row and the depth at that pixel.
            if intr is not None:
                _fx_i, _fy_i, _cx_i, _cy_i = intr
                rows = np.arange(mask.shape[0], dtype=np.float32).reshape(-1, 1)
                # Optical Y is +down, so height above floor = cam height - y_metric.
                y_metric = (rows - _cy_i) * raw_depth_frame / _fy_i
                height = CAM_HEIGHT_M - y_metric
                in_band = (height >= HEIGHT_BAND_MIN_M) & (height <= HEIGHT_BAND_MAX_M)
                mask[~in_band] = 0

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
                z_ref = max(DEPTH_NEAR_M, min(DEPTH_FAR_M, z_ref))
                adaptive_min_area = (MIN_BLOB_AREA / 4.0) * (1.5 / z_ref) ** 2
                if area < adaptive_min_area:
                    continue

                # Structure rejection: discard contours too wide to be a person-sized
                # obstacle (walls, furniture runs). Metric width is scale-invariant, so
                # one threshold works at every range.
                if intr is not None:
                    blob_width_m = w_b_tmp * z_ref / intr[0]
                    if blob_width_m > MAX_BLOB_WIDTH_M:
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
                valid_depths = depth_vals[(depth_vals >= DEPTH_NEAR_M) & (depth_vals <= DEPTH_FAR_M)
                                          & (~np.isnan(depth_vals))]

                if len(valid_depths) > 0:
                    # Decimate large depth arrays to speed up sorting (Idea 52)
                    if len(valid_depths) > 200:
                        valid_depths = valid_depths[::len(valid_depths) // 200]
                    # Reject contours spanning a large depth range — a compact obstacle
                    # is roughly fronto-parallel, a wall receding from the camera is not.
                    if len(valid_depths) >= 8:
                        d_lo, d_hi = np.percentile(valid_depths, [5, 95])
                        if float(d_hi - d_lo) > MAX_BLOB_DEPTH_SPAN_M:
                            continue
                    # 25th percentile, not median: bias to the nearer surface so any
                    # background still inside the contour does not push Z (and the X/Y
                    # that scale with it) too far out. Matches OakDCamera._locate.
                    Z = float(np.percentile(valid_depths, 25))
                else:
                    continue  # no usable depth — drop rather than invent Z = 1.0

                # Convert pixel centroid to metres using the real depth intrinsics for
                # this frame (already rescaled for the 2x downsample). fx and fy differ
                # substantially on the OAK-D Lite because its preview is squeezed
                # anisotropically, so they must not be shared.
                if intr is not None:
                    fx_i, fy_i, cx_i_pp, cy_i_pp = intr
                else:
                    fx_i = FALLBACK_FX * (w / 640.0)
                    fy_i = FALLBACK_FY * (h / 480.0)
                    cx_i_pp, cy_i_pp = w / 2.0, h / 2.0
                x_m = (cx - cx_i_pp) * Z / fx_i
                y_m = (cy - cy_i_pp) * Z / fy_i
                
                # Visual-LiDAR Fusion Gating (Idea 146)
                if self.detections_fn is not None:
                    try:
                        detections = self.detections_fn()
                        # Filter for 'person' boxes
                        person_boxes = [d.get("bbox") for d in detections if d.get("label") == "person"]
                        if person_boxes:
                            # Project (x_m, y_m, Z) to full 640x480 resolution image space
                            fx_full = 554.0
                            cx_full = 320.0
                            cy_full = 240.0
                            u = int(x_m * fx_full / Z + cx_full)
                            v = int(y_m * fx_full / Z + cy_full)
                            
                            inside_any = False
                            for x1, y1, x2, y2 in person_boxes:
                                # 15px padding margin
                                if (x1 - 15) <= u <= (x2 + 15) and (y1 - 15) <= v <= (y2 + 15):
                                    inside_any = True
                                    break
                            if not inside_any:
                                # Centroid does not match any person detection, discard
                                # continue # [DISABLED FOR TESTING] - allow tracking of all objects
                                pass
                    except Exception as ex:
                        logger.warning(f"VelocityEstimator: failed visual-lidar gating check: {ex}")
                
                centroids.append((x_m, y_m, Z))
                
            return centroids[:MAX_OBSTACLES]

        if depth_frame is None:
            return []

        # Downsample depth_frame (Idea 81 & 136)
        depth_frame = depth_frame[::2, ::2]

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
            fx = 277.0
            x_m = (cx - w / 2.0) * Z / fx
            y_m = (cy - h / 2.0) * Z / fx

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

                # Fast-path: skip all processing when no valid depth data exists
                if raw_depth_frame is not None and raw_depth_frame.size > 0:
                    if not np.any((raw_depth_frame >= 0.5) & (raw_depth_frame <= 4.0)):
                        with self._lock:
                            self._estimates = []
                        elapsed = time.monotonic() - t0
                        time.sleep(max(0.001, (1.0 / INFER_HZ) - elapsed))
                        continue

                # 2. Extract local centroids (relative to camera/robot).
                # Resolve intrinsics for the frame we will actually process — the raw
                # frame is downsampled 2x inside _extract_depth_centroids.
                intr = None
                if raw_depth_frame is not None and raw_depth_frame.size > 0:
                    ds_shape = (raw_depth_frame.shape[0] // 2, raw_depth_frame.shape[1] // 2)
                    intr = self._resolve_intrinsics(cam_src, ds_shape)
                centroids_m = self._extract_depth_centroids(depth_frame, raw_depth_frame, intr=intr)

                # 3. Query robot pose for global mapping & slip compensation (Idea 1 & 11)
                robot_data = None
                if self.robot_pose_fn is not None:
                    try:
                        robot_data = self.robot_pose_fn()
                    except Exception as e:
                        logger.error(f"VelocityEstimator: failed to query robot pose: {e}")

                if robot_data and "pose" in robot_data:
                    r_pose = robot_data["pose"]
                    rx_rob, ry_rob, rtheta_rob = r_pose["x"], r_pose["y"], r_pose["theta"]
                else:
                    rx_rob, ry_rob, rtheta_rob = 0.0, 0.0, 0.0

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
                    if track.get('visible_count', 1) < MIN_VISIBLE_FRAMES:
                        continue

                    # Beyond the estimation range there is no usable depth, so report
                    # the track's position with zero velocity rather than guessing.
                    # NOTE: this threshold must stay >= the consumer's TTC horizon —
                    # see VELOCITY_MAX_RANGE_M.
                    if track['centroid'][2] > VELOCITY_MAX_RANGE_M:
                        cx, cy, cz = track['centroid']
                        estimates.append({
                            'id':    tid,
                            'x':     round(cx, 3),
                            'y':     round(cy, 3),
                            'z':     round(cz, 3),
                            'vx':    0.0,
                            'vy':    0.0,
                            'speed': 0.0,
                            'conf':  round(min(1.0, track.get('visible_count', 0) / WINDOW_SIZE), 3),
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
                            'conf':  round(min(1.0, track.get('visible_count', 0) / WINDOW_SIZE), 3),
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
                            'conf':  round(min(1.0, track.get('visible_count', 0) / WINDOW_SIZE), 3),
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
                            'conf':  round(min(1.0, track.get('visible_count', 0) / WINDOW_SIZE), 3),
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
                        # Report the velocity as estimated. Previously it was multiplied
                        # by conf, which attenuated exactly the case that matters most: a
                        # pedestrian walking into view is by definition a NEW track, so
                        # its first reported speed was scaled by 3/10 and only reached
                        # full weight after ~1 s — by which time an approaching person
                        # has already closed most of the reaction distance. Confidence is
                        # now a separate field for consumers to gate on; the
                        # MIN_VISIBLE_FRAMES check above remains the admission test.
                        estimates.append({
                            'id':    tid,
                            'x':     round(cx, 3),
                            'y':     round(cy, 3),
                            'z':     round(cz, 3),
                            'vx':    round(vx, 3),
                            'vy':    round(vy, 3),
                            'speed': round(speed, 3),
                            'conf':  round(conf, 3),
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
                    fx = 554.0

                    for tid, track in tracks.items():
                        cx_m, cy_m, Z = track['centroid']
                        img_x = int(cx_m * fx / Z + w / 2.0)
                        img_y = int(cy_m * fx / Z + h / 2.0)

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