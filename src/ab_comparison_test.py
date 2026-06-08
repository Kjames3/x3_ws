#!/usr/bin/env python3
"""
ab_comparison_test.py — A/B comparison test for the EE244 velocity estimator.

Drives the same 4 m forward / 180° rotate / 4 m back path as
point_to_point_test.py, but labels each run with --mode {reactive,predictive}
and writes a timestamped CSV to src/logs/ for post-run analysis.

Usage:
    python3 ab_comparison_test.py --mode reactive
    python3 ab_comparison_test.py --mode predictive
"""

import sys
import os
import math
import time
import argparse
import csv
from datetime import datetime
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import asyncio
import websockets
import json
import threading
import numpy as np

# ---------------------------------------------------------------------------
# Waypoints (relative to the robot's starting pose at the tape mark).
# WAYPOINT_B is the forward target; the robot returns to WAYPOINT_A.
# ---------------------------------------------------------------------------
WAYPOINT_A_DIST = 0.0   # start — recorded dynamically from odom at init
WAYPOINT_B_DIST = 4.0   # 4 metres forward

ROTATION_ANGLE   = math.pi  # 180 degrees

# ---------------------------------------------------------------------------
# Control constants — identical to point_to_point_test.py for fair comparison
# ---------------------------------------------------------------------------
DIST_TOLERANCE    = 0.05   # metres
YAW_TOLERANCE     = 0.03   # radians (~1.7°)

KP_DIST = 0.6
KP_YAW  = 0.4
KP_ROT  = 0.5
KD_ROT  = 0.1   # Derivative gain for rotation (active damping)

MAX_LINEAR_SPEED  = 0.20   # m/s
MIN_LINEAR_SPEED  = 0.06   # m/s
MAX_ANGULAR_SPEED = 0.30   # rad/s
MIN_ANGULAR_SPEED = 0.12   # rad/s

SEGMENT_TIMEOUT  = 20.0    # seconds
ROTATION_TIMEOUT = 10.0    # seconds — tighter timeout for rotation states
SETTLE_DURATION  = 1.0     # seconds between segments
LOG_HZ           = 10      # rows per second written to CSV

WALL_STOP_DIST   = 0.20    # metres — stop this far from a confirmed static wall and turn back


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


# ---------------------------------------------------------------------------
# CSV run logger
# ---------------------------------------------------------------------------
class RunLogger:
    def __init__(self, mode_label: str, log_dir: str):
        self.mode  = mode_label
        self.rows: list[dict] = []
        self.start = datetime.now()
        os.makedirs(log_dir, exist_ok=True)
        ts = self.start.strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(log_dir, f"ab_{mode_label}_{ts}.csv")

    def log(self, pose: dict, segment: str, n_obstacles: int = 0, max_speed: float = 0.0, min_dist: float = 999.0,
            path_y: float = 0.0, vx_cmd: float = 0.0, vy_cmd: float = 0.0, vy_rep: float = 0.0,
            corrected_x: float = 0.0, corrected_y: float = 0.0, corrected_yaw_deg: float = 0.0):
        self.rows.append({
            "time_s":       round((datetime.now() - self.start).total_seconds(), 3),
            "mode":         self.mode,
            "segment":      segment,
            "robot_x":      round(pose["x"], 4),
            "robot_y":      round(pose["y"], 4),
            "robot_th_deg": round(math.degrees(pose["theta"]), 2),
            "n_obstacles":  n_obstacles,
            "max_obs_speed": max_speed,
            "min_obstacle_dist": round(min_dist, 3),
            "path_y":           round(path_y, 4),
            "vx_cmd":           round(vx_cmd, 3),
            "vy_cmd":           round(vy_cmd, 3),
            "vy_rep":           round(vy_rep, 3),
            "corrected_x":      round(corrected_x, 4),
            "corrected_y":      round(corrected_y, 4),
            "corrected_yaw_deg": round(corrected_yaw_deg, 2),
        })

    def save(self):
        if not self.rows:
            return
        with open(self.path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=self.rows[0].keys())
            writer.writeheader()
            writer.writerows(self.rows)
        print(f"[AB Test] Run log saved: {self.path}")


# ---------------------------------------------------------------------------
# State machine node
# ---------------------------------------------------------------------------
class ABComparisonTest(Node):
    # State constants
    INIT              = 0
    DRIVE_TO_B        = 1
    SETTLE_1          = 2
    ROTATE_180        = 3
    SETTLE_2          = 4
    DRIVE_TO_A        = 5
    SETTLE_3          = 6
    ROTATE_HOME       = 7
    DONE              = 8

    _STATE_NAMES = {
        0: "INIT", 1: "DRIVE_TO_B", 2: "SETTLE_1",
        3: "ROTATE_180", 4: "SETTLE_2", 5: "DRIVE_TO_A",
        6: "SETTLE_3", 7: "ROTATE_HOME", 8: "DONE",
    }

    def __init__(self, mode: str, log_dir: str, distance: float = 4.0, repeat: bool = False):
        super().__init__("x3_ab_comparison_test")
        self.get_logger().info(f"AB Comparison Test — mode: {mode.upper()} | distance: {distance}m | repeat: {repeat}")

        self._mode = mode
        self._log_dir = log_dir
        self.waypoint_b_dist = distance
        self._configured_dist = distance  # preserved for repeat-loop resets
        self.repeat = repeat
        self._csv_logger = RunLogger(mode, log_dir)
        self._latest_estimates = []
        self._estimates_lock = threading.Lock()
        self._set_estimator_mode(enabled=(mode == "predictive"))

        self._odom_sub = self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self._scan_sub = self.create_subscription(LaserScan, "/scan", self._scan_cb, 10)
        self._cmd_pub  = self.create_publisher(Twist, "/cmd_vel", 10)

        self.current_x   = 0.0
        self.current_y   = 0.0
        self.current_yaw = 0.0
        self.odom_received = False

        self.state            = self.INIT
        self.state_start_time = 0.0

        self.start_x   = 0.0
        self.start_y   = 0.0
        self.start_yaw = 0.0
        self.target_yaw = 0.0
        self.init_x    = 0.0
        self.init_y    = 0.0
        self.init_yaw  = 0.0

        self._last_log_time = 0.0

        # LiDAR data
        self.last_scan_ranges = []
        self.last_scan_angle_min = 0.0
        self.last_scan_angle_increment = 0.0

        # Holonomic bypass & Pause variables
        self.target_lateral_offset = 0.0
        self.is_paused = False
        self.state_elapsed_time = 0.0
        self.last_state_time = 0.0
        self.last_vy_cmd = 0.0
        self.last_vx_cmd = 0.0
        self.blocked_time = 0.0   # Active recovery block timer (Idea 31)
        self._pause_exit_time = 0.0  # Timestamp when robot last cleared a pause (Idea 224)
        self._is_backing = False     # Non-blocking recovery backing flag (Idea 200)
        self._backing_start = 0.0   # Timestamp when backing started (Idea 200)

        # LiDAR wall clearances and speed hysteresis (Ideas 80, 110, 122)
        self.wall_left_clearance = 5.0
        self.wall_right_clearance = 5.0
        self.vy_rep = 0.0
        self.smooth_speed_scale = 1.0

        # EMA smoothed wall clearances (Idea 197)
        self._ema_wall_left = 5.0
        self._ema_wall_right = 5.0

        # PD control tracking variables
        self.prev_yaw_error = 0.0
        self.last_time = 0.0
        self._last_debug_print_time = 0.0
        self.current_path_y = 0.0

        # Cached trig for start_yaw (Idea 209)
        self._cos_start_yaw = 1.0
        self._sin_start_yaw = 0.0

        # Static-wall early-stop state
        self._min_forward_lidar = 999.0       # minimum LiDAR range in narrow forward cone
        self._front_is_continuous_wall = False # True when front is a confirmed flat static wall
        self._early_stop_dist = None          # actual distance travelled when early stop fires
        self._wall_confirm_count = 0           # consecutive frames confirming continuous wall (Idea 203)

        # ICP scan matching state
        self.prev_scan_points = np.empty((0, 2), dtype=np.float32)

        self._timer = self.create_timer(0.05, self._control_loop)  # 20 Hz

    def _odom_cb(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_received = True

    def _scan_cb(self, msg: LaserScan):
        self.last_scan_ranges = list(msg.ranges)
        self.last_scan_angle_min = msg.angle_min
        self.last_scan_angle_increment = msg.angle_increment

        # Convert current scan to 2D Cartesian points in the robot's local frame
        curr_pts = []
        for i, r in enumerate(msg.ranges):
            if math.isnan(r) or r < 0.15 or r > 4.0:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            curr_pts.append((r * math.cos(angle), r * math.sin(angle)))

        # Initialize corrected pose to current odom if not set
        if not hasattr(self, 'corrected_x') or self.corrected_x is None:
            self.corrected_x = self.current_x
            self.corrected_y = self.current_y
            self.corrected_yaw = self.current_yaw
            self.prev_odom_pose = (self.current_x, self.current_y, self.current_yaw)
            self.prev_scan_points = np.array(curr_pts, dtype=np.float32) if curr_pts else np.empty((0, 2), dtype=np.float32)
            return

        # If there are not enough points, fallback to using raw odom updates
        if len(curr_pts) < 40 or len(self.prev_scan_points) < 40:
            p_x, p_y, p_yaw = self.prev_odom_pose
            c_x, c_y, c_yaw = self.current_x, self.current_y, self.current_yaw
            dx_g = c_x - p_x
            dy_g = c_y - p_y
            dtheta = normalize_angle(c_yaw - p_yaw)
            
            cos_p = math.cos(p_yaw)
            sin_p = math.sin(p_yaw)
            dx_odom_local = dx_g * cos_p + dy_g * sin_p
            dy_odom_local = -dx_g * sin_p + dy_g * cos_p
            
            cos_corr = math.cos(self.corrected_yaw)
            sin_corr = math.sin(self.corrected_yaw)
            self.corrected_x += dx_odom_local * cos_corr - dy_odom_local * sin_corr
            self.corrected_y += dx_odom_local * sin_corr + dy_odom_local * cos_corr
            self.corrected_yaw = normalize_angle(self.corrected_yaw + dtheta)

            self.prev_scan_points = np.array(curr_pts, dtype=np.float32) if curr_pts else np.empty((0, 2), dtype=np.float32)
            self.prev_odom_pose = (self.current_x, self.current_y, self.current_yaw)
            return

        # Calculate EKF-reported relative motion since last scan in previous robot frame
        p_x, p_y, p_yaw = self.prev_odom_pose
        c_x, c_y, c_yaw = self.current_x, self.current_y, self.current_yaw

        dx_g = c_x - p_x
        dy_g = c_y - p_y
        dtheta = normalize_angle(c_yaw - p_yaw)

        cos_p = math.cos(p_yaw)
        sin_p = math.sin(p_yaw)
        dx_odom_local = dx_g * cos_p + dy_g * sin_p
        dy_odom_local = -dx_g * sin_p + dy_g * cos_p

        # Run 2D ICP Scan Matcher
        try:
            dx_match, dy_match, dtheta_match = self._align_scans_icp(
                self.prev_scan_points, curr_pts, dx_odom_local, dy_odom_local, dtheta
            )
        except Exception as e:
            dx_match, dy_match, dtheta_match = dx_odom_local, dy_odom_local, dtheta

        if abs(dtheta_match) > 0.15:
            dtheta_match = 0.0

        if math.hypot(dx_match, dy_match) > 0.30:
            dx_match, dy_match, dtheta_match = dx_odom_local, dy_odom_local, dtheta

        # Integrate corrected local displacement
        cos_corr = math.cos(self.corrected_yaw)
        sin_corr = math.sin(self.corrected_yaw)
        self.corrected_x += dx_match * cos_corr - dy_match * sin_corr
        self.corrected_y += dx_match * sin_corr + dy_match * cos_corr
        self.corrected_yaw = normalize_angle(self.corrected_yaw + dtheta_match)

        self.prev_scan_points = np.array(curr_pts, dtype=np.float32) if curr_pts else np.empty((0, 2), dtype=np.float32)
        self.prev_odom_pose = (self.current_x, self.current_y, self.current_yaw)

    def _align_scans_icp(self, prev_pts, curr_pts, initial_dx, initial_dy, initial_dtheta):
        P = prev_pts if isinstance(prev_pts, np.ndarray) else np.array(prev_pts, dtype=np.float32)
        Q = np.array(curr_pts, dtype=np.float32)

        if len(P) > 80:
            P = P[::max(1, len(P) // 80)]
        if len(Q) > 80:
            Q = Q[::max(1, len(Q) // 80)]

        c, s = np.cos(initial_dtheta), np.sin(initial_dtheta)
        R = np.array([[c, -s],
                      [s, c]], dtype=np.float32)
        t = np.array([initial_dx, initial_dy], dtype=np.float32)
        Q_trans = (Q @ R.T) + t

        dx_corr, dy_corr, dtheta_corr = 0.0, 0.0, 0.0

        for _ in range(3):
            diff = Q_trans[:, np.newaxis, :] - P[np.newaxis, :, :]
            dists = np.sum(diff ** 2, axis=2)
            idx = np.argmin(dists, axis=1)
            min_dists2 = dists[np.arange(len(idx)), idx]

            valid = min_dists2 < 0.0625  # 0.25^2 threshold
            if np.sum(valid) < 10:
                break

            Q_matched = Q_trans[valid]
            P_matched = P[idx[valid]]

            mu_q = np.mean(Q_matched, axis=0)
            mu_p = np.mean(P_matched, axis=0)

            Q_bar = Q_matched - mu_q
            P_bar = P_matched - mu_p

            H = Q_bar.T @ P_bar
            theta = np.arctan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
            cos_t, sin_t = np.cos(theta), np.sin(theta)
            R_iter = np.array([[cos_t, -sin_t],
                               [sin_t, cos_t]], dtype=np.float32)
            t_iter = mu_p - (mu_q @ R_iter.T)

            Q_trans = (Q_trans @ R_iter.T) + t_iter

            dtheta_corr += theta
            dx_corr_new = dx_corr * cos_t - dy_corr * sin_t + t_iter[0]
            dy_corr_new = dx_corr * sin_t + dy_corr * cos_t + t_iter[1]
            dx_corr = dx_corr_new
            dy_corr = dy_corr_new

            if abs(dx_corr) + abs(dy_corr) + abs(dtheta_corr) < 1e-4:
                break

        # Inlier quality gate: if too few correspondences, fall back to odometry
        if np.sum(valid) < max(1, int(0.20 * len(Q_trans))):
            return float(initial_dx), float(initial_dy), float(initial_dtheta)

        total_dtheta = initial_dtheta + dtheta_corr
        c_c, s_c = np.cos(dtheta_corr), np.sin(dtheta_corr)
        total_dx = initial_dx * c_c - initial_dy * s_c + dx_corr
        total_dy = initial_dx * s_c + initial_dy * c_c + dy_corr

        return float(total_dx), float(total_dy), float(total_dtheta)

    def _update_bypass_offset(self):
        """Analyze obstacle proximity (fusing camera and LiDAR) and update target lateral offset."""
        with self._estimates_lock:
            estimates = list(self._latest_estimates)

        # 1. LiDAR checks: front sector, left sector, right sector (Ideas 80, 110 & Bumper)
        lidar_blocked = False
        left_clearance = 5.0
        right_clearance = 5.0
        
        # Wall Edge Detection variables (lateral wall bounds)
        wall_left_clearance = 5.0
        wall_right_clearance = 5.0

        # Front continuous blockage check variables (continuous wall detection)
        front_blocked_by_lidar = False
        front_has_edges = False
        min_forward_lidar = 999.0  # minimum range in a narrow ±20° forward cone

        if self.last_scan_ranges:
            for i, r in enumerate(self.last_scan_ranges):
                if math.isnan(r) or r < 0.15 or r > 4.0:
                    continue
                angle = normalize_angle(self.last_scan_angle_min + i * self.last_scan_angle_increment + math.pi)
                
                # Check for range discontinuities in adjacent beams (LiDAR Edge Detection)
                is_wall_edge = False
                prev_idx = (i - 1) % len(self.last_scan_ranges)
                next_idx = (i + 1) % len(self.last_scan_ranges)
                prev_r = self.last_scan_ranges[prev_idx]
                next_r = self.last_scan_ranges[next_idx]
                if not math.isnan(prev_r) and abs(r - prev_r) > 0.4:
                    is_wall_edge = True
                if not math.isnan(next_r) and abs(r - next_r) > 0.4:
                    is_wall_edge = True

                # Check if there is any blockage in the front sector within 2.0 meters
                if abs(angle) < 0.52 and r < 2.0:
                    front_blocked_by_lidar = True
                    if is_wall_edge:
                        front_has_edges = True
                    # Track minimum distance in a narrow ±20° cone for wall-stop detection
                    if abs(angle) < 0.35:
                        min_forward_lidar = min(min_forward_lidar, r)

                # Front block: ~30 degrees (0.52 rad) left/right, within 0.75m forward
                if abs(angle) < 0.52:
                    y_lat = r * math.sin(angle)
                    x_fwd = r * math.cos(angle)
                    if abs(y_lat) < 0.4 and x_fwd < 0.75:
                        lidar_blocked = True
                
                # Left side block sector: 45 to 135 degrees (0.785 to 2.356 rad)
                elif 0.785 <= angle <= 2.356:
                    left_clearance = min(left_clearance, r)
                    y_lat = r * math.sin(angle)
                    x_fwd = r * math.cos(angle)
                    # Use ALL points (not just edges) so solid parallel walls register
                    if x_fwd < 0.9:
                        wall_left_clearance = min(wall_left_clearance, y_lat)

                # Right side block sector: -135 to -45 degrees (-2.356 to -0.785 rad)
                elif -2.356 <= angle <= -0.785:
                    right_clearance = min(right_clearance, r)
                    y_lat = r * math.sin(angle)
                    x_fwd = r * math.cos(angle)
                    # Use ALL points (not just edges) so solid parallel walls register
                    if x_fwd < 0.9:
                        wall_right_clearance = min(wall_right_clearance, abs(y_lat))

        self._ema_wall_left  = 0.7 * wall_left_clearance  + 0.3 * self._ema_wall_left
        self._ema_wall_right = 0.7 * wall_right_clearance + 0.3 * self._ema_wall_right
        self.wall_left_clearance  = self._ema_wall_left
        self.wall_right_clearance = self._ema_wall_right

        # 2. Asynchronous dynamic pedestrian identification
        # Check if the camera estimates show a moving human (speed > 0.15 m/s) in our forward path
        is_dynamic_pedestrian = False
        if self._mode == "predictive":
            for est in estimates:
                ox = est.get("x", 0.0)
                oz = est.get("z", est.get("y", 0.0))
                rx = float(oz)
                ry = -float(ox)
                speed = est.get("speed", 0.0)
                # If they are within 1.8m forward and 0.5m lateral and moving
                if 0.0 < rx < 1.8 and abs(ry) < 0.5 and speed > 0.15:
                    is_dynamic_pedestrian = True
                    break

        # 3. Choose adaptive corridors based on target type
        if is_dynamic_pedestrian:
            AVOIDANCE_FORWARD = 1.8  # meters
            AVOIDANCE_LATERAL = 0.45 # meters (wider lateral corridor for humans)
            BYPASS_OFFSET = 0.4      # meters (wider shift to clear humans safely)
            LATERAL_CORRIDOR = 0.45  # meters
        else:
            # Static obstacles (chair/table legs) or reactive mode (where speed is always 0.0)
            AVOIDANCE_FORWARD = 1.5  # meters
            AVOIDANCE_LATERAL = 0.3  # meters (tighter corridor to ignore static objects further to the side)
            BYPASS_OFFSET = 0.25     # meters (smaller shift to avoid hitting side walls in narrow spaces)
            LATERAL_CORRIDOR = 0.3   # meters

        # Dynamic Corridor Width Clearance Scaling (Idea 110)
        corridor_width = left_clearance + right_clearance
        robot_width = 0.30  # meters
        max_allowed_offset = max(0.0, min(BYPASS_OFFSET, 0.5 * (corridor_width - robot_width)))
        BYPASS_OFFSET = max_allowed_offset

        # Continuously clamp any existing bypass offset to the current corridor limits.
        # Prevents the robot from holding a stale wide offset as walls close in.
        if abs(self.target_lateral_offset) > BYPASS_OFFSET:
            self.target_lateral_offset = math.copysign(BYPASS_OFFSET, self.target_lateral_offset)

        # Calculate dynamic side blockage thresholds based on the chosen BYPASS_OFFSET
        SIDE_BLOCK_THRESHOLD = BYPASS_OFFSET + 0.25  # half-width (0.15m) + safety margin (0.10m)
        left_blocked = (left_clearance < SIDE_BLOCK_THRESHOLD)
        right_blocked = (right_clearance < SIDE_BLOCK_THRESHOLD)

        # Check if the front blockage is a continuous wall (Idea 94 - Wall Filter)
        is_continuous_wall = False
        if front_blocked_by_lidar and not front_has_edges:
            # Check if there is any dynamic pedestrian in the forward zone
            has_dynamic_pedestrian = False
            for est in estimates:
                ox = est.get("x", 0.0)
                oz = est.get("z", est.get("y", 0.0))
                rx = float(oz)
                ry = -float(ox)
                speed = est.get("speed", 0.0)
                if 0.0 < rx < 2.0 and abs(ry) < 0.5 and speed > 0.15:
                    has_dynamic_pedestrian = True
                    break
            if not has_dynamic_pedestrian:
                is_continuous_wall = True

        # Wall confirmation debounce (Idea 203)
        if is_continuous_wall:
            self._wall_confirm_count = min(self._wall_confirm_count + 1, 5)
        else:
            self._wall_confirm_count = 0
        self._front_is_continuous_wall = (self._wall_confirm_count >= 2)
        self._min_forward_lidar = min_forward_lidar

        # Potential Field Wall Repulsion Calculation (Idea 141)
        d_safe = 0.55
        d_min = 0.22
        eta = 0.03
        
        f_rep_left = 0.0
        if self.wall_left_clearance < d_safe:
            denom = max(0.01, self.wall_left_clearance - d_min)
            f_rep_left = eta * (1.0 / (denom * denom) - 1.0 / ((d_safe - d_min) * (d_safe - d_min)))
            
        f_rep_right = 0.0
        if self.wall_right_clearance < d_safe:
            denom = max(0.01, self.wall_right_clearance - d_min)
            f_rep_right = eta * (1.0 / (denom * denom) - 1.0 / ((d_safe - d_min) * (d_safe - d_min)))
            
        vy_rep = f_rep_right - f_rep_left
        self.vy_rep = max(-0.12, min(0.12, vy_rep))
        if abs(self.vy_rep) < 0.012:
            self.vy_rep = 0.0

        # 4. Print diagnostic summary (throttled to 2Hz)
        now_t = time.monotonic()
        if now_t - self._last_debug_print_time >= 0.5:
            self._last_debug_print_time = now_t
            speed_scale = self._get_speed_scaling()
            
            print(f"\n--- [AB TEST ROBOT DIAGNOSTICS] ---")
            print(f"  * State:            {self._STATE_NAMES.get(self.state, str(self.state))} | Paused: {self.is_paused}")
            print(f"  * Speed scale:      {speed_scale:.2f} | Last Lateral Cmd (vy_cmd): {self.last_vy_cmd:.3f} m/s")
            print(f"  * Path Tracking:    Cross-Track Error (path_y)={self.current_path_y:.3f}m | Target Lateral Offset={self.target_lateral_offset:.2f}m")
            print(f"  * LiDAR Clearance:  Left Clearance={left_clearance:.2f}m | Right Clearance={right_clearance:.2f}m | vy_rep={self.vy_rep:.3f} m/s")
            print(f"  * Side Blockage:    Left Blocked={left_blocked} | Right Blocked={right_blocked} (Threshold={SIDE_BLOCK_THRESHOLD:.2f}m)")
            print(f"  * Front Obstacle:   LiDAR Front Blocked={lidar_blocked} | Is Wall={is_continuous_wall}")
            
            if estimates:
                print(f"  * Camera Detections ({len(estimates)}):")
                for est in estimates:
                    rx = est.get('z', 0.0)
                    ry = -est.get('x', 0.0)
                    rvx = est.get('vx', 0.0)
                    rvy = est.get('vy', 0.0)
                    speed = est.get('speed', 0.0)
                    print(f"    - ID {est.get('id')}: Pos=(fwd={rx:.2f}m, left={ry:.2f}m) | "
                          f"Vel=(fwd_vel={rvx:.2f}m/s, left_vel={rvy:.2f}m/s) | Speed={speed:.2f}m/s")
            else:
                print(f"  * Camera Detections: None")
            print(f"------------------------------------\n")

        # If LiDAR sees someone right in front, force stop and default bypass to clearer side if not set
        if lidar_blocked:
            self.is_paused = True
            if self.target_lateral_offset == 0.0:
                if is_continuous_wall:
                    # It's a continuous wall, stop safely without strafing
                    self.target_lateral_offset = 0.0
                else:
                    # Calculate clearance for default bypass side selection
                    if right_clearance >= left_clearance:
                        preferred_offset = -BYPASS_OFFSET
                    else:
                        preferred_offset = BYPASS_OFFSET

                    # Choose bypass side based on side clearances and blockage checks
                    if preferred_offset < 0.0:  # Preferred is right
                        if not right_blocked:
                            self.target_lateral_offset = -BYPASS_OFFSET
                        elif not left_blocked:
                            self.target_lateral_offset = BYPASS_OFFSET
                        else:
                            self.target_lateral_offset = 0.0  # Both blocked, stop
                    else:  # Preferred is left
                        if not left_blocked:
                            self.target_lateral_offset = BYPASS_OFFSET
                        elif not right_blocked:
                            self.target_lateral_offset = -BYPASS_OFFSET
                        else:
                            self.target_lateral_offset = 0.0  # Both blocked, stop
            return

        # 4. Camera-based bypass logic
        blocking_obstacle = None
        # Only run camera-based bypass if we are not facing a continuous wall (Idea 94)
        if not is_continuous_wall:
            for est in estimates:
                ox = est.get("x", 0.0)
                oz = est.get("z", est.get("y", 0.0))
                rx = float(oz)
                ry = -float(ox)

                # Check if obstacle is in front and blocking the path
                if 0.0 < rx < AVOIDANCE_FORWARD and abs(ry) < AVOIDANCE_LATERAL:
                    blocking_obstacle = (rx, ry)
                    break

        if blocking_obstacle is not None:
            self.is_paused = True
            if self.target_lateral_offset == 0.0:
                rx, ry = blocking_obstacle
                # If obstacle is centered or to the left, preferred is right. Else preferred is left.
                if ry >= 0.0:
                    preferred_offset = -BYPASS_OFFSET
                else:
                    preferred_offset = BYPASS_OFFSET

                # Choose bypass side based on side clearances and blockage checks
                if preferred_offset < 0.0:  # Preferred is right
                    if not right_blocked:
                        self.target_lateral_offset = -BYPASS_OFFSET
                    elif not left_blocked:
                        self.target_lateral_offset = BYPASS_OFFSET
                    else:
                        self.target_lateral_offset = 0.0  # Both blocked, stop
                else:  # Preferred is left
                    if not left_blocked:
                        self.target_lateral_offset = BYPASS_OFFSET
                    elif not right_blocked:
                        self.target_lateral_offset = -BYPASS_OFFSET
                    else:
                        self.target_lateral_offset = 0.0  # Both blocked, stop
            else:
                # We already have a bypass offset, check if it's blocked by a side wall
                if self.target_lateral_offset > 0.0:  # Left bypass
                    if left_blocked:
                        if not right_blocked:
                            self.target_lateral_offset = -BYPASS_OFFSET
                        else:
                            self.target_lateral_offset = 0.0
                elif self.target_lateral_offset < 0.0:  # Right bypass
                    if right_blocked:
                        if not left_blocked:
                            self.target_lateral_offset = BYPASS_OFFSET
                        else:
                            self.target_lateral_offset = 0.0
        elif lidar_blocked:
            # LiDAR sees someone in front, but camera doesn't track any dynamic pedestrian candidate
            self.is_paused = True
            if self.target_lateral_offset != 0.0:
                # We already have a bypass target (person is in blind spot), check side blockage
                if self.target_lateral_offset > 0.0:  # Left bypass
                    if left_blocked:
                        if not right_blocked:
                            self.target_lateral_offset = -BYPASS_OFFSET
                        else:
                            self.target_lateral_offset = 0.0
                elif self.target_lateral_offset < 0.0:  # Right bypass
                    if right_blocked:
                        if not left_blocked:
                            self.target_lateral_offset = BYPASS_OFFSET
                        else:
                            self.target_lateral_offset = 0.0
            else:
                # No bypass offset was set, and camera didn't see anyone -> static wall!
                # Keep target_lateral_offset = 0.0 to stop in front of the wall without strafing
                pass
        else:
            # Check if we have cleared the obstacle laterally and forward (both camera and LiDAR)
            has_obstacle = False
            for est in estimates:
                ox = est.get("x", 0.0)
                oz = est.get("z", est.get("y", 0.0))
                rx = float(oz)
                ry = -float(ox)
                if 0.0 < rx < 2.0 and abs(ry) < LATERAL_CORRIDOR:
                    has_obstacle = True
                    break

            if not has_obstacle and self.last_scan_ranges:
                for i, r in enumerate(self.last_scan_ranges):
                    if math.isnan(r) or r < 0.15 or r > 1.2:
                        continue
                    angle = normalize_angle(self.last_scan_angle_min + i * self.last_scan_angle_increment + math.pi)
                    if abs(angle) < 0.52:
                        y_lat = r * math.sin(angle)
                        x_fwd = r * math.cos(angle)
                        if abs(y_lat) < LATERAL_CORRIDOR and x_fwd < 1.2:
                            has_obstacle = True
                            break

            if not has_obstacle:
                self.is_paused = False
                self._pause_exit_time = time.monotonic()
                self.target_lateral_offset = 0.0

    def _stop_robot(self):
        twist = Twist()
        for _ in range(3):
            self._cmd_pub.publish(twist)

    def _set_estimator_mode(self, enabled: bool):
        """Toggle velocity estimation on the WebSocket server and listen to readouts."""
        async def _send():
            while True:
                try:
                    async with websockets.connect("ws://localhost:8081") as ws:
                        await ws.send(json.dumps({
                            "type": "set_velocity_estimation",
                            "enabled": enabled
                        }))
                        self.get_logger().info(
                            f"Velocity estimation: {'ON' if enabled else 'OFF'}"
                        )
                        async for message in ws:
                            try:
                                data = json.loads(message)
                                if data.get("type") == "readout":
                                    with self._estimates_lock:
                                        self._latest_estimates = data.get("velocity_estimates", [])
                            except Exception:
                                pass
                except Exception as e:
                    self.get_logger().warn(f"Could not toggle estimator or connection lost: {e}")
                    await asyncio.sleep(1.0)

        threading.Thread(
            target=lambda: asyncio.run(_send()),
            daemon=True
        ).start()

    def _maybe_log(self, segment: str):
        now = time.monotonic()
        if now - self._last_log_time >= 1.0 / LOG_HZ:
            with self._estimates_lock:
                estimates = list(self._latest_estimates)
            n_obstacles = len(estimates)
            max_speed = max([est.get("speed", 0.0) for est in estimates]) if estimates else 0.0

            # Compute Euclidean distance in the 2D ground plane: dist = sqrt(x^2 + z^2)
            dists = []
            for est in estimates:
                ox = est.get("x", 0.0)
                oz = est.get("z", est.get("y", 0.0))  # fallback to y if z is missing
                dists.append(math.hypot(ox, oz))
            min_dist = min(dists) if dists else 999.0

            self._csv_logger.log(
                {"x": self.current_x, "y": self.current_y, "theta": self.current_yaw},
                segment,
                n_obstacles=n_obstacles,
                max_speed=max_speed,
                min_dist=min_dist,
                path_y=self.current_path_y,
                vx_cmd=self.last_vx_cmd,
                vy_cmd=self.last_vy_cmd,
                vy_rep=self.vy_rep,
                corrected_x=self.corrected_x if hasattr(self, 'corrected_x') else 0.0,
                corrected_y=self.corrected_y if hasattr(self, 'corrected_y') else 0.0,
                corrected_yaw_deg=math.degrees(self.corrected_yaw) if hasattr(self, 'corrected_yaw') else 0.0,
            )
            self._last_log_time = now

    def _get_speed_scaling(self) -> float:
        """Compute linear velocity scaling factor based on pedestrian proximity and TTC, projected along travel direction (Idea 70)."""
        with self._estimates_lock:
            estimates = list(self._latest_estimates)

        if not estimates:
            return 1.0

        speed_scale = 1.0
        PROXIMITY_THRESHOLD = 1.8  # meters
        SAFETY_ZONE_MIN = 0.8      # meters
        TTC_THRESHOLD = 3.0        # seconds
        TTC_MIN = 1.0              # seconds
        LATERAL_THRESHOLD = 0.35   # meters (path width corridor, reduced from 0.5)

        # Get last commanded speeds to determine travel angle alpha (Idea 70)
        vx_cmd = getattr(self, 'last_vx_cmd', 0.1)
        vy_cmd = getattr(self, 'last_vy_cmd', 0.0)
        travel_yaw = math.atan2(vy_cmd, vx_cmd) if (abs(vx_cmd) > 0.01 or abs(vy_cmd) > 0.01) else 0.0
        cos_t = math.cos(travel_yaw)
        sin_t = math.sin(travel_yaw)

        for est in estimates:
            ox = est.get("x", 0.0)
            oz = est.get("z", est.get("y", 0.0))
            vx = est.get("vx", 0.0)
            vy = est.get("vy", 0.0)

            # Robot frame relative coordinates.
            # Estimator outputs vx=forward, vy=lateral(left) already in robot frame.
            rx = float(oz)
            ry = -float(ox)
            rvx = float(vx)
            rvy = float(vy)

            # Project coordinates and relative velocities into travel-aligned frame (Idea 70)
            rx_t = rx * cos_t + ry * sin_t
            ry_t = -rx * sin_t + ry * cos_t
            
            rvx_t = rvx * cos_t + rvy * sin_t
            rvy_t = -rvx * sin_t + rvy * cos_t

            # Ignore obstacles behind the robot's direction of travel
            if rx_t <= 0.0:
                continue

            # 1. Proximity Scaling (Reactive) - based on travel-aligned coordinates
            if abs(ry_t) <= LATERAL_THRESHOLD and rx_t < PROXIMITY_THRESHOLD:
                s_p = (rx_t - SAFETY_ZONE_MIN) / (PROXIMITY_THRESHOLD - SAFETY_ZONE_MIN)
                s_p = max(0.0, min(1.0, s_p))
            else:
                s_p = 1.0

            # 2. Time-to-Collision (TTC) Scaling (Predictive) - based on travel-aligned coordinates
            s_t = 1.0
            if self._mode == "predictive":
                d = math.hypot(rx_t, ry_t)
                r_dot_v = rx_t * rvx_t + ry_t * rvy_t
                if r_dot_v < 0:
                    ttc = -(d ** 2) / r_dot_v
                    if ttc < TTC_THRESHOLD:
                        s_t = (ttc - TTC_MIN) / (TTC_THRESHOLD - TTC_MIN)
                        s_t = max(0.0, min(1.0, s_t))

            # Combine proximity and TTC scaling for this obstacle
            s_obs = min(s_p, s_t)
            speed_scale = min(speed_scale, s_obs)

        # Apply predictive TTC speed hysteresis (Idea 122)
        beta = 0.15  # speed recovery smoothing factor
        if speed_scale < self.smooth_speed_scale:
            self.smooth_speed_scale = speed_scale  # brake instantly for safety
        else:
            self.smooth_speed_scale = beta * speed_scale + (1.0 - beta) * self.smooth_speed_scale  # recover speed smoothly
            
        return self.smooth_speed_scale

    def _control_loop(self):
        if not self.odom_received:
            self.get_logger().info("Waiting for /odom...", throttle_duration_sec=2.0)
            return

        now = time.monotonic()

        # Non-blocking recovery backing handler (Idea 200)
        if self._is_backing:
            if now - self._backing_start < 1.5:
                recovery_twist = Twist()
                recovery_twist.linear.x = -0.08
                self._cmd_pub.publish(recovery_twist)
                return
            else:
                self._is_backing = False
                self.blocked_time = 0.0
                self.target_lateral_offset = 0.0
                self.is_paused = False
        dt_state = now - self.last_state_time if self.last_state_time > 0.0 else 0.05
        self.last_state_time = now

        # -- INIT: record start pose --
        if self.state == self.INIT:
            # Reset scan-matched corrected pose to current odom pose
            self.corrected_x = self.current_x
            self.corrected_y = self.current_y
            self.corrected_yaw = self.current_yaw
            self.prev_odom_pose = (self.current_x, self.current_y, self.current_yaw)

            self.start_x   = self.corrected_x
            self.start_y   = self.corrected_y
            self.start_yaw = self.corrected_yaw
            self._cos_start_yaw = math.cos(self.start_yaw)
            self._sin_start_yaw = math.sin(self.start_yaw)
            self.target_yaw = self.corrected_yaw
            self.init_x    = self.corrected_x
            self.init_y    = self.corrected_y
            self.init_yaw  = self.corrected_yaw
            self.state = self.DRIVE_TO_B
            self.state_start_time = now
            self.state_elapsed_time = 0.0
            self.last_state_time = now
            self.target_lateral_offset = 0.0
            self.is_paused = False
            self.last_vy_cmd = 0.0
            self.get_logger().info(
                f"[{self._mode.upper()}] Driving {self.waypoint_b_dist}m to WP-B "
                f"(start yaw={math.degrees(self.current_yaw):.1f}°)"
            )
            return

        # -- Safety timeout for active motion states (paused when blocked) --
        if self.state in (self.DRIVE_TO_B, self.ROTATE_180, self.DRIVE_TO_A, self.ROTATE_HOME):
            if not self.is_paused:
                self.state_elapsed_time += dt_state

            timeout = ROTATION_TIMEOUT if self.state in (self.ROTATE_180, self.ROTATE_HOME) else SEGMENT_TIMEOUT
            if self.state_elapsed_time > timeout:
                self.get_logger().error(
                    f"Timeout in state {self._STATE_NAMES[self.state]}! Stopping."
                )
                self._stop_robot()
                self._csv_logger.save()
                self.state = self.DONE
                sys.exit(1)

        twist = Twist()

        # -- DRIVE_TO_B --
        if self.state == self.DRIVE_TO_B:
            self._maybe_log("drive_to_B")
            dx = self.corrected_x - self.start_x
            dy = self.corrected_y - self.start_y
            dist_travelled = dx * self._cos_start_yaw + dy * self._sin_start_yaw
            dist_error = self.waypoint_b_dist - dist_travelled

            if dist_error <= DIST_TOLERANCE:
                self._stop_robot()
                self.get_logger().info(
                    f"Reached WP-B ({dist_travelled:.2f}m). Settling..."
                )
                self.state = self.SETTLE_1
                self.state_start_time = now
                return

            # Update holonomic bypass state
            self._update_bypass_offset()

            # Static-wall early stop: if a confirmed flat wall is within WALL_STOP_DIST
            # and no dynamic pedestrian is responsible, treat current position as WP-B,
            # record the actual distance travelled, and begin the return sequence.
            if self._front_is_continuous_wall and self._min_forward_lidar < WALL_STOP_DIST:
                self._stop_robot()
                actual_dist = dx * self._cos_start_yaw + dy * self._sin_start_yaw
                self._early_stop_dist = max(0.0, actual_dist)
                self.get_logger().warn(
                    f"[Wall-Stop] Static wall at {self._min_forward_lidar:.2f} m — "
                    f"stopped at {self._early_stop_dist:.2f} m "
                    f"(target was {self.waypoint_b_dist:.2f} m). Turning around."
                )
                self.state = self.SETTLE_1
                self.state_start_time = now
                self.state_elapsed_time = 0.0
                self.target_lateral_offset = 0.0
                self.is_paused = False
                self.last_vy_cmd = 0.0
                return

            # Calculate cross-track error in path frame
            path_y = -dx * self._sin_start_yaw + dy * self._cos_start_yaw
            self.current_path_y = path_y

            # Lateral speed controller with acceleration smoothing
            KP_LATERAL = 0.8
            MAX_LATERAL_SPEED = 0.15
            MAX_LATERAL_ACCEL = 0.5  # m/s^2
            vy_target = (self.target_lateral_offset - path_y) * KP_LATERAL + self.vy_rep
            vy_target = max(-MAX_LATERAL_SPEED, min(MAX_LATERAL_SPEED, vy_target))

            # Apply rate limiting to lateral speed command
            max_dv = MAX_LATERAL_ACCEL * dt_state
            vy_cmd = max(self.last_vy_cmd - max_dv, min(self.last_vy_cmd + max_dv, vy_target))
            self.last_vy_cmd = vy_cmd

            # Active side-wall collision guard (blocking vy_cmd if too close to a wall edge)
            if vy_cmd > 0.0 and self.wall_left_clearance < 0.30:
                vy_cmd = 0.0
            elif vy_cmd < 0.0 and self.wall_right_clearance < 0.30:
                vy_cmd = 0.0

            # Forward speed controller with blended diagonal profiling (Idea 45)
            speed_scale = self._get_speed_scaling()
            if speed_scale <= 0.1:
                self.get_logger().info("Obstacle blocking path! Stopping forward drive...", throttle_duration_sec=2.0)
                speed = 0.0
            else:
                max_speed = MAX_LINEAR_SPEED * speed_scale
                min_speed = MIN_LINEAR_SPEED * min(1.0, speed_scale)
                base_forward_speed = max(min_speed, min(max_speed, dist_error * KP_DIST))

                # Blend forward and lateral velocities (Idea 45)
                lateral_error = abs(self.target_lateral_offset - path_y)
                forward_scale = 1.0 - max(0.0, min(0.8, lateral_error / 0.5))
                speed = base_forward_speed * forward_scale

            # Active recovery timeout logic (Idea 31)
            if self.is_paused or speed == 0.0:
                self.blocked_time += dt_state
            else:
                self.blocked_time = 0.0

            if self.blocked_time > 8.0:
                # Check rear-sector LiDAR protection before recovery backing (Idea 80)
                rear_blocked = False
                if self.last_scan_ranges:
                    for i, r in enumerate(self.last_scan_ranges):
                        if math.isnan(r) or r < 0.15 or r > 0.50:
                            continue
                        angle = normalize_angle(self.last_scan_angle_min + i * self.last_scan_angle_increment + math.pi)
                        if abs(angle) >= 2.35:  # 135 deg to 225 deg
                            rear_blocked = True
                            break
                if rear_blocked:
                    self.get_logger().error("[AB Test] Blocked for > 8 seconds, but rear LiDAR is blocked! Aborting recovery backing.")
                    self.blocked_time = 0.0
                    return

                self.get_logger().warn("[AB Test] Blocked for > 8 seconds! Initiating non-blocking reverse recovery...")
                self._is_backing = True
                self._backing_start = now
                return

            # Heading controller (yaw correction)
            yaw_error = normalize_angle(self.target_yaw - self.corrected_yaw)
            rot_correction = max(-0.2, min(0.2, yaw_error * KP_YAW))

            # Zero out rotation if fully stopped due to obstacle to prevent chattering
            if speed == 0.0 and vy_cmd == 0.0:
                rot_correction = 0.0

            _ramp = min(1.0, (now - self._pause_exit_time) / 0.3) if self._pause_exit_time > 0.0 else 1.0
            speed = speed * _ramp

            twist.linear.x  = speed
            twist.linear.y  = vy_cmd
            twist.angular.z = rot_correction
            self.last_vx_cmd = speed  # Store last speed for travel projection (Idea 70)
            self._cmd_pub.publish(twist)

        # -- SETTLE_1 --
        elif self.state == self.SETTLE_1:
            self._maybe_log("settle_1")
            self._stop_robot()
            if now - self.state_start_time >= SETTLE_DURATION:
                self.start_yaw  = self.corrected_yaw
                self._cos_start_yaw = math.cos(self.start_yaw)
                self._sin_start_yaw = math.sin(self.start_yaw)
                self.target_yaw = normalize_angle(self.init_yaw + ROTATION_ANGLE)
                yaw_error = normalize_angle(self.target_yaw - self.corrected_yaw)
                self.prev_yaw_error = yaw_error
                self.last_time = now
                self.state = self.ROTATE_180
                self.state_start_time = now
                self.last_vy_cmd = 0.0
                self.get_logger().info(
                    f"Rotating 180° ({math.degrees(self.start_yaw):.1f}° → "
                    f"{math.degrees(self.target_yaw):.1f}°)"
                )

        # -- ROTATE_180 --
        elif self.state == self.ROTATE_180:
            self._maybe_log("rotate_180")
            if self.last_scan_ranges:
                _min_r = min((r for r in self.last_scan_ranges if 0.15 < r < 4.0), default=999.0)
                if _min_r < 0.22:
                    self._stop_robot()
                    return
            yaw_error = normalize_angle(self.target_yaw - self.corrected_yaw)
            if abs(yaw_error) > math.pi - 0.1:
                yaw_error = math.pi - 0.05  # break chattering at ±180° boundary

            if abs(yaw_error) <= YAW_TOLERANCE:
                self._stop_robot()
                self.get_logger().info(
                    f"Rotation done (err={math.degrees(yaw_error):.1f}°). Settling..."
                )
                self.state = self.SETTLE_2
                self.state_start_time = now
                return

            dt = now - self.last_time
            if dt > 0.0:
                yaw_error_dot = (yaw_error - self.prev_yaw_error) / dt
            else:
                yaw_error_dot = 0.0
            self.prev_yaw_error = yaw_error
            self.last_time = now

            # PD Controller
            u = yaw_error * KP_ROT + yaw_error_dot * KD_ROT
            speed_magnitude = abs(u)
            speed_magnitude = max(MIN_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, speed_magnitude))
            rot_speed = math.copysign(speed_magnitude, u)

            twist.angular.z = rot_speed
            self._cmd_pub.publish(twist)

        # -- SETTLE_2 --
        elif self.state == self.SETTLE_2:
            self._maybe_log("settle_2")
            self._stop_robot()
            if now - self.state_start_time >= SETTLE_DURATION:
                # Use the actual distance travelled if an early wall-stop fired,
                # so DRIVE_TO_A drives exactly back to the true starting position.
                effective_dist = self._early_stop_dist if self._early_stop_dist is not None else self.waypoint_b_dist
                self._early_stop_dist = None  # consume — reset for next leg
                self.start_x   = self.init_x + effective_dist * math.cos(self.init_yaw)
                self.start_y   = self.init_y + effective_dist * math.sin(self.init_yaw)
                self.start_yaw = normalize_angle(self.init_yaw + ROTATION_ANGLE)
                self._cos_start_yaw = math.cos(self.start_yaw)
                self._sin_start_yaw = math.sin(self.start_yaw)
                self.target_yaw = self.start_yaw
                self.state = self.DRIVE_TO_A
                self.state_start_time = now
                self.state_elapsed_time = 0.0
                self.last_state_time = now
                self.target_lateral_offset = 0.0
                self.is_paused = False
                self.blocked_time = 0.0
                self.last_vy_cmd = 0.0
                # DRIVE_TO_A will drive effective_dist back; reuse the same variable
                self.waypoint_b_dist = effective_dist
                self.get_logger().info(f"Driving {effective_dist:.2f}m back to WP-A...")

        # -- DRIVE_TO_A --
        elif self.state == self.DRIVE_TO_A:
            self._maybe_log("drive_to_A")
            dx = self.corrected_x - self.start_x
            dy = self.corrected_y - self.start_y
            dist_travelled = dx * self._cos_start_yaw + dy * self._sin_start_yaw
            dist_error = self.waypoint_b_dist - dist_travelled

            if dist_error <= DIST_TOLERANCE:
                self._stop_robot()
                self.get_logger().info(
                    f"Returned to WP-A ({dist_travelled:.2f}m). Settling..."
                )
                self.state = self.SETTLE_3
                self.state_start_time = now
                return

            # Update holonomic bypass state
            self._update_bypass_offset()

            # Static-wall early stop on return leg: same logic as DRIVE_TO_B.
            # If an unexpected wall appears in the return path, stop and settle cleanly.
            if self._front_is_continuous_wall and self._min_forward_lidar < WALL_STOP_DIST:
                self._stop_robot()
                self.get_logger().warn(
                    f"[Wall-Stop] Static wall at {self._min_forward_lidar:.2f} m on return leg — settling."
                )
                self.state = self.SETTLE_3
                self.state_start_time = now
                self.target_lateral_offset = 0.0
                self.is_paused = False
                self.last_vy_cmd = 0.0
                return

            # Calculate cross-track error in path frame
            path_y = -dx * self._sin_start_yaw + dy * self._cos_start_yaw
            self.current_path_y = path_y

            # Lateral speed controller with acceleration smoothing
            KP_LATERAL = 0.8
            MAX_LATERAL_SPEED = 0.15
            MAX_LATERAL_ACCEL = 0.5  # m/s^2
            vy_target = (self.target_lateral_offset - path_y) * KP_LATERAL + self.vy_rep
            vy_target = max(-MAX_LATERAL_SPEED, min(MAX_LATERAL_SPEED, vy_target))

            # Apply rate limiting to lateral speed command
            max_dv = MAX_LATERAL_ACCEL * dt_state
            vy_cmd = max(self.last_vy_cmd - max_dv, min(self.last_vy_cmd + max_dv, vy_target))
            self.last_vy_cmd = vy_cmd

            # Active side-wall collision guard (blocking vy_cmd if too close to a wall edge)
            if vy_cmd > 0.0 and self.wall_left_clearance < 0.30:
                vy_cmd = 0.0
            elif vy_cmd < 0.0 and self.wall_right_clearance < 0.30:
                vy_cmd = 0.0

            # Forward speed controller with blended diagonal profiling (Idea 45)
            speed_scale = self._get_speed_scaling()
            if speed_scale <= 0.1:
                self.get_logger().info("Obstacle blocking path! Stopping return drive...", throttle_duration_sec=2.0)
                speed = 0.0
            else:
                max_speed = MAX_LINEAR_SPEED * speed_scale
                min_speed = MIN_LINEAR_SPEED * min(1.0, speed_scale)
                base_forward_speed = max(min_speed, min(max_speed, dist_error * KP_DIST))

                # Blend forward and lateral velocities (Idea 45)
                lateral_error = abs(self.target_lateral_offset - path_y)
                forward_scale = 1.0 - max(0.0, min(0.8, lateral_error / 0.5))
                speed = base_forward_speed * forward_scale

            # Active recovery timeout logic (Idea 31)
            if self.is_paused or speed == 0.0:
                self.blocked_time += dt_state
            else:
                self.blocked_time = 0.0

            if self.blocked_time > 8.0:
                # Check rear-sector LiDAR protection before recovery backing (Idea 80)
                rear_blocked = False
                if self.last_scan_ranges:
                    for i, r in enumerate(self.last_scan_ranges):
                        if math.isnan(r) or r < 0.15 or r > 0.50:
                            continue
                        angle = normalize_angle(self.last_scan_angle_min + i * self.last_scan_angle_increment + math.pi)
                        if abs(angle) >= 2.35:  # 135 deg to 225 deg
                            rear_blocked = True
                            break
                if rear_blocked:
                    self.get_logger().error("[AB Test] Blocked for > 8 seconds, but rear LiDAR is blocked! Aborting recovery backing.")
                    self.blocked_time = 0.0
                    return

                self.get_logger().warn("[AB Test] Blocked for > 8 seconds! Initiating non-blocking reverse recovery...")
                self._is_backing = True
                self._backing_start = now
                return

            # Heading controller (yaw correction)
            yaw_error = normalize_angle(self.target_yaw - self.corrected_yaw)
            rot_correction = max(-0.2, min(0.2, yaw_error * KP_YAW))

            # Zero out rotation if fully stopped due to obstacle to prevent chattering
            if speed == 0.0 and vy_cmd == 0.0:
                rot_correction = 0.0

            _ramp = min(1.0, (now - self._pause_exit_time) / 0.3) if self._pause_exit_time > 0.0 else 1.0
            speed = speed * _ramp

            twist.linear.x  = speed
            twist.linear.y  = vy_cmd
            twist.angular.z = rot_correction
            self.last_vx_cmd = speed  # Store last speed for travel projection (Idea 70)
            self._cmd_pub.publish(twist)

        # -- SETTLE_3 --
        elif self.state == self.SETTLE_3:
            self._maybe_log("settle_3")
            self._stop_robot()
            if now - self.state_start_time >= SETTLE_DURATION:
                self.start_yaw  = self.corrected_yaw
                self.target_yaw = normalize_angle(self.init_yaw)
                yaw_error = normalize_angle(self.target_yaw - self.corrected_yaw)
                self.prev_yaw_error = yaw_error
                self.last_time = now
                self.state = self.ROTATE_HOME
                self.state_start_time = now
                self.get_logger().info("Final rotation back to start heading...")

        # -- ROTATE_HOME --
        elif self.state == self.ROTATE_HOME:
            self._maybe_log("rotate_home")
            if self.last_scan_ranges:
                _min_r = min((r for r in self.last_scan_ranges if 0.15 < r < 4.0), default=999.0)
                if _min_r < 0.22:
                    self._stop_robot()
                    return
            yaw_error = normalize_angle(self.target_yaw - self.corrected_yaw)
            if abs(yaw_error) > math.pi - 0.1:
                yaw_error = math.pi - 0.05

            if abs(yaw_error) <= YAW_TOLERANCE:
                self._stop_robot()
                if self.repeat:
                    self.get_logger().info(
                        f"[{self._mode.upper()}] Run segment complete (err={math.degrees(yaw_error):.1f}°). Saving log and repeating..."
                    )
                    self._csv_logger.save()
                    
                    # Reset logger for next run
                    self._csv_logger = RunLogger(self._mode, self._log_dir)
                    
                    # Re-initialize pose reference point to current odom readings and reset corrected pose
                    self.corrected_x = self.current_x
                    self.corrected_y = self.current_y
                    self.corrected_yaw = self.current_yaw
                    self.prev_odom_pose = (self.current_x, self.current_y, self.current_yaw)
                    self.prev_scan_points = np.empty((0, 2), dtype=np.float32)

                    self.start_x   = self.corrected_x
                    self.start_y   = self.corrected_y
                    self.start_yaw = self.corrected_yaw
                    self.target_yaw = self.corrected_yaw
                    self.init_x    = self.corrected_x
                    self.init_y    = self.corrected_y
                    self.init_yaw  = self.corrected_yaw
                    
                    # Restore full configured distance and clear any early-stop state
                    self.waypoint_b_dist = self._configured_dist
                    self._early_stop_dist = None

                    # Transition state back to DRIVE_TO_B
                    self.state = self.DRIVE_TO_B
                    self.state_start_time = now
                    self.state_elapsed_time = 0.0
                    self.last_state_time = now
                    self.target_lateral_offset = 0.0
                    self.is_paused = False
                    self.last_vy_cmd = 0.0
                    self.blocked_time = 0.0
                    self.get_logger().info(
                        f"[{self._mode.upper()}] (REPEAT) Driving {self.waypoint_b_dist}m to WP-B "
                        f"(start yaw={math.degrees(self.current_yaw):.1f}°)"
                    )
                    return
                else:
                    self.get_logger().info(
                        f"[{self._mode.upper()}] Run complete. "
                        f"Final yaw error: {math.degrees(yaw_error):.1f}°"
                    )
                    self._csv_logger.save()
                    self.state = self.DONE
                    rclpy.shutdown()
                    sys.exit(0)

            dt = now - self.last_time
            if dt > 0.0:
                yaw_error_dot = (yaw_error - self.prev_yaw_error) / dt
            else:
                yaw_error_dot = 0.0
            self.prev_yaw_error = yaw_error
            self.last_time = now

            # PD Controller
            u = yaw_error * KP_ROT + yaw_error_dot * KD_ROT
            speed_magnitude = abs(u)
            speed_magnitude = max(MIN_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, speed_magnitude))
            rot_speed = math.copysign(speed_magnitude, u)

            twist.angular.z = rot_speed
            self._cmd_pub.publish(twist)


def main():
    parser = argparse.ArgumentParser(description="A/B comparison test for EE244 velocity estimator")
    parser.add_argument(
        "--mode",
        choices=["reactive", "predictive"],
        required=True,
        help="Label for this run: 'reactive' (estimator OFF) or 'predictive' (estimator ON)",
    )
    parser.add_argument(
        "--log-dir",
        default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs"),
        help="Directory to write CSV logs (default: src/logs/)",
    )
    parser.add_argument(
        "--distance",
        type=float,
        default=4.0,
        help="Distance the robot will drift forward and back",
    )
    parser.add_argument(
        "--repeat",
        action="store_true",
        help="Repeat the action continuously until canceled",
    )
    parser.add_argument(
        "--domain-id",
        type=int,
        default=None,
        dest="domain_id",
        help="ROS_DOMAIN_ID to use (default: inherit from environment; Jetson uses 42)",
    )
    args = parser.parse_args()

    if args.domain_id is not None:
        os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)

    rclpy.init()
    node = ABComparisonTest(
        mode=args.mode,
        log_dir=args.log_dir,
        distance=args.distance,
        repeat=args.repeat,
    )
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        node._stop_robot()
        node._csv_logger.save()
        node.get_logger().info("AB Test interrupted.")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass


if __name__ == "__main__":
    main()
