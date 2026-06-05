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
SETTLE_DURATION  = 1.0     # seconds between segments
LOG_HZ           = 10      # rows per second written to CSV


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

    def log(self, pose: dict, segment: str, n_obstacles: int = 0, max_speed: float = 0.0, min_dist: float = 999.0):
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

    def __init__(self, mode: str, log_dir: str):
        super().__init__("x3_ab_comparison_test")
        self.get_logger().info(f"AB Comparison Test — mode: {mode.upper()}")

        self._mode = mode
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

        # PD control tracking variables
        self.prev_yaw_error = 0.0
        self.last_time = 0.0

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

    def _update_bypass_offset(self):
        """Analyze obstacle proximity (fusing camera and LiDAR) and update target lateral offset."""
        with self._estimates_lock:
            estimates = list(self._latest_estimates)

        # 1. LiDAR checks: front sector, left sector, right sector
        lidar_blocked = False
        left_blocked = False
        right_blocked = False

        if self.last_scan_ranges:
            for i, r in enumerate(self.last_scan_ranges):
                if math.isnan(r) or r < 0.15 or r > 2.0:
                    continue
                angle = normalize_angle(self.last_scan_angle_min + i * self.last_scan_angle_increment)
                
                # Front block: ~30 degrees (0.52 rad) left/right, within 0.75m forward
                if abs(angle) < 0.52:
                    y_lat = r * math.sin(angle)
                    x_fwd = r * math.cos(angle)
                    if abs(y_lat) < 0.4 and x_fwd < 0.75:
                        lidar_blocked = True
                
                # Left side block: 60 to 120 degrees (1.047 to 2.094 rad), within 0.45m range
                elif 1.047 <= angle <= 2.094:
                    if r < 0.45:
                        left_blocked = True
                
                # Right side block: -120 to -60 degrees (-2.094 to -1.047 rad), within 0.45m range
                elif -2.094 <= angle <= -1.047:
                    if r < 0.45:
                        right_blocked = True

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

        # If LiDAR sees someone right in front, force stop and default bypass to clearer side if not set
        if lidar_blocked:
            self.is_paused = True
            if self.target_lateral_offset == 0.0:
                # Calculate clearance for default bypass side selection
                left_clearance = 5.0
                right_clearance = 5.0
                if self.last_scan_ranges:
                    for i, r in enumerate(self.last_scan_ranges):
                        if math.isnan(r) or r < 0.1:
                            continue
                        angle = normalize_angle(self.last_scan_angle_min + i * self.last_scan_angle_increment)
                        if 0.785 <= angle <= 2.356:
                            left_clearance = min(left_clearance, r)
                        elif -2.356 <= angle <= -0.785:
                            right_clearance = min(right_clearance, r)
                
                if right_clearance >= left_clearance:
                    self.target_lateral_offset = -BYPASS_OFFSET  # Strafing right
                else:
                    self.target_lateral_offset = BYPASS_OFFSET   # Strafing left
            return

        # 4. Camera-based bypass logic
        blocking_obstacle = None
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
                    angle = normalize_angle(self.last_scan_angle_min + i * self.last_scan_angle_increment)
                    if abs(angle) < 0.52:
                        y_lat = r * math.sin(angle)
                        x_fwd = r * math.cos(angle)
                        if abs(y_lat) < LATERAL_CORRIDOR and x_fwd < 1.2:
                            has_obstacle = True
                            break

            if not has_obstacle:
                self.is_paused = False
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
            )
            self._last_log_time = now

    def _get_speed_scaling(self) -> float:
        """Compute linear velocity scaling factor based on pedestrian proximity and TTC."""
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

        for est in estimates:
            ox = est.get("x", 0.0)
            oz = est.get("z", est.get("y", 0.0))
            vx = est.get("vx", 0.0)
            vy = est.get("vy", 0.0)

            # Robot frame relative coordinates
            rx = float(oz)
            ry = -float(ox)
            rvx = float(vy)
            rvy = -float(vx)

            # Ignore obstacles behind the robot
            if rx <= 0.0:
                continue

            # 1. Proximity Scaling (Reactive) - only triggers if obstacle is within the lateral path corridor
            if abs(ry) <= LATERAL_THRESHOLD and rx < PROXIMITY_THRESHOLD:
                s_p = (rx - SAFETY_ZONE_MIN) / (PROXIMITY_THRESHOLD - SAFETY_ZONE_MIN)
                s_p = max(0.0, min(1.0, s_p))
            else:
                s_p = 1.0

            # 2. Time-to-Collision (TTC) Scaling (Predictive) - only active in predictive mode
            s_t = 1.0
            if self._mode == "predictive":
                d = math.hypot(rx, ry)
                r_dot_v = rx * rvx + ry * rvy
                if r_dot_v < 0:
                    ttc = -(d ** 2) / r_dot_v
                    if ttc < TTC_THRESHOLD:
                        s_t = (ttc - TTC_MIN) / (TTC_THRESHOLD - TTC_MIN)
                        s_t = max(0.0, min(1.0, s_t))

            # Combine proximity and TTC scaling for this obstacle
            s_obs = min(s_p, s_t)
            speed_scale = min(speed_scale, s_obs)

        return speed_scale

    def _control_loop(self):
        if not self.odom_received:
            self.get_logger().info("Waiting for /odom...", throttle_duration_sec=2.0)
            return

        now = time.monotonic()
        dt_state = now - self.last_state_time if self.last_state_time > 0.0 else 0.05
        self.last_state_time = now

        # -- INIT: record start pose --
        if self.state == self.INIT:
            self.start_x   = self.current_x
            self.start_y   = self.current_y
            self.start_yaw = self.current_yaw
            self.target_yaw = self.current_yaw
            self.init_x    = self.current_x
            self.init_y    = self.current_y
            self.init_yaw  = self.current_yaw
            self.state = self.DRIVE_TO_B
            self.state_start_time = now
            self.state_elapsed_time = 0.0
            self.last_state_time = now
            self.target_lateral_offset = 0.0
            self.is_paused = False
            self.last_vy_cmd = 0.0
            self.get_logger().info(
                f"[{self._mode.upper()}] Driving {WAYPOINT_B_DIST}m to WP-B "
                f"(start yaw={math.degrees(self.current_yaw):.1f}°)"
            )
            return

        # -- Safety timeout for active motion states (paused when blocked) --
        if self.state in (self.DRIVE_TO_B, self.ROTATE_180, self.DRIVE_TO_A, self.ROTATE_HOME):
            if not self.is_paused:
                self.state_elapsed_time += dt_state
            
            if self.state_elapsed_time > SEGMENT_TIMEOUT:
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
            dx = self.current_x - self.start_x
            dy = self.current_y - self.start_y
            dist_travelled = dx * math.cos(self.start_yaw) + dy * math.sin(self.start_yaw)
            dist_error = WAYPOINT_B_DIST - dist_travelled

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

            # Calculate cross-track error in path frame
            path_y = -dx * math.sin(self.start_yaw) + dy * math.cos(self.start_yaw)

            # Lateral speed controller with acceleration smoothing
            KP_LATERAL = 0.8
            MAX_LATERAL_SPEED = 0.15
            MAX_LATERAL_ACCEL = 0.5  # m/s^2
            vy_target = (self.target_lateral_offset - path_y) * KP_LATERAL
            vy_target = max(-MAX_LATERAL_SPEED, min(MAX_LATERAL_SPEED, vy_target))

            # Apply rate limiting to lateral speed command
            max_dv = MAX_LATERAL_ACCEL * dt_state
            vy_cmd = max(self.last_vy_cmd - max_dv, min(self.last_vy_cmd + max_dv, vy_target))
            self.last_vy_cmd = vy_cmd

            # Forward speed controller
            if self.is_paused and abs(self.target_lateral_offset - path_y) > 0.15:
                # Actively shifting laterally to clear the obstacle
                self.get_logger().info("Strafing laterally to bypass obstacle...", throttle_duration_sec=2.0)
                speed = 0.0
            else:
                speed_scale = self._get_speed_scaling()
                if speed_scale <= 0.1 or self.is_paused:
                    self.get_logger().info("Obstacle blocking path! Pausing forward drive...", throttle_duration_sec=2.0)
                    speed = 0.0
                else:
                    max_speed = MAX_LINEAR_SPEED * speed_scale
                    min_speed = MIN_LINEAR_SPEED * min(1.0, speed_scale)
                    speed = max(min_speed, min(max_speed, dist_error * KP_DIST))

            # Heading controller (yaw correction)
            yaw_error = normalize_angle(self.target_yaw - self.current_yaw)
            rot_correction = max(-0.2, min(0.2, yaw_error * KP_YAW))

            # Zero out rotation if fully stopped due to obstacle to prevent chattering
            if speed == 0.0 and vy_cmd == 0.0:
                rot_correction = 0.0

            twist.linear.x  = speed
            twist.linear.y  = vy_cmd
            twist.angular.z = rot_correction
            self._cmd_pub.publish(twist)

        # -- SETTLE_1 --
        elif self.state == self.SETTLE_1:
            self._maybe_log("settle_1")
            self._stop_robot()
            if now - self.state_start_time >= SETTLE_DURATION:
                self.start_yaw  = self.current_yaw
                self.target_yaw = normalize_angle(self.init_yaw + ROTATION_ANGLE)
                yaw_error = normalize_angle(self.target_yaw - self.current_yaw)
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
            yaw_error = normalize_angle(self.target_yaw - self.current_yaw)
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
                self.start_x   = self.init_x + WAYPOINT_B_DIST * math.cos(self.init_yaw)
                self.start_y   = self.init_y + WAYPOINT_B_DIST * math.sin(self.init_yaw)
                self.start_yaw = normalize_angle(self.init_yaw + ROTATION_ANGLE)
                self.target_yaw = self.start_yaw
                self.state = self.DRIVE_TO_A
                self.state_start_time = now
                self.state_elapsed_time = 0.0
                self.last_state_time = now
                self.target_lateral_offset = 0.0
                self.is_paused = False
                self.last_vy_cmd = 0.0
                self.get_logger().info(f"Driving {WAYPOINT_B_DIST}m back to WP-A...")

        # -- DRIVE_TO_A --
        elif self.state == self.DRIVE_TO_A:
            self._maybe_log("drive_to_A")
            dx = self.current_x - self.start_x
            dy = self.current_y - self.start_y
            dist_travelled = dx * math.cos(self.start_yaw) + dy * math.sin(self.start_yaw)
            dist_error = WAYPOINT_B_DIST - dist_travelled

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

            # Calculate cross-track error in path frame
            path_y = -dx * math.sin(self.start_yaw) + dy * math.cos(self.start_yaw)

            # Lateral speed controller with acceleration smoothing
            KP_LATERAL = 0.8
            MAX_LATERAL_SPEED = 0.15
            MAX_LATERAL_ACCEL = 0.5  # m/s^2
            vy_target = (self.target_lateral_offset - path_y) * KP_LATERAL
            vy_target = max(-MAX_LATERAL_SPEED, min(MAX_LATERAL_SPEED, vy_target))

            # Apply rate limiting to lateral speed command
            max_dv = MAX_LATERAL_ACCEL * dt_state
            vy_cmd = max(self.last_vy_cmd - max_dv, min(self.last_vy_cmd + max_dv, vy_target))
            self.last_vy_cmd = vy_cmd

            # Forward speed controller
            if self.is_paused and abs(self.target_lateral_offset - path_y) > 0.15:
                # Actively shifting laterally to clear the obstacle
                self.get_logger().info("Strafing laterally to bypass obstacle...", throttle_duration_sec=2.0)
                speed = 0.0
            else:
                speed_scale = self._get_speed_scaling()
                if speed_scale <= 0.1 or self.is_paused:
                    self.get_logger().info("Obstacle blocking path! Pausing return drive...", throttle_duration_sec=2.0)
                    speed = 0.0
                else:
                    max_speed = MAX_LINEAR_SPEED * speed_scale
                    min_speed = MIN_LINEAR_SPEED * min(1.0, speed_scale)
                    speed = max(min_speed, min(max_speed, dist_error * KP_DIST))

            # Heading controller (yaw correction)
            yaw_error = normalize_angle(self.target_yaw - self.current_yaw)
            rot_correction = max(-0.2, min(0.2, yaw_error * KP_YAW))

            # Zero out rotation if fully stopped due to obstacle to prevent chattering
            if speed == 0.0 and vy_cmd == 0.0:
                rot_correction = 0.0

            twist.linear.x  = speed
            twist.linear.y  = vy_cmd
            twist.angular.z = rot_correction
            self._cmd_pub.publish(twist)

        # -- SETTLE_3 --
        elif self.state == self.SETTLE_3:
            self._maybe_log("settle_3")
            self._stop_robot()
            if now - self.state_start_time >= SETTLE_DURATION:
                self.start_yaw  = self.current_yaw
                self.target_yaw = normalize_angle(self.init_yaw)
                yaw_error = normalize_angle(self.target_yaw - self.current_yaw)
                self.prev_yaw_error = yaw_error
                self.last_time = now
                self.state = self.ROTATE_HOME
                self.state_start_time = now
                self.get_logger().info("Final rotation back to start heading...")

        # -- ROTATE_HOME --
        elif self.state == self.ROTATE_HOME:
            self._maybe_log("rotate_home")
            yaw_error = normalize_angle(self.target_yaw - self.current_yaw)
            if abs(yaw_error) > math.pi - 0.1:
                yaw_error = math.pi - 0.05

            if abs(yaw_error) <= YAW_TOLERANCE:
                self._stop_robot()
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
    args = parser.parse_args()

    rclpy.init()
    node = ABComparisonTest(mode=args.mode, log_dir=args.log_dir)
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
