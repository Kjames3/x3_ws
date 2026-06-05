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
YAW_TOLERANCE     = 0.08   # radians (~4.6°)

KP_DIST = 0.6
KP_YAW  = 0.4
KP_ROT  = 0.6
KD_ROT  = 0.1   # Derivative gain for rotation (active damping)

MAX_LINEAR_SPEED  = 0.20   # m/s
MIN_LINEAR_SPEED  = 0.06   # m/s
MAX_ANGULAR_SPEED = 0.40   # rad/s
MIN_ANGULAR_SPEED = 0.0    # rad/s

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

        self._last_log_time = 0.0

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
        if self._mode != "predictive":
            return 1.0

        with self._estimates_lock:
            estimates = list(self._latest_estimates)

        if not estimates:
            return 1.0

        speed_scale = 1.0
        PROXIMITY_THRESHOLD = 1.8  # meters
        SAFETY_ZONE_MIN = 0.8      # meters
        TTC_THRESHOLD = 3.0        # seconds
        TTC_MIN = 1.0              # seconds

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

            d = math.hypot(rx, ry)

            # 1. Proximity Scaling
            if d < PROXIMITY_THRESHOLD:
                s_p = (d - SAFETY_ZONE_MIN) / (PROXIMITY_THRESHOLD - SAFETY_ZONE_MIN)
                s_p = max(0.1, min(1.0, s_p))
            else:
                s_p = 1.0

            # 2. Time-to-Collision (TTC) Scaling
            s_t = 1.0
            r_dot_v = rx * rvx + ry * rvy
            if r_dot_v < 0:
                ttc = -(d ** 2) / r_dot_v
                if ttc < TTC_THRESHOLD:
                    s_t = (ttc - TTC_MIN) / (TTC_THRESHOLD - TTC_MIN)
                    s_t = max(0.1, min(1.0, s_t))

            # Combine proximity and TTC scaling for this obstacle
            s_obs = min(s_p, s_t)
            speed_scale = min(speed_scale, s_obs)

        return speed_scale

    def _control_loop(self):
        if not self.odom_received:
            self.get_logger().info("Waiting for /odom...", throttle_duration_sec=2.0)
            return

        now = time.monotonic()

        # -- INIT: record start pose --
        if self.state == self.INIT:
            self.start_x   = self.current_x
            self.start_y   = self.current_y
            self.start_yaw = self.current_yaw
            self.target_yaw = self.current_yaw
            self.state = self.DRIVE_TO_B
            self.state_start_time = now
            self.get_logger().info(
                f"[{self._mode.upper()}] Driving {WAYPOINT_B_DIST}m to WP-B "
                f"(start yaw={math.degrees(self.current_yaw):.1f}°)"
            )
            return

        # -- Safety timeout for active motion states --
        if self.state in (self.DRIVE_TO_B, self.ROTATE_180, self.DRIVE_TO_A, self.ROTATE_HOME):
            if now - self.state_start_time > SEGMENT_TIMEOUT:
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
            dist_travelled = math.hypot(dx, dy)
            dist_error = WAYPOINT_B_DIST - dist_travelled

            if dist_error <= DIST_TOLERANCE:
                self._stop_robot()
                self.get_logger().info(
                    f"Reached WP-B ({dist_travelled:.2f}m). Settling..."
                )
                self.state = self.SETTLE_1
                self.state_start_time = now
                return

            speed_scale = self._get_speed_scaling()
            max_speed = MAX_LINEAR_SPEED * speed_scale
            min_speed = MIN_LINEAR_SPEED * min(1.0, speed_scale)
            speed = max(min_speed, min(max_speed, dist_error * KP_DIST))
            if speed_scale <= 0.1:
                speed = 0.0

            yaw_error = normalize_angle(self.target_yaw - self.current_yaw)
            rot_correction = max(-0.2, min(0.2, yaw_error * KP_YAW))
            twist.linear.x  = speed
            twist.angular.z = rot_correction
            self._cmd_pub.publish(twist)

        # -- SETTLE_1 --
        elif self.state == self.SETTLE_1:
            self._maybe_log("settle_1")
            self._stop_robot()
            if now - self.state_start_time >= SETTLE_DURATION:
                self.start_yaw  = self.current_yaw
                self.target_yaw = normalize_angle(self.start_yaw + ROTATION_ANGLE)
                yaw_error = normalize_angle(self.target_yaw - self.current_yaw)
                self.prev_yaw_error = yaw_error
                self.last_time = now
                self.state = self.ROTATE_180
                self.state_start_time = now
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
                self.start_x   = self.current_x
                self.start_y   = self.current_y
                self.start_yaw = self.current_yaw
                self.target_yaw = self.current_yaw
                self.state = self.DRIVE_TO_A
                self.state_start_time = now
                self.get_logger().info(f"Driving {WAYPOINT_B_DIST}m back to WP-A...")

        # -- DRIVE_TO_A --
        elif self.state == self.DRIVE_TO_A:
            self._maybe_log("drive_to_A")
            dx = self.current_x - self.start_x
            dy = self.current_y - self.start_y
            dist_travelled = math.hypot(dx, dy)
            dist_error = WAYPOINT_B_DIST - dist_travelled

            if dist_error <= DIST_TOLERANCE:
                self._stop_robot()
                self.get_logger().info(
                    f"Returned to WP-A ({dist_travelled:.2f}m). Settling..."
                )
                self.state = self.SETTLE_3
                self.state_start_time = now
                return

            speed_scale = self._get_speed_scaling()
            max_speed = MAX_LINEAR_SPEED * speed_scale
            min_speed = MIN_LINEAR_SPEED * min(1.0, speed_scale)
            speed = max(min_speed, min(max_speed, dist_error * KP_DIST))
            if speed_scale <= 0.1:
                speed = 0.0

            yaw_error = normalize_angle(self.target_yaw - self.current_yaw)
            rot_correction = max(-0.2, min(0.2, yaw_error * KP_YAW))
            twist.linear.x  = speed
            twist.angular.z = rot_correction
            self._cmd_pub.publish(twist)

        # -- SETTLE_3 --
        elif self.state == self.SETTLE_3:
            self._maybe_log("settle_3")
            self._stop_robot()
            if now - self.state_start_time >= SETTLE_DURATION:
                self.start_yaw  = self.current_yaw
                self.target_yaw = normalize_angle(self.start_yaw + ROTATION_ANGLE)
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
