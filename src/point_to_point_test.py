#!/usr/bin/env python3
"""
point_to_point_test.py — Point-to-Point calibration test for Yahboom X3.
Drives the robot 2.0 meters forward, rotates 180°, drives 2.0 meters back, and rotates 180°.

Uses fused odometry feedback (/odom or /odom_raw) for high-accuracy closed-loop control.
"""

import sys
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf2_ros

# --- Control Config ---
TARGET_DISTANCE = 4.0  # meters
ROTATION_ANGLE  = math.pi  # 180 degrees in radians
DIST_TOLERANCE  = 0.05  # meters
YAW_TOLERANCE   = 0.03  # radians (~1.7°) — tighter for better mark consistency

# Proportional gains — tuned conservatively for smooth motion
KP_DIST = 0.6   # Linear velocity gain (lower = smoother deceleration)
KP_YAW  = 0.4   # Heading drift correction during forward drive (gentle)
KP_ROT  = 0.5   # Angular velocity gain during pure rotation (lowered for less overshoot)
KD_ROT  = 0.1   # Derivative velocity gain during pure rotation (active damping)

MAX_LINEAR_SPEED  = 0.20  # m/s — keep low for accuracy
MIN_LINEAR_SPEED  = 0.06  # m/s — just enough to overcome friction
MAX_ANGULAR_SPEED = 0.30  # rad/s — lowered to reduce momentum overshoot
MIN_ANGULAR_SPEED = 0.12  # rad/s — ensure we have power to overcome static friction near target

# Timeout for safety
SEGMENT_TIMEOUT = 20.0  # seconds per segment (increased for slower speeds)

def normalize_angle(angle):
    """Normalize yaw angle to [-pi, pi] range."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

class PointToPointTest(Node):
    def __init__(self):
        super().__init__('x3_point_to_point_test')
        self.get_logger().info("Initializing Point-to-Point Test Node...")

        # Subscriptions & Publishers
        self._odom_sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self._cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)

        # State Variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False
        self.using_amcl = False
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # State Machine
        # 0: INIT
        # 1: DRIVE_FORWARD_2M
        # 2: SETTLE_BEFORE_ROTATE_1
        # 3: ROTATE_180
        # 4: SETTLE_BEFORE_DRIVE_BACK
        # 5: DRIVE_BACK_2M
        # 6: SETTLE_BEFORE_ROTATE_2
        # 7: ROTATE_BACK_180
        # 8: DONE
        self.state = 0
        self.state_start_time = 0.0

        # Segment start references
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0
        self.target_yaw = 0.0

        # Settle duration (non-blocking)
        self.settle_duration = 1.0  # seconds to wait between segments

        # PD control tracking variables
        self.prev_yaw_error = 0.0
        self.last_time = 0.0

        # Create control loop timer at 20Hz
        self._timer = self.create_timer(0.05, self._control_loop)

    def _odom_cb(self, msg: Odometry):
        # Extract X, Y position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Extract yaw angle from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_received = True

    def _stop_robot(self):
        twist = Twist()
        # Send multiple stop commands to ensure the driver receives it
        for _ in range(3):
            self._cmd_pub.publish(twist)

    def _control_loop(self):
        old_x, old_y, old_yaw = self.current_x, self.current_y, self.current_yaw
        
        try:
            trans = self._tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            new_x = trans.transform.translation.x
            new_y = trans.transform.translation.y
            q = trans.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            new_yaw = math.atan2(siny_cosp, cosy_cosp)
            
            if not getattr(self, 'using_amcl', False):
                dx_jump = new_x - old_x
                dy_jump = new_y - old_y
                dyaw_jump = new_yaw - old_yaw
                self.start_x += dx_jump
                self.start_y += dy_jump
                self.start_yaw = normalize_angle(self.start_yaw + dyaw_jump)
                self.target_yaw = normalize_angle(self.target_yaw + dyaw_jump)
                self.using_amcl = True
                self.get_logger().info("Switched localization from Odometry to AMCL (TF map->base_link)")
            
            self.current_x = new_x
            self.current_y = new_y
            self.current_yaw = new_yaw
            self.odom_received = True
            
        except Exception:
            pass

        if not self.odom_received:
            self.get_logger().info("Waiting for /odom or TF messages...", throttle_duration_sec=2.0)
            return

        now = time.monotonic()

        # State 0: Init & Record starting Pose
        if self.state == 0:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.start_yaw = self.current_yaw
            self.target_yaw = self.current_yaw  # maintain heading
            self.state = 1
            self.state_start_time = now
            self.get_logger().info(f"P2P Segment 1: Drive Forward {TARGET_DISTANCE}m "
                                   f"(yaw={math.degrees(self.current_yaw):.1f}°)")
            return

        # Check Timeout for safety (only for active movement states)
        if self.state in [1, 3, 5, 7] and (now - self.state_start_time > SEGMENT_TIMEOUT):
            self.get_logger().error(f"Timeout reached in State {self.state}! Stopping robot for safety.")
            self._stop_robot()
            self.state = 8
            sys.exit(1)

        twist = Twist()

        # State 1: Drive Forward 2 Meters
        if self.state == 1:
            dx = self.current_x - self.start_x
            dy = self.current_y - self.start_y
            dist_travelled = math.hypot(dx, dy)
            dist_error = TARGET_DISTANCE - dist_travelled

            if dist_error <= DIST_TOLERANCE:
                self._stop_robot()
                self.get_logger().info(f"Reached 2m (travelled {dist_travelled:.2f}m). Settling...")
                self.state = 2  # settle
                self.state_start_time = now
                return

            # Proportional velocity with smooth deceleration
            speed = dist_error * KP_DIST
            speed = max(MIN_LINEAR_SPEED, min(MAX_LINEAR_SPEED, speed))

            # Gentle heading drift correction
            yaw_error = normalize_angle(self.target_yaw - self.current_yaw)
            rot_correction = yaw_error * KP_YAW
            rot_correction = max(-0.2, min(0.2, rot_correction))  # limit correction

            twist.linear.x = speed
            twist.angular.z = rot_correction
            self._cmd_pub.publish(twist)

        # State 2: Settle before first rotation
        elif self.state == 2:
            self._stop_robot()
            if now - self.state_start_time >= self.settle_duration:
                self.start_yaw = self.current_yaw
                self.target_yaw = normalize_angle(self.start_yaw + ROTATION_ANGLE)
                yaw_error = normalize_angle(self.target_yaw - self.current_yaw)
                self.prev_yaw_error = yaw_error
                self.last_time = now
                self.state = 3
                self.state_start_time = now
                self.get_logger().info(f"Rotating 180° (from {math.degrees(self.start_yaw):.1f}° "
                                       f"to {math.degrees(self.target_yaw):.1f}°)")

        # State 3: Rotate 180 degrees
        elif self.state == 3:
            yaw_error = normalize_angle(self.target_yaw - self.current_yaw)

            # Prevent chattering/oscillation at the 180-degree boundary.
            # If the yaw error is close to 180 degrees (pi radians), sensor noise/fluctuation 
            # can cause the sign of the error to flip rapidly, making the robot alternate 
            # turning left and right. Force a consistent direction (CCW/positive) to break the tie.
            if abs(yaw_error) > math.pi - 0.1:
                yaw_error = math.pi - 0.05

            if abs(yaw_error) <= YAW_TOLERANCE:
                self._stop_robot()
                self.get_logger().info(f"Rotation complete (error {math.degrees(yaw_error):.1f}°). Settling...")
                self.state = 4  # settle
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

        # State 4: Settle before driving back
        elif self.state == 4:
            self._stop_robot()
            if now - self.state_start_time >= self.settle_duration:
                self.start_x = self.current_x
                self.start_y = self.current_y
                self.start_yaw = self.current_yaw
                self.target_yaw = self.current_yaw
                self.state = 5
                self.state_start_time = now
                self.get_logger().info(f"Driving back {TARGET_DISTANCE}m...")

        # State 5: Drive Back 2 Meters
        elif self.state == 5:
            dx = self.current_x - self.start_x
            dy = self.current_y - self.start_y
            dist_travelled = math.hypot(dx, dy)
            dist_error = TARGET_DISTANCE - dist_travelled

            if dist_error <= DIST_TOLERANCE:
                self._stop_robot()
                self.get_logger().info(f"Returned (travelled {dist_travelled:.2f}m). Settling...")
                self.state = 6  # settle
                self.state_start_time = now
                return

            speed = dist_error * KP_DIST
            speed = max(MIN_LINEAR_SPEED, min(MAX_LINEAR_SPEED, speed))

            yaw_error = normalize_angle(self.target_yaw - self.current_yaw)
            rot_correction = yaw_error * KP_YAW
            rot_correction = max(-0.2, min(0.2, rot_correction))

            twist.linear.x = speed
            twist.angular.z = rot_correction
            self._cmd_pub.publish(twist)

        # State 6: Settle before final rotation
        elif self.state == 6:
            self._stop_robot()
            if now - self.state_start_time >= self.settle_duration:
                self.start_yaw = self.current_yaw
                self.target_yaw = normalize_angle(self.start_yaw + ROTATION_ANGLE)
                yaw_error = normalize_angle(self.target_yaw - self.current_yaw)
                self.prev_yaw_error = yaw_error
                self.last_time = now
                self.state = 7
                self.state_start_time = now
                self.get_logger().info(f"Final rotation 180°...")

        # State 7: Rotate back to start orientation
        elif self.state == 7:
            yaw_error = normalize_angle(self.target_yaw - self.current_yaw)

            # Prevent chattering/oscillation at the 180-degree boundary.
            # If the yaw error is close to 180 degrees (pi radians), sensor noise/fluctuation 
            # can cause the sign of the error to flip rapidly, making the robot alternate 
            # turning left and right. Force a consistent direction (CCW/positive) to break the tie.
            if abs(yaw_error) > math.pi - 0.1:
                yaw_error = math.pi - 0.05

            if abs(yaw_error) <= YAW_TOLERANCE:
                self._stop_robot()
                self.get_logger().info(f"P2P Loop complete! Final yaw error: {math.degrees(yaw_error):.1f}°. Restarting...")
                self.state = 0
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

def main():
    rclpy.init()
    node = PointToPointTest()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        node._stop_robot()
        node.get_logger().info("P2P Test interrupted by user.")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

if __name__ == '__main__':
    main()
