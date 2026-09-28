#!/usr/bin/env python3
"""Hand-rotation heading check: how far do /odom and the IMU say the robot turned?

  source /opt/ros/humble/setup.bash && ROS_DOMAIN_ID=42 python3 scripts/yaw_check.py

Line the robot up with a wall or tile edge, press Enter, rotate it BY HAND
exactly 90 degrees (don't drive it), press Enter again. It prints the turn
each source measured; repeat as often as you like, Ctrl+C to quit. A correct
source reads ~90. Note /odom_raw takes its yaw from the same IMU.
"""
import math
import threading

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Watch:
    """Unwrapped yaw per source, plus the integrated raw gyro."""

    def __init__(self, node):
        self.lock = threading.Lock()
        self.yaw = {}
        self.gyro_int = 0.0
        self.gyro_t = None
        for name in ('/odom', '/odom_raw'):
            node.create_subscription(Odometry, name,
                                     lambda m, n=name: self._set(n, yaw(m.pose.pose.orientation)), 20)
        node.create_subscription(Imu, '/imu/data', lambda m: self._set('/imu/data', yaw(m.orientation)),
                                 qos_profile_sensor_data)
        node.create_subscription(Imu, '/imu/data_raw', self._gyro, qos_profile_sensor_data)

    def _set(self, name, y):
        with self.lock:
            prev = self.yaw.get(name)
            if prev is not None:   # unwrap
                y = prev + math.atan2(math.sin(y - prev), math.cos(y - prev))
            self.yaw[name] = y

    def _gyro(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        with self.lock:
            if self.gyro_t is not None and 0 < t - self.gyro_t < 0.2:
                self.gyro_int += m.angular_velocity.z * (t - self.gyro_t)
            self.gyro_t = t

    def snapshot(self):
        with self.lock:
            s = dict(self.yaw)
            s['gyro (raw, integrated)'] = self.gyro_int
            return s


def main():
    rclpy.init()
    node = rclpy.create_node('yaw_check')
    w = Watch(node)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    try:
        while True:
            input('\nAlign the robot, then press Enter to start... ')
            a = w.snapshot()
            if not a:
                print('No heading data yet -- is x3_server running? (ROS_DOMAIN_ID=42)')
                continue
            input('Rotate it BY HAND exactly 90 deg, then press Enter... ')
            b = w.snapshot()
            print(f'{"source":24s} turned (deg)')
            for k in sorted(set(a) & set(b)):
                print(f'{k:24s} {math.degrees(b[k] - a[k]):+8.1f}')
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
