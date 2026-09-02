#!/usr/bin/env python3
"""Measure stationary heading drift in /odom -- the number the upgrade targets.

Before the vel_raw swap, /odom's yaw was anchored by the MPU9250 (the EKF gives
odom0 both yaw and vyaw, imu0 only vyaw), so the new IMU could not fix drift no
matter how good it was. This measures the end result Nav2 and RViz actually see.

ROBOT MUST BE STATIONARY. Commands no motion.
"""
import argparse
import math
import statistics
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu

R2DPM = 180.0 / math.pi * 60.0


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Drift(Node):
    def __init__(self):
        super().__init__('odom_drift_test')
        self.odom = []
        self.icm = []
        self.vel = []
        self.create_subscription(Odometry, '/odom', self._o, 20)
        self.create_subscription(Imu, '/imu/data_raw', self._i, 200)
        self.create_subscription(Twist, '/vel_raw', self._v, 50)

    def _o(self, m): self.odom.append((time.time(), yaw_of(m.pose.pose.orientation)))
    def _i(self, m): self.icm.append(m.angular_velocity.z)
    def _v(self, m): self.vel.append(m.angular.z)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=60.0)
    a = ap.parse_args()
    rclpy.init()
    n = Drift()
    print(f'measuring {a.seconds:.0f}s. KEEP THE ROBOT STILL.\n', flush=True)
    end = time.time() + a.seconds
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.05)

    if len(n.odom) < 10:
        raise SystemExit('no /odom data')
    t0, y0 = n.odom[0]
    t1, y1 = n.odom[-1]
    d = y1 - y0
    while d > math.pi:  d -= 2 * math.pi
    while d < -math.pi: d += 2 * math.pi
    span = t1 - t0
    drift = math.degrees(d) / (span / 60.0)

    print(f'  span {span:.1f}s, {len(n.odom)} odom samples\n')
    print(f'  {"source":<34}{"deg/min":>10}')
    print('  ' + '-' * 44)
    print(f'  {"/odom yaw drift (what Nav2 sees)":<34}{drift:>10.2f}')
    if n.vel:
        print(f'  {"/vel_raw angular.z mean":<34}'
              f'{statistics.fmean(n.vel)*R2DPM:>10.2f}')
    if n.icm:
        print(f'  {"/imu/data_raw gyro z mean (ICM)":<34}'
              f'{statistics.fmean(n.icm)*R2DPM:>10.2f}')
    print(f'\n  reference: MPU9250-anchored /odom measured -6.84 deg/min')
    print(f'  over 30 min parked that was {-6.84*30:+.0f} deg;'
          f' this build gives {drift*30:+.0f} deg')
    n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
