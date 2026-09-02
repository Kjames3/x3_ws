#!/usr/bin/env python3
"""Test 2: in-place rotation -- yaw sign and SCALE under real motor drive.

Rotates left (CCW) until the ICM-42688-P's integrated yaw reaches --degrees,
then stops, and reports what every other yaw source said at that moment:

  ICM   /imu/data_raw  angular_velocity.z   (the new sensor, closed-loop target)
  MPU   /vel_raw       angular.z            (MPU9250 gyro -- still feeds odom)
  EKF   /odom          quaternion yaw       (the fused estimate Nav2 uses)
  RAW   /odom_raw      quaternion yaw       (dead reckoning, MPU-derived)

The operator then measures the PHYSICAL angle. Disagreement between ICM and the
physical mark is a scale error; disagreement between ICM and MPU tells you which
one is wrong. Hand-rotation cannot find scale errors that only appear under the
vibration and current draw of real driving.

*** THE ROBOT WILL SPIN IN PLACE ON THE FLOOR. Clear space required. ***
"""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu

R2D = 180.0 / math.pi


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RotTest(Node):
    def __init__(self):
        super().__init__('imu_rotation_test')
        self.icm = 0.0          # integrated rad
        self.mpu = 0.0
        self.t_icm = None
        self.t_mpu = None
        self.odom_yaw = None
        self.odom0 = None
        self.raw_yaw = None
        self.raw0 = None
        self.peak = 0.0
        self.create_subscription(Imu, '/imu/data_raw', self._imu, 200)
        self.create_subscription(Twist, '/vel_raw', self._vel, 50)
        self.create_subscription(Odometry, '/odom', self._odom, 20)
        self.create_subscription(Odometry, '/odom_raw', self._raw, 20)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def _imu(self, m):
        t = time.time()
        if self.t_icm is not None:
            self.icm += m.angular_velocity.z * (t - self.t_icm)
            self.peak = max(self.peak, abs(m.angular_velocity.z))
        self.t_icm = t

    def _vel(self, m):
        t = time.time()
        if self.t_mpu is not None:
            self.mpu += m.angular.z * (t - self.t_mpu)
        self.t_mpu = t

    def _odom(self, m):
        y = yaw_of(m.pose.pose.orientation)
        if self.odom0 is None:
            self.odom0 = y
        self.odom_yaw = y

    def _raw(self, m):
        y = yaw_of(m.pose.pose.orientation)
        if self.raw0 is None:
            self.raw0 = y
        self.raw_yaw = y

    def spin_cmd(self, wz):
        t = Twist(); t.angular.z = wz
        self.pub.publish(t)

    def halt(self):
        for _ in range(30):
            self.spin_cmd(0.0)
            rclpy.spin_once(self, timeout_sec=0.02)


def unwrap(d):
    while d > math.pi:  d -= 2 * math.pi
    while d < -math.pi: d += 2 * math.pi
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--degrees', type=float, default=360.0)
    ap.add_argument('--rate', type=float, default=0.5, help='rad/s, CCW')
    ap.add_argument('--timeout', type=float, default=60.0)
    ap.add_argument('--yes-floor-is-clear', action='store_true')
    a = ap.parse_args()
    if not a.yes_floor_is_clear:
        raise SystemExit('refusing to spin: pass --yes-floor-is-clear')

    rclpy.init()
    n = RotTest()
    print('settling (collecting a reference before motion)...', flush=True)
    t0 = time.time()
    while time.time() - t0 < 3.0:
        rclpy.spin_once(n, timeout_sec=0.05)
    # zero the integrators AFTER the reference settle
    n.icm = n.mpu = 0.0
    n.odom0 = n.odom_yaw
    n.raw0 = n.raw_yaw
    if n.odom_yaw is None:
        print('WARNING: no /odom -- EKF comparison unavailable')

    target = math.radians(a.degrees)
    print(f'rotating CCW at {a.rate} rad/s until ICM reads {a.degrees:.0f} deg'
          f' (timeout {a.timeout:.0f}s)\n', flush=True)
    start = time.time()
    last = 0.0
    try:
        while n.icm < target and time.time() - start < a.timeout:
            n.spin_cmd(a.rate)
            rclpy.spin_once(n, timeout_sec=0.02)
            if n.icm * R2D - last >= 45.0:
                last = n.icm * R2D
                print(f'  ICM {last:6.1f} deg   MPU {n.mpu*R2D:6.1f} deg', flush=True)
    finally:
        n.halt()
    el = time.time() - start
    for _ in range(20):
        rclpy.spin_once(n, timeout_sec=0.05)

    ekf = unwrap(n.odom_yaw - n.odom0) * R2D if n.odom_yaw is not None and n.odom0 is not None else float('nan')
    raw = unwrap(n.raw_yaw - n.raw0) * R2D if n.raw_yaw is not None and n.raw0 is not None else float('nan')
    # a full turn wraps; report the wrapped value plus the expected full turns
    print(f'\n  elapsed {el:.1f}s, peak rate {n.peak*R2D:.1f} deg/s\n')
    print(f'  {"source":<28}{"integrated deg":>16}')
    print('  ' + '-' * 44)
    print(f'  {"ICM-42688-P (/imu/data_raw)":<28}{n.icm*R2D:>16.1f}   <- closed-loop target')
    print(f'  {"MPU9250 (/vel_raw)":<28}{n.mpu*R2D:>16.1f}')
    print(f'  {"EKF (/odom, wrapped)":<28}{ekf:>16.1f}')
    print(f'  {"dead reckon (/odom_raw)":<28}{raw:>16.1f}')
    d = n.mpu * R2D - n.icm * R2D
    print(f'\n  MPU - ICM disagreement: {d:+.1f} deg '
          f'({100*abs(d)/max(abs(n.icm*R2D),1e-6):.1f}% of the turn)')
    print(f'\n  NOW MEASURE THE PHYSICAL ANGLE against your floor mark.')
    print(f'  ICM claims the robot turned {n.icm*R2D:.1f} deg.')
    print(f'    physical == ICM  -> scale correct, ICM is trustworthy')
    print(f'    physical <  ICM  -> ICM over-reads (gyro scale high)')
    print(f'    physical >  ICM  -> ICM under-reads')
    n.halt()
    n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
