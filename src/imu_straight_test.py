#!/usr/bin/env python3
"""Test 3: drive straight -- heading drift with NO commanded rotation.

This is where residual vibration shows up UNDER LOAD. The bench test could not
tell us, because free-spinning wheels on a stand are undamped and unloaded and
vibrate quite differently from wheels carrying the robot on the floor.

Drives forward until /odom reports --metres, then stops, and reports how much
heading each source thinks was gained. Any non-zero yaw is error: nothing
commanded a turn.

*** THE ROBOT DRIVES FORWARD. Clear the path. ***
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


class Straight(Node):
    def __init__(self):
        super().__init__('imu_straight_test')
        self.icm = 0.0
        self.t_icm = None
        self.p0 = None
        self.p = None
        self.y0 = None
        self.y = None
        self.vpeak = 0.0
        self.create_subscription(Imu, '/imu/data_raw', self._i, 200)
        self.create_subscription(Odometry, '/odom', self._o, 20)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def _i(self, m):
        t = time.time()
        if self.t_icm is not None:
            self.icm += m.angular_velocity.z * (t - self.t_icm)
        self.t_icm = t

    def _o(self, m):
        p = m.pose.pose.position
        self.p = (p.x, p.y)
        self.y = yaw_of(m.pose.pose.orientation)
        self.vpeak = max(self.vpeak, abs(m.twist.twist.linear.x))

    def dist(self):
        if self.p0 is None or self.p is None:
            return 0.0
        return math.hypot(self.p[0] - self.p0[0], self.p[1] - self.p0[1])

    def go(self, vx):
        t = Twist(); t.linear.x = vx
        self.pub.publish(t)

    def halt(self):
        for _ in range(30):
            self.go(0.0)
            rclpy.spin_once(self, timeout_sec=0.02)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--metres', type=float, default=4.0)
    ap.add_argument('--speed', type=float, default=0.2)
    ap.add_argument('--timeout', type=float, default=60.0)
    ap.add_argument('--yes-path-is-clear', action='store_true')
    a = ap.parse_args()
    if not a.yes_path_is_clear:
        raise SystemExit('refusing to drive: pass --yes-path-is-clear')

    rclpy.init()
    n = Straight()
    t0 = time.time()
    while time.time() - t0 < 3.0:
        rclpy.spin_once(n, timeout_sec=0.05)
    if n.p is None:
        raise SystemExit('no /odom')
    n.p0, n.y0, n.icm = n.p, n.y, 0.0

    print(f'driving forward at {a.speed} m/s until /odom reads '
          f'{a.metres:.1f} m\n', flush=True)
    start = time.time()
    mark = 0.0
    try:
        while n.dist() < a.metres and time.time() - start < a.timeout:
            n.go(a.speed)
            rclpy.spin_once(n, timeout_sec=0.02)
            if n.dist() - mark >= 0.5:
                mark = n.dist()
                print(f'  {mark:4.2f} m   heading {(n.y-n.y0)*R2D:+6.2f} deg'
                      f'   ICM {n.icm*R2D:+6.2f} deg', flush=True)
    finally:
        n.halt()
    el = time.time() - start
    for _ in range(20):
        rclpy.spin_once(n, timeout_sec=0.05)

    d = n.dist()
    dy = n.y - n.y0
    while dy > math.pi:  dy -= 2 * math.pi
    while dy < -math.pi: dy += 2 * math.pi
    lat = n.p[1] - n.p0[1]
    print(f'\n  elapsed {el:.1f}s, peak /odom speed {n.vpeak:.2f} m/s'
          f' (commanded {a.speed})')
    print(f'\n  {"quantity":<32}{"value":>12}')
    print('  ' + '-' * 46)
    print(f'  {"/odom distance":<32}{d:>10.3f} m')
    print(f'  {"/odom heading change":<32}{dy*R2D:>10.2f} deg   <- want ~0')
    print(f'  {"ICM integrated yaw":<32}{n.icm*R2D:>10.2f} deg   <- want ~0')
    print(f'  {"/odom lateral offset":<32}{lat:>10.3f} m')
    print(f'\n  PASS if heading drift < 3 deg over {a.metres:.0f} m')
    ok = abs(dy * R2D) < 3.0
    print(f'  -> {"PASS" if ok else "FAIL"} ({abs(dy*R2D):.2f} deg)')
    print(f'\n  NOW MEASURE PHYSICALLY:')
    print(f'    1. actual distance travelled (/odom claims {d:.2f} m)')
    print(f'       -> if these differ, linear odometry scale is off')
    print(f'    2. sideways offset from your line at the end')
    print(f'    3. final heading vs the line (/odom claims {dy*R2D:+.1f} deg)')
    n.halt()
    n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
