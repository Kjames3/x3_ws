#!/usr/bin/env python3
"""Test 3 (passive): heading drift while the OPERATOR drives.

Publishes nothing -- it only listens. The robot is driven by hand along a
measured straight line, which avoids the /cmd_vel scaling problem (commanding
0.15 m/s produced 0.80 m/s, so a closed-loop run overshoots its target).

Auto-detects the driven segment inside the window, so it does not matter when
you start. Any heading change during a straight run is error: you commanded no
rotation.
"""
import argparse
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu

R2D = 180.0 / math.pi
MOVING = 0.03          # m/s; above this the robot counts as driving


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Passive(Node):
    def __init__(self):
        super().__init__('imu_straight_passive')
        self.odom = []      # (t, x, y, yaw, v)
        self.gyro = []      # (t, wz)
        self.create_subscription(Odometry, '/odom', self._o, 50)
        self.create_subscription(Imu, '/imu/data_raw', self._i, 200)

    def _o(self, m):
        p = m.pose.pose.position
        self.odom.append((time.time(), p.x, p.y,
                          yaw_of(m.pose.pose.orientation),
                          m.twist.twist.linear.x))

    def _i(self, m):
        self.gyro.append((time.time(), m.angular_velocity.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--window', type=float, default=120.0)
    a = ap.parse_args()
    rclpy.init()
    n = Passive()
    print(f'LISTENING for {a.window:.0f}s -- drive the robot forward now.',
          flush=True)
    end = time.time() + a.window
    last = 1e9
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.02)
        rem = end - time.time()
        if last - rem > 10.0:
            last = rem
            v = n.odom[-1][4] if n.odom else 0.0
            sys.stdout.write(f'\r  {rem:4.0f}s left   speed {v:+5.2f} m/s   '
                             f'{len(n.odom)} odom samples  ')
            sys.stdout.flush()
    print('\r' + ' ' * 60 + '\r  window closed.\n', flush=True)

    if len(n.odom) < 20:
        raise SystemExit('no /odom data')
    mv = [k for k, r in enumerate(n.odom) if abs(r[4]) > MOVING]
    if not mv:
        peak = max(abs(r[4]) for r in n.odom)
        raise SystemExit(f'NO MOTION DETECTED (peak {peak:.3f} m/s). '
                         'Did the robot actually drive?')
    lo, hi = mv[0], mv[-1]
    seg = n.odom[lo:hi + 1]
    t0, t1 = seg[0][0], seg[-1][0]
    dx = seg[-1][1] - seg[0][1]
    dy = seg[-1][2] - seg[0][2]
    dist = math.hypot(dx, dy)
    dyaw = seg[-1][3] - seg[0][3]
    while dyaw > math.pi:  dyaw -= 2 * math.pi
    while dyaw < -math.pi: dyaw += 2 * math.pi

    g = [w for t, w in n.gyro if t0 <= t <= t1]
    gt = [t for t, w in n.gyro if t0 <= t <= t1]
    icm = 0.0
    for k in range(1, len(g)):
        icm += g[k] * (gt[k] - gt[k - 1])

    # lateral offset relative to the robot's OWN initial heading, which is the
    # line it was aimed along -- not the odom x axis
    h0 = seg[0][3]
    fwd = dx * math.cos(h0) + dy * math.sin(h0)
    lat = -dx * math.sin(h0) + dy * math.cos(h0)
    vpk = max(abs(r[4]) for r in seg)

    print(f'  driven segment: {t1-t0:.1f}s, peak speed {vpk:.2f} m/s\n')
    print(f'  {"quantity":<34}{"value":>12}')
    print('  ' + '-' * 48)
    print(f'  {"/odom path length":<34}{dist:>10.3f} m')
    print(f'  {"/odom forward (along start heading)":<34}{fwd:>10.3f} m')
    print(f'  {"/odom lateral (crab)":<34}{lat:>10.3f} m')
    print(f'  {"/odom heading change":<34}{dyaw*R2D:>10.2f} deg')
    print(f'  {"ICM integrated yaw":<34}{icm*R2D:>10.2f} deg')
    if dist > 0.05:
        print(f'  {"heading drift per metre":<34}{dyaw*R2D/dist:>10.2f} deg/m')
    print(f'\n  PASS if heading drift < 3 deg over the run')
    print(f'  -> {"PASS" if abs(dyaw*R2D) < 3.0 else "FAIL"} '
          f'({abs(dyaw*R2D):.2f} deg over {dist:.2f} m)')
    print(f'\n  COMPARE PHYSICALLY:')
    print(f'    actual distance  vs /odom {dist:.2f} m  -> linear scale')
    print(f'    actual sideways  vs /odom {lat:+.2f} m  -> real crab or artifact')
    print(f'    actual heading   vs /odom {dyaw*R2D:+.1f} deg')
    n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
