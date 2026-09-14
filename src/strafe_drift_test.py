#!/usr/bin/env python3
"""Measure mecanum strafe yaw drift: degrees of unwanted rotation per metre
of lateral travel.

Nothing commands a turn, so ALL yaw is error. The headline number is deg/m,
not deg/s, because deg/m is the speed-independent quantity and it converts
straight into the radius of the circle the robot is really driving:

    radius = 57.3 / drift_deg_per_m   metres

Runs a sweep of speeds x both directions. The SIGN STRUCTURE across +vy/-vy
is the diagnosis:

  * drift MIRRORS (opposite sign for +vy vs -vy)
      -> multiplicative per-wheel gain / diameter asymmetry. Reversing vy
         reverses every wheel's sign, so a gain error reverses with it.
      -> fixable with a feedforward omega proportional to vy.

  * drift is the SAME SIGN both ways
      -> a sign-invariant disturbance: wheel toe/axle misalignment, a
         dragging roller set, an unloaded wheel, chassis tweak.
      -> mechanical. Software will only paper over it.

*** THE ROBOT DRIVES SIDEWAYS. Clear a lane on both sides. ***

Trap: /odom cannot see this. base_node derives yaw from encoder ticks through
the ideal inverse kinematics, and strafe error comes from roller slip, which
encoders do not observe. The ICM gyro is the ground truth here; /odom yaw is
printed only so you can watch it disagree.
"""
import argparse
import math
import sys
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


class Strafe(Node):
    def __init__(self):
        super().__init__('strafe_drift_test')
        self.gz = 0.0          # latest raw gyro z, rad/s
        self.icm = 0.0         # bias-corrected integrated yaw, rad
        self.bias = 0.0        # rad/s, re-measured before every run
        self.t_icm = None
        self.p = self.p0 = None
        self.y = self.y0 = None
        self.vpeak = 0.0
        self.create_subscription(Imu, '/imu/data_raw', self._i, 200)
        self.create_subscription(Odometry, '/odom', self._o, 20)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def _i(self, m):
        t = time.time()
        self.gz = m.angular_velocity.z
        if self.t_icm is not None:
            self.icm += (self.gz - self.bias) * (t - self.t_icm)
        self.t_icm = t

    def _o(self, m):
        p = m.pose.pose.position
        self.p = (p.x, p.y)
        self.y = yaw_of(m.pose.pose.orientation)
        self.vpeak = max(self.vpeak, abs(m.twist.twist.linear.y))

    def dist(self):
        if self.p0 is None or self.p is None:
            return 0.0
        return math.hypot(self.p[0] - self.p0[0], self.p[1] - self.p0[1])

    def spin(self, secs):
        t0 = time.time()
        while time.time() - t0 < secs:
            rclpy.spin_once(self, timeout_sec=0.02)

    def go(self, vy):
        # the Rosmaster /cmd_vel watchdog cuts motors after 500 ms, so this
        # has to be re-published continuously, not once.
        t = Twist()
        t.linear.y = float(vy)
        self.pub.publish(t)

    def halt(self):
        for _ in range(30):
            self.go(0.0)
            rclpy.spin_once(self, timeout_sec=0.02)

    def measure_bias(self, secs=3.0):
        """Average gyro z while stationary. Un-removed bias masquerades as
        drift: 0.5 deg/s of bias over a 10 s run is 5 deg of fiction."""
        self.bias = 0.0
        samples = []
        t0 = time.time()
        while time.time() - t0 < secs:
            rclpy.spin_once(self, timeout_sec=0.02)
            samples.append(self.gz)
        self.bias = sum(samples) / max(len(samples), 1)
        return self.bias, samples


def one_run(n, speed, sign, metres, timeout):
    n.halt()
    print(f'  measuring gyro bias (hold still)...', flush=True)
    bias, samples = n.measure_bias(3.0)
    spread = (max(samples) - min(samples)) * R2D if samples else 0.0
    print(f'    bias {bias*R2D:+.3f} deg/s   (peak-to-peak {spread:.3f})',
          flush=True)

    n.spin(0.3)
    n.p0, n.y0, n.icm, n.t_icm, n.vpeak = n.p, n.y, 0.0, None, 0.0

    vy = sign * speed
    label = 'LEFT (+vy)' if sign > 0 else 'RIGHT (-vy)'
    print(f'  strafing {label} at {speed:.2f} m/s for {metres:.1f} m',
          flush=True)
    start = time.time()
    mark = 0.0
    try:
        while n.dist() < metres and time.time() - start < timeout:
            n.go(vy)
            rclpy.spin_once(n, timeout_sec=0.02)
            if n.dist() - mark >= 0.5:
                mark = n.dist()
                print(f'    {mark:4.2f} m   ICM {n.icm*R2D:+6.2f} deg'
                      f'   /odom {(n.y-n.y0)*R2D:+6.2f} deg', flush=True)
    finally:
        n.halt()
    el = time.time() - start
    n.spin(0.5)

    d = n.dist()
    dy = n.y - n.y0
    while dy > math.pi:  dy -= 2 * math.pi
    while dy < -math.pi: dy += 2 * math.pi
    icm_deg = n.icm * R2D
    # nominal distance from the command is a cross-check on odom: if these
    # disagree badly, odom's lateral scale is off and deg/m is distorted.
    nominal = speed * el
    per_m = icm_deg / d if d > 0.05 else float('nan')

    print(f'    -> {el:.1f} s, /odom {d:.2f} m, commanded {nominal:.2f} m,'
          f' peak /odom vy {n.vpeak:.2f}')
    print(f'    -> ICM yaw {icm_deg:+.2f} deg   =  {per_m:+.2f} deg/m'
          f'   (/odom yaw {dy*R2D:+.2f} deg)', flush=True)
    return dict(speed=speed, sign=sign, dist=d, nominal=nominal, elapsed=el,
                icm_deg=icm_deg, odom_deg=dy * R2D, per_m=per_m, bias=bias)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--speeds', default='0.10,0.20,0.30',
                    help='comma-separated lateral speeds, m/s')
    ap.add_argument('--metres', type=float, default=2.0,
                    help='lateral distance per run')
    ap.add_argument('--repeats', type=int, default=1,
                    help='repeats of the full +/- sweep')
    ap.add_argument('--timeout', type=float, default=60.0)
    ap.add_argument('--surface', default='unspecified',
                    help='free text, recorded in the output')
    ap.add_argument('--json', default='', help='write raw results here')
    ap.add_argument('--yes-path-is-clear', action='store_true')
    a = ap.parse_args()
    if not a.yes_path_is_clear:
        raise SystemExit('refusing to drive: pass --yes-path-is-clear')

    speeds = [float(s) for s in a.speeds.split(',') if s.strip()]

    rclpy.init()
    n = Strafe()
    n.spin(3.0)
    if n.p is None:
        raise SystemExit('no /odom -- is the bringup running?')
    if n.t_icm is None:
        raise SystemExit('no /imu/data_raw -- the ICM is the ground truth here')

    print(f'\nstrafe drift sweep  surface={a.surface}  '
          f'{len(speeds)} speeds x 2 directions x {a.repeats}\n')
    results = []
    try:
        for rep in range(a.repeats):
            for speed in speeds:
                for sign in (+1, -1):
                    print(f'\n--- rep {rep+1}  {speed:.2f} m/s  '
                          f'{"left" if sign > 0 else "right"} ---')
                    input('  reposition the robot, then press Enter '
                          '(Ctrl-C to stop): ')
                    results.append(one_run(n, speed, sign, a.metres,
                                           a.timeout))
    except KeyboardInterrupt:
        print('\ninterrupted', flush=True)
    finally:
        n.halt()

    if not results:
        n.destroy_node(); rclpy.shutdown(); return

    print(f'\n\n{"="*62}\n  SUMMARY  surface={a.surface}\n{"="*62}')
    print(f'  {"speed":>6} {"dir":>6} {"dist_m":>8} {"yaw_deg":>9} '
          f'{"deg/m":>9} {"circle_r_m":>11}')
    for r in results:
        rad = abs(57.3 / r['per_m']) if r['per_m'] and abs(r['per_m']) > 1e-6 \
            else float('inf')
        print(f'  {r["speed"]:>6.2f} {"left" if r["sign"] > 0 else "right":>6}'
              f' {r["dist"]:>8.2f} {r["icm_deg"]:>9.2f} {r["per_m"]:>9.2f}'
              f' {rad:>11.1f}')

    left = [r['per_m'] for r in results if r['sign'] > 0
            and r['per_m'] == r['per_m']]
    right = [r['per_m'] for r in results if r['sign'] < 0
             and r['per_m'] == r['per_m']]
    if left and right:
        ml = sum(left) / len(left)
        mr = sum(right) / len(right)
        print(f'\n  mean left  {ml:+.2f} deg/m')
        print(f'  mean right {mr:+.2f} deg/m')
        # mirrored -> ml and mr have opposite signs; the common-mode part is
        # the sign-invariant disturbance, the differential part is gain error.
        common = (ml + mr) / 2.0
        differ = (ml - mr) / 2.0
        print(f'\n  common-mode  {common:+.2f} deg/m  '
              f'(same direction both ways -> MECHANICAL: toe/axle '
              f'misalignment, dragging roller, unloaded wheel)')
        print(f'  differential {differ:+.2f} deg/m  '
              f'(mirrors with vy -> per-wheel GAIN asymmetry, '
              f'fixable in software)')
        if abs(common) > abs(differ) * 1.5:
            verdict = ('MECHANICAL dominates. Do the paper-drag load check '
                       'and verify the X roller pattern before writing code.')
        elif abs(differ) > abs(common) * 1.5:
            verdict = ('GAIN asymmetry dominates. A feedforward '
                       'omega = -k*vy will null most of it; '
                       f'k ~ {differ/57.3:+.4f} rad per m of lateral travel.')
        else:
            verdict = ('mixed -- both terms are comparable. Fix the '
                       'mechanical side first, then re-run.')
        print(f'\n  VERDICT: {verdict}')

        # does drift scale with speed? constant -> static friction.
        by_speed = {}
        for r in results:
            if r['per_m'] == r['per_m']:
                by_speed.setdefault(r['speed'], []).append(abs(r['per_m']))
        if len(by_speed) > 1:
            print(f'\n  |deg/m| vs speed:')
            for s in sorted(by_speed):
                v = by_speed[s]
                print(f'    {s:.2f} m/s -> {sum(v)/len(v):.2f}')
            print('    roughly flat -> static friction/misalignment '
                  '(needs an integral term)')
            print('    rising with speed -> dynamic gain error '
                  '(feedforward proportional to vy works)')

    print('\n  NOW MEASURE PHYSICALLY, for at least one run:')
    print('    1. final heading vs your tape line (protractor or a laser)')
    print('    2. actual lateral distance travelled vs the /odom column above')
    print('       -> mecanum roller slip makes /odom over-read lateral travel,')
    print('          which flatters deg/m. If they differ, rescale by hand.')

    if a.json:
        import json as _j
        with open(a.json, 'w') as f:
            _j.dump(dict(surface=a.surface, metres=a.metres,
                         results=results), f, indent=2)
        print(f'\n  wrote {a.json}')

    n.halt()
    n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
