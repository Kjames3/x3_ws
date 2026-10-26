#!/usr/bin/env python3
"""Step test of the driver's closed-loop wheel speed (Mcnamu_driver_X3 speed_control).

Turns speed_control on, publishes a sequence of constant /cmd_vel steps and
compares what the wheels, /odom and the gyro did with what was asked. Turns it
back off afterwards unless --keep-on.

*** THE ROBOT DRIVES ITSELF and this bypasses the CBF. Clear the floor:
    quick needs 0.5 m ahead and behind, full needs 1 m all round. ***

    python3 src/tools/hw_tests/base_speed_test.py --yes-path-is-clear                 # quick
    python3 src/tools/hw_tests/base_speed_test.py --yes-path-is-clear --profile full

Aborts (zero command, speed_control off) if a wheel runs away: faster than
twice its target plus 0.25 m/s. server_x3.py yields /cmd_vel while this runs.
"""

import sys as _sys
from pathlib import Path as _Path
_SRC_DIR = str(_Path(__file__).resolve().parents[2])  # runtime modules (drivers_x3, velocity_estimator, ...) live in src/
if _SRC_DIR not in _sys.path:
    _sys.path.insert(0, _SRC_DIR)

import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState

L = 0.215     # the driver's yaw_wheel_separation
# (label, vx, vy, omega, seconds)
QUICK = [('fwd 0.10', 0.10, 0, 0, 3.0), ('back 0.10', -0.10, 0, 0, 3.0)]
TURN = [('turn L 0.6', 0, 0, 0.6, 3.0), ('turn R 0.6', 0, 0, -0.6, 3.0),
        ('turn L 1.2', 0, 0, 1.2, 2.0), ('turn R 1.2', 0, 0, -1.2, 2.0)]
FULL = QUICK + [
    ('fwd 0.05', 0.05, 0, 0, 4.0), ('back 0.05', -0.05, 0, 0, 4.0),
    ('fwd 0.20', 0.20, 0, 0, 2.5), ('back 0.20', -0.20, 0, 0, 2.5),
    ('fwd 0.40', 0.40, 0, 0, 2.0), ('back 0.40', -0.40, 0, 0, 2.0),
    ('left 0.10', 0, 0.10, 0, 3.0), ('right 0.10', 0, -0.10, 0, 3.0),
    ('left 0.20', 0, 0.20, 0, 2.0), ('right 0.20', 0, -0.20, 0, 2.0),
    ('turn L 0.6', 0, 0, 0.6, 3.0), ('turn R 0.6', 0, 0, -0.6, 3.0),
    ('turn L 1.2', 0, 0, 1.2, 2.0), ('turn R 1.2', 0, 0, -1.2, 2.0),
]


def targets(vx, vy, wz):
    r = wz * L
    return [vx - vy - r, vx + vy - r, vx + vy + r, vx - vy + r]


class Test(Node):
    def __init__(self):
        super().__init__('base_speed_test')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.enc, self.odom, self.gyro = [], [], []
        self.create_subscription(JointState, '/wheel_encoders', self.on_enc, 50)
        self.create_subscription(Odometry, '/odom', lambda m: self.odom.append(
            (time.monotonic(), m.twist.twist.linear.x, m.twist.twist.linear.y)), 50)
        self.create_subscription(Imu, '/imu/data', lambda m: self.gyro.append(
            (time.monotonic(), m.angular_velocity.z)), 200)
        self.cli = self.create_client(SetParameters, '/driver_node/set_parameters')

    def on_enc(self, m):
        if len(m.velocity) == 4:
            self.enc.append((time.monotonic(), list(m.velocity), list(m.effort)))

    def speed_control(self, on):
        if not self.cli.wait_for_service(timeout_sec=5.0):
            raise SystemExit('driver_node parameter service not found')
        req = SetParameters.Request(parameters=[Parameter(
            name='speed_control', value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=on))])
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if fut.result() is None or not fut.result().results[0].successful:
            raise SystemExit('could not set speed_control')

    def command(self, vx, vy, wz, seconds, guard=True):
        """Publish at 20 Hz for `seconds`. Returns False if a wheel ran away."""
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
        want = targets(vx, vy, wz)
        end = time.monotonic() + seconds
        nxt = 0.0
        while time.monotonic() < end:
            now = time.monotonic()
            if now >= nxt:
                self.pub.publish(t)
                nxt = now + 0.05
            rclpy.spin_once(self, timeout_sec=0.01)
            if guard and self.enc:
                for w, tgt in zip(self.enc[-1][1], want):
                    if abs(w) > 2.0 * abs(tgt) + 0.25:
                        return False
        return True


def mean(rows, a, b, col):
    v = [r[col] for r in rows if a <= r[0] <= b]
    return sum(v) / len(v) if v else float('nan')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--yes-path-is-clear', action='store_true')
    ap.add_argument('--profile', choices=('quick', 'turn', 'full'), default='quick')
    ap.add_argument('--keep-on', action='store_true', help='leave speed_control on afterwards')
    ap.add_argument('--json')
    a = ap.parse_args()
    if not a.yes_path_is_clear:
        raise SystemExit('refusing to drive: pass --yes-path-is-clear')
    rclpy.init()
    n = Test()
    steps = {'quick': QUICK, 'turn': TURN, 'full': FULL}[a.profile]
    results, aborted = [], None
    try:
        n.command(0, 0, 0, 1.5, guard=False)       # let the server notice and yield
        n.speed_control(True)
        n.command(0, 0, 0, 0.5, guard=False)
        for label, vx, vy, wz, secs in steps:
            t0 = time.monotonic()
            ok = n.command(vx, vy, wz, secs)
            t1 = time.monotonic()
            n.command(0, 0, 0, 1.5, guard=False)
            a_, b_ = t0 + 0.5 * secs, t1
            want = targets(vx, vy, wz)
            got = [sum(e[1][i] for e in n.enc if a_ <= e[0] <= b_) / max(1, sum(1 for e in n.enc if a_ <= e[0] <= b_)) for i in range(4)]
            peak = max((abs(w) for e in n.enc if t0 <= e[0] <= t1 for w in e[1]), default=0.0)
            pwm = [e[2][:4] for e in n.enc if a_ <= e[0] <= b_ and len(e[2]) >= 4]
            results.append({'step': label, 'cmd': [vx, vy, wz], 'wheel_target': want, 'wheel_speed': got,
                            'wheel_peak': peak, 'odom_vx': mean(n.odom, a_, b_, 1), 'odom_vy': mean(n.odom, a_, b_, 2),
                            'gyro_wz': mean(n.gyro, a_, b_, 1),
                            'pwm': [sum(p[i] for p in pwm) / len(pwm) for i in range(4)] if pwm else None,
                            'trace': [(round(e[0] - t0, 3), [round(w, 3) for w in e[1]], e[2][:4]) for e in n.enc if t0 - 0.1 <= e[0] <= t1 + 1.2]})
            if not ok:
                aborted = label
                break
    finally:
        n.command(0, 0, 0, 0.6, guard=False)
        if aborted or not a.keep_on:
            n.speed_control(False)
    print('%-11s %-22s | %-27s | %-27s | %5s | %7s %7s %7s | %s' % (
        'step', 'cmd vx, vy, wz', 'wheel target FL RL FR RR', 'wheel measured', 'peak', 'odom vx', 'odom vy', 'gyro wz', 'PWM'))
    for r in results:
        print('%-11s (%5.2f, %5.2f, %5.2f)   | %s | %s | %5.2f | %7.3f %7.3f %7.3f | %s' % (
            r['step'], *r['cmd'], ' '.join('%6.3f' % v for v in r['wheel_target']),
            ' '.join('%6.3f' % v for v in r['wheel_speed']), r['wheel_peak'], r['odom_vx'], r['odom_vy'], r['gyro_wz'],
            ' '.join('%4.0f' % p for p in r['pwm']) if r['pwm'] else '-'))
    if aborted:
        print('ABORTED at "%s": a wheel ran away. speed_control is OFF.' % aborted)
    else:
        print('speed_control left %s' % ('ON' if a.keep_on else 'OFF'))
    if a.json:
        json.dump(results, open(a.json, 'w'))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
