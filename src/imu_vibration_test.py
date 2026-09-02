#!/usr/bin/env python3
"""Test 1: does motor vibration alias into the ICM-42688-P gyro?

The ICM runs at 200 Hz with NO anti-alias/low-pass filter configured, whereas
the MPU9250 was heavily filtered before its 10 Hz serial hop. Vibration energy
above the 100 Hz Nyquist folds back down and is indistinguishable from real
rotation -- it would present as heading drift only while driving.

*** THE WHEELS MUST BE OFF THE GROUND. This spins the motors. ***

Phases: baseline (motors off), then each --speeds value, then recovery.
Reads /imu/data_raw so it exercises the deployed pipeline rather than the bus.
"""
import argparse
import math
import statistics
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu

R2DPM = 180.0 / math.pi * 60.0


class VibTest(Node):
    def __init__(self):
        super().__init__('imu_vibration_test')
        self.buf = []
        self.rec = False
        self.create_subscription(Imu, '/imu/data_raw', self._cb, 200)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def _cb(self, m):
        if self.rec:
            self.buf.append((m.angular_velocity.x, m.angular_velocity.y,
                             m.angular_velocity.z, m.linear_acceleration.x,
                             m.linear_acceleration.y, m.linear_acceleration.z))

    def drive(self, vx):
        t = Twist(); t.linear.x = vx
        self.pub.publish(t)

    def phase(self, label, sec, vx):
        """Spin motors at vx (0 = off) and record. Motors are re-commanded at
        50 Hz because the driver's watchdog cuts them after 500 ms."""
        self.buf = []
        settle = time.time() + 2.0
        while time.time() < settle:            # let the speed stabilise first
            self.drive(vx)
            rclpy.spin_once(self, timeout_sec=0.02)
        self.rec = True
        end = time.time() + sec
        while time.time() < end:
            self.drive(vx)
            rclpy.spin_once(self, timeout_sec=0.02)
        self.rec = False
        self.drive(0.0)
        d = list(self.buf)
        print(f'  {label:<22} {len(d):>6} samples', flush=True)
        return d


def analyse(name, d, base=None):
    if len(d) < 50:
        print(f'  {name}: too few samples'); return None
    gz = [r[2] for r in d]
    gsd = statistics.pstdev(gz)
    gmean = statistics.fmean(gz)
    asd = statistics.pstdev([math.sqrt(r[3]**2 + r[4]**2 + r[5]**2) for r in d])
    row = (f'  {name:<14} {gsd:>10.6f} {gmean*R2DPM:>+12.2f} {asd:>10.4f}')
    if base:
        row += f' {gsd/base[0]:>8.2f}x {abs(gmean*R2DPM - base[1]*R2DPM):>+10.2f}'
    print(row)
    return (gsd, gmean, asd)


def spectrum(d, label):
    """Energy above 100 Hz cannot be represented at 200 Hz and folds back."""
    try:
        import numpy as np
    except ImportError:
        return
    gz = np.array([r[2] for r in d], float)
    gz -= gz.mean()
    n = len(gz)
    if n < 256:
        return
    f = np.fft.rfftfreq(n, 1.0 / 200.0)
    p = np.abs(np.fft.rfft(gz * np.hanning(n))) ** 2
    tot = p.sum()
    if tot <= 0:
        return
    hi = p[f > 60].sum() / tot
    pk = f[int(np.argmax(p[1:])) + 1]
    print(f'    {label}: peak {pk:5.1f} Hz, {100*hi:5.1f}% of energy above 60 Hz')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=12.0)
    ap.add_argument('--speeds', type=float, nargs='*', default=[0.1, 0.2, 0.3])
    ap.add_argument('--yes-wheels-are-off-the-ground', action='store_true')
    a = ap.parse_args()
    if not a.yes_wheels_are_off_the_ground:
        raise SystemExit('refusing to spin motors: pass '
                         '--yes-wheels-are-off-the-ground')

    rclpy.init()
    n = VibTest()
    print('waiting for /imu/data_raw...', flush=True)
    t0 = time.time()
    n.rec = True
    while not n.buf and time.time() - t0 < 10:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.rec = False
    if not n.buf:
        raise SystemExit('no /imu/data_raw -- is the bringup running?')

    print('\nrecording phases (motors WILL spin):', flush=True)
    runs = [('baseline OFF', n.phase('baseline OFF', a.seconds, 0.0))]
    for v in a.speeds:
        runs.append((f'motors {v:.2f} m/s', n.phase(f'motors {v:.2f} m/s', a.seconds, v)))
    runs.append(('recovery OFF', n.phase('recovery OFF', a.seconds, 0.0)))
    n.drive(0.0); n.drive(0.0)

    print(f'\n  {"phase":<14} {"gyro_z sd":>10} {"gyro_z bias":>12} {"|a| sd":>10}'
          f' {"sd rel":>8} {"bias shift":>10}')
    print('  ' + '-' * 70)
    base = analyse(runs[0][0], runs[0][1])
    res = []
    for name, d in runs[1:]:
        res.append((name, analyse(name, d, base)))
    print('\n  spectral content (aliasing risk):')
    for name, d in runs:
        spectrum(d, name)

    print('\n  VERDICT')
    worst_sd = max((r[1][0] / base[0]) for r in res if r[1])
    worst_bias = max(abs(r[1][1] * R2DPM - base[1] * R2DPM) for r in res if r[1])
    print(f'    worst noise growth   : {worst_sd:.2f}x   (pass < 3x)')
    print(f'    worst yaw bias shift : {worst_bias:.2f} deg/min   (pass < 10)')
    ok = worst_sd < 3.0 and worst_bias < 10.0
    print(f'    -> {"PASS" if ok else "FAIL -- configure the ICM UI filter"}')

    n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
