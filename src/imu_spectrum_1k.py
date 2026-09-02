#!/usr/bin/env python3
"""Test 1b: capture the TRUE vibration spectrum at 1 kHz ODR.

At the deployed 200 Hz, Nyquist is 100 Hz and anything above it folds back
invisibly -- which is why the measured peak fell from 95 Hz to 30 Hz as wheel
speed ROSE. This raises the ICM's ODR to 1 kHz (Nyquist 500 Hz) and samples the
device directly, so the real spectrum is visible and an anti-alias bandwidth can
be chosen from data instead of guessed.

*** WHEELS MUST BE OFF THE GROUND. This spins the motors. ***

The ODR change is global to the chip, so the running icm42688_node briefly sees
1 kHz data while still polling at 200 Hz (harmless). Original config is restored
in a finally block.
"""
import argparse
import math
import statistics
import threading
import time

import rclpy
import smbus2
from geometry_msgs.msg import Twist
from rclpy.node import Node

ADDR, BUS = 0x68, 7
PWR_MGMT0, GYRO_CONFIG0, ACCEL_CONFIG0 = 0x4E, 0x4F, 0x50
ACCEL_DATA_X1 = 0x1F
ODR_1KHZ, ODR_200HZ = 0x06, 0x07
GYRO_LSB_PER_DPS = 16.4
DEG2RAD = math.pi / 180.0
R2DPM = 180.0 / math.pi * 60.0


class Driver(Node):
    """Re-commands /cmd_vel at 50 Hz; the driver watchdog cuts motors at 500 ms."""

    def __init__(self):
        super().__init__('imu_spectrum_driver')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.vx = 0.0
        self.stop = False
        self.t = threading.Thread(target=self._loop, daemon=True)
        self.t.start()

    def _loop(self):
        while not self.stop:
            m = Twist(); m.linear.x = self.vx
            self.pub.publish(m)
            time.sleep(0.02)


def sample(bus, sec):
    out, t0 = [], time.time()
    end = t0 + sec
    while time.time() < end:
        try:
            d = bus.read_i2c_block_data(ADDR, ACCEL_DATA_X1, 12)
        except OSError:
            continue
        v = (d[10] << 8) | d[11]
        out.append((v - 65536 if v & 0x8000 else v) / GYRO_LSB_PER_DPS * DEG2RAD)
    return out, len(out) / (time.time() - t0)


def spectrum(gz, fs, label):
    import numpy as np
    a = np.array(gz, float); a -= a.mean()
    n = len(a)
    f = np.fft.rfftfreq(n, 1.0 / fs)
    p = np.abs(np.fft.rfft(a * np.hanning(n))) ** 2
    tot = p.sum()
    if tot <= 0:
        return
    order = np.argsort(p[1:])[::-1][:3] + 1
    peaks = ', '.join(f'{f[i]:.0f}Hz' for i in order)
    above100 = 100.0 * p[f > 100].sum() / tot
    above200 = 100.0 * p[f > 200].sum() / tot
    print(f'  {label:<16} sd {statistics.pstdev(gz):.6f}  peaks {peaks:<24}'
          f' >100Hz {above100:5.1f}%  >200Hz {above200:5.1f}%')
    return above100


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=10.0)
    ap.add_argument('--speeds', type=float, nargs='*', default=[0.1, 0.2, 0.3])
    ap.add_argument('--yes-wheels-are-off-the-ground', action='store_true')
    a = ap.parse_args()
    if not a.yes_wheels_are_off_the_ground:
        raise SystemExit('refusing to spin motors')

    bus = smbus2.SMBus(BUS)
    g0 = bus.read_byte_data(ADDR, GYRO_CONFIG0)
    a0 = bus.read_byte_data(ADDR, ACCEL_CONFIG0)
    rclpy.init()
    drv = Driver()
    try:
        bus.write_byte_data(ADDR, GYRO_CONFIG0, ODR_1KHZ)
        bus.write_byte_data(ADDR, ACCEL_CONFIG0, ODR_1KHZ)
        time.sleep(0.2)
        print(f'ODR raised to 1 kHz (was 0x{g0:02X}). Sampling directly.\n')
        runs = []
        for label, vx in [('baseline OFF', 0.0)] + [(f'{v:.2f} m/s', v) for v in a.speeds]:
            drv.vx = vx
            time.sleep(2.0)
            gz, fs = sample(bus, a.seconds)
            runs.append((label, gz, fs))
            print(f'  {label:<16} {len(gz)} samples at {fs:.0f} Hz'
                  f'  (Nyquist {fs/2:.0f} Hz)', flush=True)
            drv.vx = 0.0
            time.sleep(1.0)
        print('\nTRUE SPECTRUM (energy above 100 Hz was invisible at 200 Hz ODR):')
        hi = []
        for label, gz, fs in runs:
            r = spectrum(gz, fs, label)
            if r is not None:
                hi.append((label, r))
        print('\n  VERDICT')
        worst = max(h[1] for h in hi[1:]) if len(hi) > 1 else 0.0
        base = hi[0][1] if hi else 0.0
        print(f'    baseline energy >100 Hz : {base:.1f}%')
        print(f'    worst driving  >100 Hz  : {worst:.1f}%')
        if worst > 15.0:
            print('    -> ALIASING CONFIRMED: significant energy above the 100 Hz')
            print('       Nyquist of the deployed 200 Hz rate. Configure the AAF.')
        else:
            print('    -> little energy above 100 Hz; the 200 Hz noise growth is')
            print('       real in-band vibration, not aliasing.')
    finally:
        drv.vx = 0.0
        drv.stop = True
        time.sleep(0.3)
        bus.write_byte_data(ADDR, GYRO_CONFIG0, g0)
        bus.write_byte_data(ADDR, ACCEL_CONFIG0, a0)
        bus.close()
        print(f'\nrestored ODR (gyro 0x{g0:02X}, accel 0x{a0:02X}); motors stopped')
        drv.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
