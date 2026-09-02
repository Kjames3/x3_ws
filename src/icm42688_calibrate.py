#!/usr/bin/env python3
"""Bias calibration and axis identification for the ICM-42688-P on i2c-7 @ 0x68.

  --bias [SEC]            robot STATIONARY: gyro/accel bias, noise, scale, rate
  --motion SEC --label L  record a deliberate motion and report which axis moved

The mount orientation is determined empirically here, not assumed. At rest the
accelerometer reads +1 g along whichever axis points UP; tilting the nose up
moves gravity onto the fore/aft axis; yawing left excites the up axis in gyro.
"""
import argparse, json, math, os, time
import smbus2

ADDR, BUS = 0x68, 7
PWR_MGMT0, GYRO_CONFIG0, ACCEL_CONFIG0 = 0x4E, 0x4F, 0x50
ACCEL_DATA_X1, WHO_AM_I = 0x1F, 0x75
GYRO_LSB_PER_DPS, ACCEL_LSB_PER_G = 16.4, 2048.0
DEG2RAD, G = math.pi / 180.0, 9.80665
AX = ('x', 'y', 'z')
CAL = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   '..', 'config', 'icm42688_calibration.json')


class ICM:
    def __init__(self):
        self.b = smbus2.SMBus(BUS)
        if self.b.read_byte_data(ADDR, WHO_AM_I) != 0x47:
            raise SystemExit('WHO_AM_I != 0x47')
        self.b.write_byte_data(ADDR, GYRO_CONFIG0, 0x06)    # 1 kHz, +/-2000 dps
        self.b.write_byte_data(ADDR, ACCEL_CONFIG0, 0x06)   # 1 kHz, +/-16 g
        self.b.write_byte_data(ADDR, PWR_MGMT0, 0x0F)       # accel+gyro low-noise
        time.sleep(0.05)

    def sample(self):
        d = self.b.read_i2c_block_data(ADDR, ACCEL_DATA_X1, 12)
        def s16(i):
            v = (d[i] << 8) | d[i + 1]
            return v - 65536 if v & 0x8000 else v
        return ([s16(0) / ACCEL_LSB_PER_G * G, s16(2) / ACCEL_LSB_PER_G * G,
                 s16(4) / ACCEL_LSB_PER_G * G],
                [s16(6) / GYRO_LSB_PER_DPS * DEG2RAD, s16(8) / GYRO_LSB_PER_DPS * DEG2RAD,
                 s16(10) / GYRO_LSB_PER_DPS * DEG2RAD])


def collect(imu, sec, hz=200.0):
    A, Gy, T = [], [], []
    dt, t_end = 1.0 / hz, time.time() + sec
    nxt = time.time()
    while time.time() < t_end:
        a, g = imu.sample()
        A.append(a); Gy.append(g); T.append(time.time())
        nxt += dt
        s = nxt - time.time()
        if s > 0: time.sleep(s)
    return A, Gy, T


def stats(v):
    n = len(v)
    m = sum(v) / n
    sd = math.sqrt(sum((x - m) ** 2 for x in v) / n) if n > 1 else 0.0
    return m, sd, min(v), max(v)


def do_bias(sec):
    imu = ICM()
    print(f'Collecting {sec:.0f}s stationary. DO NOT TOUCH THE ROBOT.')
    A, Gy, T = collect(imu, sec)
    n = len(A)
    rate = n / (T[-1] - T[0])
    print(f'\n{n} samples in {T[-1]-T[0]:.1f}s = {rate:.1f} Hz\n')

    print('ACCEL (m/s^2)      mean      sd      min      max')
    am = []
    for i, ax in enumerate(AX):
        m, sd, lo, hi = stats([r[i] for r in A]); am.append(m)
        print(f'  {ax}            {m:+8.4f} {sd:7.4f} {lo:+8.3f} {hi:+8.3f}')
    norm = math.sqrt(sum(x * x for x in am))
    up = max(range(3), key=lambda i: abs(am[i]))
    print(f'\n  |a| = {norm:.4f} m/s^2  (expect {G:.4f}) -> scale err {100*(norm/G-1):+.2f}%')
    print(f'  UP axis = sensor {AX[up].upper()}{"+" if am[up] > 0 else "-"} '
          f'({am[up]:+.2f} of {norm:.2f})')

    print('\nGYRO (rad/s)       mean       sd     deg/min')
    gm = []
    for i, ax in enumerate(AX):
        m, sd, _, _ = stats([r[i] for r in Gy]); gm.append(m)
        print(f'  {ax}            {m:+9.6f} {sd:8.6f} {m*180/math.pi*60:+9.3f}')
    worst = max(abs(v) * 180 / math.pi * 60 for v in gm)
    print(f'\n  worst-axis drift {worst:.2f} deg/min   (MPU9250 was -15.5)')

    cal = {'sensor': 'ICM-42688-P', 'bus': BUS, 'addr': ADDR,
           'captured': time.strftime('%Y-%m-%dT%H:%M:%S'), 'samples': n,
           'rate_hz': round(rate, 1),
           'gyro_bias_rad_s': [round(v, 8) for v in gm],
           'accel_mean_m_s2': [round(v, 5) for v in am],
           'accel_scale_correction': round(G / norm, 6),
           'up_axis': f'{AX[up]}{"+" if am[up] > 0 else "-"}',
           'gyro_noise_sd_rad_s': [round(stats([r[i] for r in Gy])[1], 8) for i in range(3)]}
    os.makedirs(os.path.dirname(CAL), exist_ok=True)
    with open(CAL, 'w') as f:
        json.dump(cal, f, indent=2)
    print(f'\nwrote {os.path.normpath(CAL)}')


def do_motion(sec, label, thresh=0.15):
    """Record for `sec` seconds and AUTO-DETECT the active motion segment, so the
    operator can perform the motion at any point inside the window."""
    import sys
    imu = ICM()
    print(f'  [{label}] recording {sec:.0f}s -- perform the motion ANY TIME in the window.',
          flush=True)
    A, Gy, T = [], [], []
    t_end = time.time() + sec
    last = 1e9
    while time.time() < t_end:
        a, g = imu.sample()
        A.append(a); Gy.append(g); T.append(time.time())
        rem = t_end - time.time()
        if last - rem > 2.0:
            last = rem
            sys.stdout.write(f'\r  {rem:4.0f}s left   gyro=({g[0]:+5.2f},{g[1]:+5.2f},'
                             f'{g[2]:+5.2f}) rad/s  ')
            sys.stdout.flush()
    print(f'\r  done -- {len(A)} samples' + ' ' * 40, flush=True)

    n = len(A)
    dt = (T[-1] - T[0]) / max(n - 1, 1)
    bias = [sum(r[i] for r in Gy[:100]) / min(100, n) for i in range(3)]
    mag = [math.sqrt(sum((r[i] - bias[i]) ** 2 for i in range(3))) for r in Gy]
    act = [k for k, m in enumerate(mag) if m > thresh]
    if not act:
        print(f'\n  *** NO MOTION DETECTED (peak {max(mag):.3f} < {thresh} rad/s) ***')
        print('  The robot never rotated during the window. Nothing to report.')
        return
    lo, hi = max(0, act[0] - 20), min(n - 1, act[-1] + 20)
    print(f'\n  motion segment: samples {lo}..{hi}  ({(hi-lo)*dt:.1f}s of {sec:.0f}s)')
    print(f'  peak rate {max(mag):.3f} rad/s = {max(mag)*180/math.pi:.0f} deg/s\n')

    print('  ACCEL      before   after   delta  |  GYRO     peak    integrated deg')
    ba = [sum(r[i] for r in A[max(0,lo-20):lo+1]) / max(1, len(A[max(0,lo-20):lo+1]))
          for i in range(3)]
    ea = [sum(r[i] for r in A[hi:hi+21]) / max(1, len(A[hi:hi+21])) for i in range(3)]
    integ = []
    for i, ax in enumerate(AX):
        seg = [Gy[k][i] - bias[i] for k in range(lo, hi + 1)]
        pk = max(seg, key=abs)
        ang = sum(seg) * dt * 180 / math.pi
        integ.append(ang)
        print(f'  {ax}      {ba[i]:+7.2f} {ea[i]:+7.2f} {ea[i]-ba[i]:+7.2f}  |  '
              f'{pk:+7.3f} {ang:+11.1f}')
    gi = max(range(3), key=lambda i: abs(integ[i]))
    print(f'\n  DOMINANT GYRO AXIS: sensor {AX[gi].upper()}  '
          f'{integ[gi]:+.1f} deg  ({"POSITIVE" if integ[gi] > 0 else "NEGATIVE"})')
    print(f'  ROS-frame yaw (ros_z = +sensor_z): {integ[2]:+.1f} deg')


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--bias', nargs='?', type=float, const=20.0)
    p.add_argument('--motion', type=float)
    p.add_argument('--label', default='motion')
    a = p.parse_args()
    if a.bias: do_bias(a.bias)
    elif a.motion: do_motion(a.motion, a.label)
    else: p.error('need --bias or --motion')
