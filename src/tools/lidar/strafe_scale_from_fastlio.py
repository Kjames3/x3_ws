#!/usr/bin/env python3
"""Per-pass strafe (and forward) odometry scale, against FAST-LIO.

    python3 strafe_scale_from_fastlio.py SOURCE_BAG FASTLIO_OUT_BAG [--out DIR]

Passes are the motion between pauses in /odom_raw (wheel odometry, the thing
being calibrated): drive a pass, stop ~1 s, drive the next.  A pass is strafe
if its sideways travel is over twice its forward travel.  Wheel distance is
integrated from the twist; FAST-LIO's is summed step by step in the body
frame, so both are body-frame distances even when the pass turns.  The ratio
FAST-LIO / odom multiplies linear_scale_y (or _x).  Yaw over a pass is real
rotation when odometry and FAST-LIO agree on it; no odometry scale fixes that.

FAST-LIO run with config/fastlio_mid360_icm.yaml reports the ICM frame
(base axes, ~3 cm behind centre); the lever arm is a few mm per pass, ignored.
"""
import argparse
import glob
import json
import math
import os
import sqlite3

import numpy as np
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

from fastlio_bag_compare import read_odom


def read_twist(bag, topic='/odom_raw'):
    db = sqlite3.connect(glob.glob(os.path.join(bag, '*.db3'))[0])
    tid = db.execute('select id from topics where name=?', (topic,)).fetchone()
    rows = []
    for (data,) in db.execute('select data from messages where topic_id=? order by timestamp', tid):
        m = deserialize_message(data, Odometry)
        tw = m.twist.twist
        rows.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                     tw.linear.x, tw.linear.y, tw.angular.z))
    return np.array(rows)


def still_windows(t, vx, vy, wz, min_s=0.8, eps=0.005):
    """Stretches where the wheels report no motion at all (the pauses)."""
    still = np.abs(vx) + np.abs(vy) + np.abs(wz) < eps
    out, s = [], None
    for i, f in enumerate(np.append(still, False)):
        if f and s is None:
            s = i
        elif not f and s is not None:
            if t[i - 1] - t[s] >= min_s:
                out.append((t[s], t[i - 1]))
            s = None
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('source_bag')
    ap.add_argument('fastlio_bag')
    ap.add_argument('--min-m', type=float, default=0.3, help='ignore passes shorter than this')
    ap.add_argument('--max-turn-deg', type=float, default=45.0, help='skip passes that turn more')
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    R = read_twist(args.source_bag)
    L = read_odom(args.fastlio_bag, '/Odometry')
    t, vx, vy, wz = R.T

    # FAST-LIO increments rotated into the body frame AT EACH STEP -- what the
    # wheels measure.  Using the displacement in the pass's start frame is
    # wrong once a pass turns: 20 deg of yaw made it read 2-25% low.
    d = np.diff(L[:, 1:3], axis=0)
    y = L[:-1, 6]
    bx = np.cos(y) * d[:, 0] + np.sin(y) * d[:, 1]
    by = -np.sin(y) * d[:, 0] + np.cos(y) * d[:, 1]
    lt = L[:-1, 0]

    out = {'strafe': [], 'forward': []}
    W = still_windows(t, vx, vy, wz)
    for (_, a1), (b0, _) in zip(W[:-1], W[1:]):
        m = (t >= a1) & (t <= b0)
        dt = np.diff(t[m])
        ox, oy = np.sum(vx[m][:-1] * dt), np.sum(vy[m][:-1] * dt)
        oyaw = math.degrees(np.sum(wz[m][:-1] * dt))
        k = (lt >= a1) & (lt < b0)
        fx, fy = bx[k].sum(), by[k].sum()
        fyaw = math.degrees(np.interp(b0, L[:, 0], L[:, 6]) - np.interp(a1, L[:, 0], L[:, 6]))
        if abs(oyaw) > args.max_turn_deg:
            print(f'{a1 - t[0]:6.1f} turn of {oyaw:+.0f} deg, skipped')
            continue
        kind = 'strafe' if abs(oy) > 2 * abs(ox) else 'forward'
        odo, lio = (oy, fy) if kind == 'strafe' else (ox, fx)
        if abs(odo) < args.min_m:
            continue
        out[kind].append({
            't0': round(a1 - t[0], 1), 'dur_s': round(b0 - a1, 2),
            'speed': round(abs(odo) / (b0 - a1), 3),
            'odom_m': round(odo, 3), 'fastlio_m': round(lio, 3), 'ratio': round(lio / odo, 3),
            'cross_odom_m': round(ox if kind == 'strafe' else oy, 3),
            'cross_fastlio_m': round(fx if kind == 'strafe' else fy, 3),
            'yaw_odom_deg': round(oyaw, 2), 'yaw_fastlio_deg': round(fyaw, 2),
        })

    for kind in ('strafe', 'forward'):
        legs = out[kind]
        print(f'\n{kind} passes: {len(legs)}')
        print('    t0   dur  speed  odom_m  lio_m   ratio  yaw_odom  yaw_lio')
        for g in legs:
            print(f"{g['t0']:6.1f} {g['dur_s']:5.1f} {g['speed']:5.2f} {g['odom_m']:+7.3f} {g['fastlio_m']:+7.3f} "
                  f"{g['ratio']:6.3f} {g['yaw_odom_deg']:+8.1f} {g['yaw_fastlio_deg']:+8.1f}")
        for sign, name in ((1, 'pos'), (-1, 'neg')):
            sel = [g for g in legs if np.sign(g['odom_m']) == sign]
            if sel:
                o = sum(abs(g['odom_m']) for g in sel)
                f = sum(abs(g['fastlio_m']) for g in sel)
                print(f'  {"+" if sign > 0 else "-"} direction: {len(sel)} passes, odom {o:.2f} m, '
                      f'FAST-LIO {f:.2f} m, true/odom {f / o:.3f}')
                out[f'{kind}_scale_{name}'] = round(f / o, 4)
        if legs:
            o = sum(abs(g['odom_m']) for g in legs)
            f = sum(abs(g['fastlio_m']) for g in legs)
            out[f'{kind}_scale_all'] = round(f / o, 4)
            print(f'  all: true/odom {f / o:.3f}  (multiply linear_scale by this)')

    if args.out:
        os.makedirs(args.out, exist_ok=True)
        with open(os.path.join(args.out, 'strafe_scale.json'), 'w') as f:
            json.dump(out, f, indent=2)


if __name__ == '__main__':
    main()
