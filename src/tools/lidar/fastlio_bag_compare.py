#!/usr/bin/env python3
"""Compare an offline FAST-LIO run against the wheel/IMU odometry in the same bag.

    python3 fastlio_bag_compare.py SOURCE_BAG FASTLIO_OUT_BAG --out DIR

SOURCE_BAG is the recorded drive (has /odom, /odom_raw); FASTLIO_OUT_BAG holds
FAST-LIO's /Odometry from replaying that drive (scripts/fastlio_replay.sh).  Both are read straight
from sqlite, no ROS graph needed.

Reports, per source: path length, start-to-end closure (the drive starts and
ends on one mark, so closure ~ drift), yaw closure, and for FAST-LIO the z and
roll/pitch excursions (the floor is flat, so these should stay near zero).
Then aligns /odom onto FAST-LIO in 2D (rigid fit over the first 20 s, so the
later divergence is drift, not a frame offset) and reports how far they part.

FAST-LIO's frame is the Livox IMU ("body"), ~5 cm from base_footprint in x/y;
over a yaw difference of a few degrees that lever arm is < 1 cm, ignored.
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


def read_odom(bag, topic):
    db = sqlite3.connect(glob.glob(os.path.join(bag, '*.db3'))[0])
    tid = db.execute('select id from topics where name=?', (topic,)).fetchone()
    if tid is None:
        return None
    rows = []
    for (data,) in db.execute('select data from messages where topic_id=? order by timestamp', tid):
        m = deserialize_message(data, Odometry)
        p, q = m.pose.pose.position, m.pose.pose.orientation
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x ** 2 + q.y ** 2))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        rows.append((t, p.x, p.y, p.z, roll, pitch, yaw))
    a = np.array(rows)
    a[:, 6] = np.unwrap(a[:, 6])
    return a


def stats(a):
    xy = a[:, 1:3]
    return {
        'samples': len(a), 'duration_s': round(a[-1, 0] - a[0, 0], 1),
        'path_m': round(float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1))), 3),
        'closure_m': round(float(np.linalg.norm(xy[-1] - xy[0])), 3),
        'closure_yaw_deg': round(math.degrees(a[-1, 6] - a[0, 6]), 2),
        'z_range_m': round(float(np.ptp(a[:, 3])), 3),
        'max_abs_roll_deg': round(math.degrees(np.abs(a[:, 4]).max()), 2),
        'max_abs_pitch_deg': round(math.degrees(np.abs(a[:, 5]).max()), 2),
    }


def rigid2d(src, dst):
    """R, t minimizing |R src + t - dst| (Umeyama, no scale)."""
    ms, md = src.mean(0), dst.mean(0)
    u, _, vt = np.linalg.svd((dst - md).T @ (src - ms))
    d = np.sign(np.linalg.det(u @ vt))
    R = u @ np.diag([1, d]) @ vt
    return R, md - R @ ms


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('source_bag')
    ap.add_argument('fastlio_bag')
    ap.add_argument('--align-seconds', type=float, default=20.0)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    lio = read_odom(args.fastlio_bag, '/Odometry')
    odo = read_odom(args.source_bag, '/odom')
    raw = read_odom(args.source_bag, '/odom_raw')
    res = {'fastlio': stats(lio), 'odom': stats(odo)}
    if raw is not None:
        res['odom_raw'] = stats(raw)

    # time-align /odom onto FAST-LIO stamps
    t = lio[:, 0]
    keep = (t >= odo[0, 0]) & (t <= odo[-1, 0])
    t, L = t[keep], lio[keep]
    O = np.column_stack([np.interp(t, odo[:, 0], odo[:, i]) for i in (1, 2, 6)])
    w = t < t[0] + args.align_seconds
    if w.sum() < 10 or np.ptp(O[w, :2], axis=0).max() < 0.05:
        w = t < t[0] + 60.0          # robot still at the start: use the first minute
    R, tr = rigid2d(O[w, :2], L[w, 1:3])
    Oxy = (R @ O[:, :2].T).T + tr
    dyaw0 = math.atan2(R[1, 0], R[0, 0])
    sep = np.linalg.norm(Oxy - L[:, 1:3], axis=1)
    dyaw = np.degrees((O[:, 2] + dyaw0) - L[:, 6])
    dyaw -= dyaw[0]
    res['odom_vs_fastlio'] = {
        'aligned_on_first_s': args.align_seconds,
        'separation_end_m': round(float(sep[-1]), 3),
        'separation_max_m': round(float(sep.max()), 3),
        'yaw_diff_end_deg': round(float(dyaw[-1]), 2),
        'yaw_diff_maxabs_deg': round(float(np.abs(dyaw).max()), 2),
    }
    print(json.dumps(res, indent=2))
    with open(os.path.join(args.out, 'compare.json'), 'w') as f:
        json.dump(res, f, indent=2)
    np.savez_compressed(os.path.join(args.out, 'tracks.npz'), t=t, fastlio=L, odom_aligned=Oxy,
                        odom_yaw=O[:, 2] + dyaw0, sep=sep)

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(13, 5.5))
        ax[0].plot(L[:, 1], L[:, 2], label='FAST-LIO', lw=1.6)
        ax[0].plot(Oxy[:, 0], Oxy[:, 1], label='/odom (wheel+IMU EKF)', lw=1.2)
        ax[0].plot(*L[0, 1:3], 'ko', label='start')
        ax[0].plot(*L[-1, 1:3], 'kx', ms=9, label='end (FAST-LIO)')
        ax[0].set_aspect('equal'); ax[0].grid(alpha=.3); ax[0].legend(); ax[0].set_title('Trajectories (m)')
        tt = t - t[0]
        ax[1].plot(tt, sep, label='position separation (m)')
        ax[1].plot(tt, np.abs(dyaw) / 10, label='|yaw difference| / 10 (deg)')
        ax[1].set_xlabel('s'); ax[1].grid(alpha=.3); ax[1].legend(); ax[1].set_title('/odom vs FAST-LIO')
        fig.tight_layout(); fig.savefig(os.path.join(args.out, 'compare.png'), dpi=110)
        print('wrote', os.path.join(args.out, 'compare.png'))
    except ImportError:
        pass


if __name__ == '__main__':
    main()
