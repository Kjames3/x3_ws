#!/usr/bin/env python3
"""Measure the Livox Mid-360S mounting height and tilt from the floor.

Park the robot on flat, open floor (clear well past the ~2 m blind radius),
keep still, and run with the driver publishing PointCloud2 in the SENSOR
frame (extrinsic_parameter all zero in MID360s_config.json):

    ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
        -p xfer_format:=0 -p frame_id:=livox_frame -p publish_freq:=10.0 \
        -p user_config_path:=$HOME/ws_livox/src/livox_ros_driver2/config/MID360s_config.json
    python3 livox_floor_fit.py --seconds 10 --out artifacts/livox-floor

A RANSAC plane fit on the floor gives the optical-centre height (plane
distance from the origin) and roll/pitch (from the plane normal).  Those are
the lidar's attitude relative to the floor, i.e. the URDF rpy of the lidar
frame if base_link is level.  The Livox IMU gravity vector is reported as an
independent tilt check (accel is in g; the IMU axes follow the lidar's).

Per-sector offsets show whether the floor is really one plane: a raised
platform or a rug shows up as one sector sitting a few mm high.
"""
import argparse
import json
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2


class Collector(Node):
    def __init__(self):
        super().__init__('livox_floor_fit')
        self.clouds = []
        self.acc = []
        self.create_subscription(PointCloud2, '/livox/lidar', self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(Imu, '/livox/imu', self.on_imu, qos_profile_sensor_data)

    def on_cloud(self, msg):
        pts = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        self.clouds.append(np.stack([pts[n].astype(np.float64) for n in 'xyz'], axis=1))

    def on_imu(self, msg):
        a = msg.linear_acceleration
        self.acc.append((a.x, a.y, a.z))


def ransac_plane(p, thresh, iters, max_tilt_deg, rng):
    """Best plane n.x + d = 0 with n pointing up (toward the sensor)."""
    best, best_count = None, 0
    cos_max = np.cos(np.radians(max_tilt_deg))
    for _ in range(iters):
        a, b, c = p[rng.choice(len(p), 3, replace=False)]
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n /= norm
        if n[2] < 0:
            n = -n
        if n[2] < cos_max:      # not floor-like
            continue
        d = -n @ a
        count = np.count_nonzero(np.abs(p @ n + d) < thresh)
        if count > best_count:
            best, best_count = (n, d), count
    return best


def refine(p, n, d, thresh):
    """Least-squares (SVD) refit on the inliers of (n, d)."""
    for _ in range(3):
        inl = p[np.abs(p @ n + d) < thresh]
        c = inl.mean(axis=0)
        n_new = np.linalg.svd(inl - c, full_matrices=False)[2][2]
        if n_new[2] < 0:
            n_new = -n_new
        n, d = n_new, -n_new @ c
    return n, d, inl


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seconds', type=float, default=10.0)
    ap.add_argument('--min-range', type=float, default=1.2, help='horizontal range window (m)')
    ap.add_argument('--max-range', type=float, default=6.0)
    ap.add_argument('--thresh', type=float, default=0.015, help='RANSAC inlier band (m)')
    ap.add_argument('--refine-thresh', type=float, default=0.010)
    ap.add_argument('--max-tilt', type=float, default=15.0, help='reject planes tilted more than this (deg)')
    ap.add_argument('--out', default=None, help='directory for summary.json + floor.npz')
    args = ap.parse_args()

    rclpy.init()
    node = Collector()
    t_end = time.monotonic() + args.seconds
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    if not node.clouds:
        raise SystemExit('no /livox/lidar messages -- is the driver running with xfer_format:=0?')

    p = np.concatenate(node.clouds)
    p = p[np.any(p != 0, axis=1)]                 # no-return points come back as zeros
    r_h = np.hypot(p[:, 0], p[:, 1])
    cand = p[(r_h > args.min_range) & (r_h < args.max_range) & (p[:, 2] < -0.05)]
    print(f'{len(node.clouds)} frames, {len(p)} returns, {len(cand)} floor candidates')
    if len(cand) < 500:
        raise SystemExit('too few low points in the range window; floor not visible?')

    rng = np.random.default_rng(0)
    sub = cand[rng.choice(len(cand), min(len(cand), 20000), replace=False)]
    # Furniture bases and wall skirts form a denser horizontal band than the
    # floor itself (seen 2026-10-29: a plane at -0.11 m beat the real floor at
    # -0.32 m).  Keep stepping down while a well-supported plane lies below.
    fit = None
    while True:
        f = ransac_plane(sub, args.thresh, 400, args.max_tilt, rng)
        if f is None:
            break
        fit = f
        below = sub[sub @ f[0] + f[1] < -3 * args.thresh]
        if len(below) < max(300, 0.02 * len(sub)):
            break
        sub = below
    if fit is None:
        raise SystemExit('no floor-like plane found')
    n, d, inl = refine(cand, *fit, args.refine_thresh)
    resid = inl @ n + d

    height = d                                     # n points up, origin above floor
    roll = np.degrees(np.arctan2(n[1], n[2]))
    pitch = np.degrees(np.arctan2(-n[0], n[2]))    # + = nose down (ROS)

    print(f'\nheight  {height:.4f} m   (optical centre above the fitted floor)')
    print(f'roll    {roll:+.2f} deg')
    print(f'pitch   {pitch:+.2f} deg  (+ = nose down)')
    print(f'inliers {len(inl)} ({100 * len(inl) / len(cand):.0f}% of candidates), '
          f'residual sd {1000 * resid.std():.1f} mm')

    # Per-sector offset of the floor points from the global plane.
    az = np.degrees(np.arctan2(inl[:, 1], inl[:, 0]))
    sectors = {}
    print('\nsector       points  offset_mm  (+ = floor higher than the fit)')
    for name, lo, hi in (('front', -45, 45), ('left', 45, 135), ('rear', 135, 225), ('right', -135, -45)):
        a = (az - lo) % 360
        m = a < (hi - lo)
        if m.sum() < 50:
            print(f'{name:<10} {m.sum():>8}  (too few)')
            continue
        off = 1000 * np.median(resid[m])
        sectors[name] = {'points': int(m.sum()), 'offset_mm': round(float(off), 2)}
        print(f'{name:<10} {m.sum():>8}  {off:+8.1f}')

    imu = None
    if node.acc:
        g = np.mean(node.acc, axis=0)
        gn = g / np.linalg.norm(g)
        imu = {'g_mean': [round(float(v), 4) for v in g],
               'roll_deg': round(float(np.degrees(np.arctan2(gn[1], gn[2]))), 2),
               'pitch_deg': round(float(np.degrees(np.arctan2(-gn[0], gn[2]))), 2)}
        print(f"\nIMU gravity |g|={np.linalg.norm(g):.3f}  roll {imu['roll_deg']:+.2f}  "
              f"pitch {imu['pitch_deg']:+.2f} deg  ({len(node.acc)} samples)")

    if args.out:
        os.makedirs(args.out, exist_ok=True)
        summary = {'height_m': round(float(height), 4), 'roll_deg': round(float(roll), 3),
                   'pitch_deg': round(float(pitch), 3), 'normal': [float(v) for v in n],
                   'inliers': int(len(inl)), 'residual_sd_mm': round(float(1000 * resid.std()), 2),
                   'sectors': sectors, 'imu': imu, 'frames': len(node.clouds), 'args': vars(args)}
        with open(os.path.join(args.out, 'summary.json'), 'w') as f:
            json.dump(summary, f, indent=2)
        np.savez_compressed(os.path.join(args.out, 'floor.npz'), inliers=inl.astype(np.float32),
                            candidates=cand.astype(np.float32))
        print(f'\nwrote {args.out}/summary.json, floor.npz')


if __name__ == '__main__':
    main()
