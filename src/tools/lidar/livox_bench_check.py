#!/usr/bin/env python3
"""Bench characterization of a Livox Mid-360S from livox_ros_driver2 topics.

Collects /livox/lidar (PointCloud2) and /livox/imu for a fixed window, then
reports rates, points per frame, range/elevation coverage, the near blind
zone, intensity, and the IMU gravity vector (which gives the lidar's
mounting attitude). Writes a JSON summary and an .npz with one merged cloud.

    python3 livox_bench_check.py --seconds 20 --out artifacts/livox-bench
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
        super().__init__('livox_bench_check')
        self.clouds = []      # (recv_time, stamp, Nx4 xyz+intensity)
        self.fields = None
        self.imu = []         # (recv_time, stamp, ax, ay, az, gx, gy, gz)
        self.create_subscription(PointCloud2, '/livox/lidar', self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(Imu, '/livox/imu', self.on_imu, qos_profile_sensor_data)

    def on_cloud(self, msg):
        if self.fields is None:
            self.fields = [(f.name, f.datatype) for f in msg.fields]
        names = [n for n in ('x', 'y', 'z', 'intensity') if n in [f[0] for f in self.fields]]
        pts = point_cloud2.read_points(msg, field_names=names, skip_nans=True)
        pts = np.stack([pts[n].astype(np.float64) for n in names], axis=1)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.clouds.append((time.monotonic(), stamp, pts))

    def on_imu(self, m):
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        a, g = m.linear_acceleration, m.angular_velocity
        self.imu.append((time.monotonic(), stamp, a.x, a.y, a.z, g.x, g.y, g.z))


def rate(times):
    d = np.diff(np.asarray(times))
    if len(d) == 0:
        return None
    return {'hz': float(1.0 / d.mean()), 'dt_mean_ms': float(d.mean() * 1e3),
            'dt_std_ms': float(d.std() * 1e3), 'dt_max_ms': float(d.max() * 1e3)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=20.0)
    ap.add_argument('--out', default='artifacts/livox-bench')
    args = ap.parse_args()

    rclpy.init()
    node = Collector()
    t_end = time.monotonic() + args.seconds
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    os.makedirs(args.out, exist_ok=True)
    res = {'seconds': args.seconds, 'fields': node.fields}

    if node.clouds:
        counts = np.array([len(c[2]) for c in node.clouds])
        allp = np.concatenate([c[2] for c in node.clouds])
        xyz = allp[:, :3]
        r = np.linalg.norm(xyz, axis=1)
        valid = r > 1e-3
        xyz, r = xyz[valid], r[valid]
        elev = np.degrees(np.arcsin(xyz[:, 2] / r))
        az = np.degrees(np.arctan2(xyz[:, 1], xyz[:, 0]))
        res['cloud'] = {
            'frames': len(node.clouds),
            'rate_stamp': rate([c[1] for c in node.clouds]),
            'rate_recv': rate([c[0] for c in node.clouds]),
            'points_per_frame': {'mean': float(counts.mean()), 'min': int(counts.min()), 'max': int(counts.max())},
            'points_per_sec': float(counts.sum() / args.seconds),
            'zero_points_fraction': float(1 - valid.mean()),
            'range_m': {f'p{q}': float(np.percentile(r, q)) for q in (0.1, 1, 5, 50, 95, 99, 99.9)},
            'range_min_m': float(r.min()), 'range_max_m': float(r.max()),
            'elevation_deg': {f'p{q}': float(np.percentile(elev, q)) for q in (0.1, 1, 50, 99, 99.9)},
            'azimuth_coverage_deg': int(len(np.unique(np.floor(az)))),
        }
        if allp.shape[1] > 3:
            inten = allp[valid, 3]
            res['cloud']['intensity'] = {f'p{q}': float(np.percentile(inten, q)) for q in (1, 50, 99)}
        # Stamp vs receive latency is only meaningful if the driver uses host time.
        lat = np.array([c[0] for c in node.clouds])
        res['cloud']['stamp_minus_wall_s'] = float(np.median(
            [c[1] for c in node.clouds]) - time.time() + (time.monotonic() - np.median(lat)))
        # Save the last ~1 s merged, plus one single frame.
        n1 = max(1, int(round(res['cloud']['rate_stamp']['hz'] or 10)))
        np.savez_compressed(os.path.join(args.out, 'clouds.npz'),
                            merged_1s=np.concatenate([c[2] for c in node.clouds[-n1:]]),
                            single=node.clouds[-1][2])

    if node.imu:
        im = np.array(node.imu)
        acc, gyr = im[:, 2:5], im[:, 5:8]
        g = acc.mean(axis=0)
        gn = np.linalg.norm(g)
        res['imu'] = {
            'samples': len(im),
            'rate_stamp': rate(im[:, 1]),
            'accel_mean': g.tolist(), 'accel_norm': float(gn),
            'accel_std': acc.std(axis=0).tolist(),
            'gyro_mean_rad_s': gyr.mean(axis=0).tolist(),
            'gyro_std_rad_s': gyr.std(axis=0).tolist(),
            # Tilt of the sensor's z axis from vertical, from gravity.
            'roll_deg': float(np.degrees(np.arctan2(g[1], g[2]))),
            'pitch_deg': float(np.degrees(np.arctan2(-g[0], np.hypot(g[1], g[2])))),
        }

    path = os.path.join(args.out, 'summary.json')
    with open(path, 'w') as f:
        json.dump(res, f, indent=2)
    print(json.dumps(res, indent=2))
    print('wrote', path)


if __name__ == '__main__':
    main()
