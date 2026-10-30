#!/usr/bin/env python3
"""Capture independent odometry + Livox health. No motion commands, no TF writes.

Reports per-stream excursion/closure, NOT ground-truth accuracy. Frame alignment
and sensor lever-arm calibration are required before comparing moving poses.
"""
import argparse
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from livox_ros_driver2.msg import CustomMsg


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def summary(rows):
    if len(rows) < 2:
        return {'samples': len(rows), 'valid': False}
    a = np.asarray(rows)
    dt = np.diff(a[:, 0])
    xyz = a[:, 2:5]
    yaw = np.unwrap(a[:, 5])
    return {'samples': len(rows), 'valid': bool(np.all(dt > 0)),
            'hz': float((len(rows)-1) / (a[-1, 0]-a[0, 0])) if np.all(dt > 0) else None,
            'max_gap_s': float(dt.max()),
            'age_p50_s': float(np.median(a[:, 1])),
            'age_p95_s': float(np.percentile(a[:, 1], 95)),
            'position_excursion_m': float(np.linalg.norm(xyz-xyz[0], axis=1).max()),
            'endpoint_displacement_m': float(np.linalg.norm(xyz[-1]-xyz[0])),
            'yaw_excursion_deg': float(np.rad2deg(np.max(np.abs(yaw-yaw[0])))),
            'yaw_endpoint_deg': float(np.rad2deg(yaw[-1]-yaw[0]))}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--seconds', type=float, default=30)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = Node('fastlio_compare')
    streams = {t: [] for t in ['/odom', '/odom_raw', '/fastlio/odom']}
    frames = {}
    imu, clouds = [], []
    cloud_example = {}
    log = (out / 'samples.jsonl').open('w')

    def odom(m, topic):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        row = [stamp(m), node.get_clock().now().nanoseconds*1e-9-stamp(m),
               p.x, p.y, p.z, yaw]
        streams[topic].append(row)
        frames[topic] = [m.header.frame_id, m.child_frame_id]
        log.write(json.dumps({'topic': topic, 'data': row})+'\n')

    for topic in streams:
        node.create_subscription(Odometry, topic, lambda m, t=topic: odom(m, t), qos_profile_sensor_data)
    node.create_subscription(Imu, '/livox/imu', lambda m: imu.append(
        [stamp(m), m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z]), qos_profile_sensor_data)
    def cloud(raw):
        # Python expansion of 20k nested point messages can stall all other
        # callbacks. Decode one example only; count subsequent serialized frames.
        clouds.append(time.monotonic())
        if not cloud_example:
            m = deserialize_message(raw, CustomMsg)
            cloud_example.update(points=m.point_num,
                                 last_offset_ns=m.points[-1].offset_time if m.points else 0)

    node.create_subscription(CustomMsg, '/livox/lidar', cloud, qos_profile_sensor_data, raw=True)
    print(f'CAPTURE START: {args.seconds:g} seconds -> {out}', flush=True)
    end = time.monotonic()+args.seconds
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        log.close()
        node.destroy_node()
        rclpy.shutdown()
    result = {'interpretation': 'Excursion/closure only; no ground truth or cross-frame alignment.',
              'frames': frames, 'odometry': {t: summary(r) for t, r in streams.items()},
              'imu_samples': len(imu), 'cloud_samples': len(clouds),
              'imu_accel_norm_median_driver_units': float(np.median(np.linalg.norm(np.asarray(imu)[:, 1:], axis=1))) if imu else None,
              'cloud_example': cloud_example,
              'cloud_receive_hz': (len(clouds)-1)/(clouds[-1]-clouds[0]) if len(clouds)>1 else None}
    (out / 'summary.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    if not all(len(r) >= 2 for r in streams.values()) or not imu or not clouds:
        raise SystemExit('INCOMPLETE: missing stream; do not interpret as zero drift.')


if __name__ == '__main__':
    main()
