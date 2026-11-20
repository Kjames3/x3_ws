#!/usr/bin/env python3
"""Spot-check the front OAK's stereo depth against a tape measure, through
oakd_driver's own pipeline (the one config/oak_depth_correction.json applies to).

Park the robot square-on to a flat wall (or a big box face) at a tape-measured
distance from the OAK glass, then run. Prints the median depth in a central
ROI once a second; Ctrl-C prints the overall median. x3_server must run with
--oak-ros-publish so /oak/depth/image_raw exists.

    python3 src/tools/calibration/oak_depth_spot_check.py --truth 1.00
"""
import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--truth', type=float, required=True, help='tape distance, OAK glass to wall (m)')
    ap.add_argument('--roi', type=float, default=0.12, help='ROI half-size as a fraction of the image')
    a = ap.parse_args()

    rclpy.init()
    node = Node('oak_depth_spot_check')
    vals, last = [], [0.0, []]

    def cb(m):
        if m.encoding not in ('16UC1', 'mono16'):
            dt, scale = np.float32, 1.0
        else:
            dt, scale = np.uint16, 0.001
        d = np.frombuffer(bytes(m.data), dt).reshape(m.height, m.width).astype(np.float32) * scale
        h, w = d.shape
        r = int(a.roi * min(h, w))
        roi = d[h // 2 - r:h // 2 + r, w // 2 - r:w // 2 + r]
        good = roi[(roi > 0.1) & np.isfinite(roi)]
        if good.size < 0.3 * roi.size:
            return
        z = float(np.median(good))
        vals.append(z)
        last[1].append(z)
        if time.time() - last[0] > 1.0:
            zz = float(np.median(last[1]))
            print(f'depth {zz:.3f} m  ({100 * (zz / a.truth - 1):+.1f}% vs {a.truth:.3f})  '
                  f'valid {good.size / roi.size:.0%}')
            last[0], last[1] = time.time(), []

    node.create_subscription(Image, '/oak/depth/image_raw', cb, qos_profile_sensor_data)
    print('waiting for /oak/depth/image_raw ...')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if vals:
        z = float(np.median(vals))
        print(f'\n{len(vals)} frames: median {z:.3f} m, p10-p90 {np.percentile(vals, 10):.3f}-'
              f'{np.percentile(vals, 90):.3f}, error {100 * (z / a.truth - 1):+.1f}%')
    import os
    os._exit(0)


if __name__ == '__main__':
    main()
