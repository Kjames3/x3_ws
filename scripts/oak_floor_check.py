#!/usr/bin/env python3
"""Is the OAK-D seeing a flat floor, or a floor that tilts up with range?

The voxel layer marks any point above min_obstacle_height. If the reconstructed
floor rises with distance -- from a camera pitch/extrinsic error, or from depth
bias -- it will cross that gate at some range and the robot will start seeing a
phantom wall made of its own floor. That range is the single most useful number
for judging whether the depth obstacle layer is trustworthy.

Point the robot at open, flat floor with no real obstacles inside ~3 m and run:

    python3 oak_floor_check.py

A healthy result is a median floor height that stays flat within a couple of cm
across all range bins. A rising trend is a calibration problem, not noise.
"""
import argparse
import math
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener


def quat_to_mat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--width", type=float, default=40.0,
                    help="forward cone width in degrees")
    ap.add_argument("--frames", type=int, default=25)
    ap.add_argument("--z-gate", type=float, default=0.12,
                    help="oak_voxel_layer min_obstacle_height")
    ap.add_argument("--floor-band", type=float, default=0.35,
                    help="points below this height are treated as floor candidates")
    args = ap.parse_args()

    rclpy.init()
    node = Node("oak_floor_check")
    buf = Buffer()
    TransformListener(buf, node)
    clouds = []
    node.create_subscription(PointCloud2, "/oak/points", clouds.append,
                             qos_profile_sensor_data)

    half = math.radians(args.width) / 2.0
    allpts = []
    t0 = time.time()
    try:
        while len(allpts) < args.frames and time.time() - t0 < 30:
            rclpy.spin_once(node, timeout_sec=0.2)
            while clouds:
                m = clouds.pop(0)
                n = m.width * m.height
                if n == 0:
                    continue
                p = np.frombuffer(m.data, dtype=np.float32,
                                  count=n * (m.point_step // 4))
                p = p.reshape(n, m.point_step // 4)[:, :3]
                try:
                    tf = buf.lookup_transform("base_footprint", m.header.frame_id,
                                              rclpy.time.Time())
                except Exception:  # noqa: BLE001 - TF not ready yet
                    continue
                t = tf.transform.translation
                p = p @ quat_to_mat(tf.transform.rotation).T
                p = p + np.array([t.x, t.y, t.z], dtype=np.float32)
                allpts.append(p)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    if not allpts:
        print("no clouds received (or TF base_footprint <- oak frame missing)")
        return

    p = np.concatenate(allpts, axis=0)
    rng = np.hypot(p[:, 0], p[:, 1])
    ang = np.arctan2(p[:, 1], p[:, 0])
    sel = (np.abs(ang) <= half) & (rng > 0.3) & (p[:, 2] < args.floor_band)
    p, rng = p[sel], rng[sel]
    print(f"{len(allpts)} clouds, {len(p)} floor-candidate points "
          f"in the forward {args.width:.0f} deg cone\n")
    # Real objects standing on the floor drag the median up, so the floor
    # itself is better estimated by a low percentile of the height histogram.
    print("  range bin      n       p25 z      median z     p95 z")

    edges = np.arange(0.4, 3.61, 0.4)
    mids, floors = [], []
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (rng >= lo) & (rng < hi)
        if m.sum() < 30:
            print(f"  {lo:.1f}-{hi:.1f} m   {int(m.sum()):6d}   (too few points)")
            continue
        p25 = float(np.percentile(p[m, 2], 25))
        med = float(np.median(p[m, 2]))
        p95 = float(np.percentile(p[m, 2], 95))
        flag = "  <-- FLOOR ABOVE GATE" if p25 >= args.z_gate else ""
        print(f"  {lo:.1f}-{hi:.1f} m   {int(m.sum()):6d}   "
              f"{p25:+.3f} m   {med:+.3f} m   {p95:+.3f} m{flag}")
        mids.append((lo + hi) / 2)
        floors.append(p25)

    if len(mids) >= 3:
        slope, icept = np.polyfit(mids, floors, 1)
        print(f"\n  floor fit: z = {slope:+.4f} * range {icept:+.4f}")
        print(f"  implied pitch error: {math.degrees(math.atan(slope)):+.2f} deg")
        if slope > 0.01:
            cross = (args.z_gate - icept) / slope
            print(f"  -> floor crosses the {args.z_gate:.2f} m gate at "
                  f"{cross:.2f} m: beyond that the floor marks as an obstacle")
        else:
            print("  -> floor is flat within tolerance; no phantom-floor risk")


if __name__ == "__main__":
    main()
