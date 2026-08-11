#!/usr/bin/env python3
"""Compare what the lidar sees, what the OAK-D sees, and what the costmap marks.

The point of Phase 0 is that the depth camera contributes obstacles the 2D lidar
cannot see. This makes that claim measurable: park the robot facing a test
object, run this, and read off three independent ranges for the same sector.

    lidar   nearest /scan return in the sector
    oak     nearest /oak/points return in the sector, with its height band
    costmap nearest lethal cell in the sector

A camera-only obstacle shows up as a large lidar range, a short oak range, and a
short costmap range. If the costmap range stays large while oak is short, the
voxel layer is dropping the detection (usually a height-gate or range-gate
problem, not a perception problem).

Usage:
    ros2 run ... no — just: python3 oak_obstacle_check.py [--bearing 0] [--width 20]
"""
import argparse
import math
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_ros import Buffer, TransformListener

LETHAL = 99  # nav2 marks lethal as 254 internally, 99/100 over the ROS msg


def quat_to_mat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def in_sector(ang, centre, half):
    """Angle test that wraps correctly across +/-pi."""
    d = np.arctan2(np.sin(ang - centre), np.cos(ang - centre))
    return np.abs(d) <= half


class Checker(Node):
    def __init__(self, bearing, width, scan_topic, z_min, z_max):
        super().__init__("oak_obstacle_check")
        self.centre = math.radians(bearing)
        self.half = math.radians(width) / 2.0
        self.z_min = z_min
        self.z_max = z_max
        self.gated_range = None
        self.costmap_missing = False
        self.scan = None
        self.cloud = None
        self.grid = None

        latched = QoSProfile(depth=1)
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        latched.reliability = QoSReliabilityPolicy.RELIABLE

        self.create_subscription(LaserScan, scan_topic,
                                 lambda m: setattr(self, "scan", m),
                                 qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/oak/points",
                                 lambda m: setattr(self, "cloud", m),
                                 qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap",
                                 lambda m: setattr(self, "grid", m), latched)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    # -- lidar ------------------------------------------------------------
    def lidar_range(self):
        m = self.scan
        if m is None:
            return None, "no /scan"
        r = np.asarray(m.ranges, dtype=np.float32)
        ang = m.angle_min + np.arange(r.size, dtype=np.float32) * m.angle_increment
        ok = np.isfinite(r) & (r > m.range_min) & (r < m.range_max)
        if not ok.any():
            return None, "no valid returns at all"

        # The 4ROS is mounted rpy="0 0 pi", so laser-frame 0 deg points BACKWARDS
        # in base_link. Bearings are meaningless unless we go through TF.
        try:
            tf = self.tf_buffer.lookup_transform(
                "base_link", m.header.frame_id, rclpy.time.Time())
        except Exception as e:  # noqa: BLE001
            return None, f"TF base_link <- {m.header.frame_id} failed: {e}"
        t = tf.transform.translation
        pts = np.stack([r[ok] * np.cos(ang[ok]),
                        r[ok] * np.sin(ang[ok]),
                        np.zeros(int(ok.sum()), dtype=np.float32)], axis=1)
        pts = pts @ quat_to_mat(tf.transform.rotation).T
        pts = pts + np.array([t.x, t.y, t.z], dtype=np.float32)

        brng = np.arctan2(pts[:, 1], pts[:, 0])
        rng = np.hypot(pts[:, 0], pts[:, 1])
        sel = in_sector(brng, self.centre, self.half)
        if not sel.any():
            return None, f"no valid returns in sector ({int(ok.sum())} valid overall)"
        return float(rng[sel].min()), f"{int(sel.sum())} beams in sector"

    # -- oak --------------------------------------------------------------
    def oak_range(self):
        """Nearest OAK return in the sector, and — the number that actually
        predicts marking — the nearest one inside the voxel layer's height gate.

        Heights are taken in base_footprint (ground level), matching how nav2
        applies min/max_obstacle_height in the costmap's global frame.
        """
        m = self.cloud
        if m is None:
            return None, "no /oak/points", None
        n = m.width * m.height
        if n == 0:
            return None, "cloud is empty (0 points)", None
        pts = np.frombuffer(m.data, dtype=np.float32,
                            count=n * (m.point_step // 4))
        pts = pts.reshape(n, m.point_step // 4)[:, :3]

        ref, tf = None, None
        for cand in ("base_footprint", "base_link"):
            try:
                tf = self.tf_buffer.lookup_transform(cand, m.header.frame_id,
                                                     rclpy.time.Time())
                ref = cand
                break
            except Exception:  # noqa: BLE001 - try the fallback frame
                continue
        if tf is None:
            return None, f"TF base_footprint <- {m.header.frame_id} failed", None
        t = tf.transform.translation
        pts = pts @ quat_to_mat(tf.transform.rotation).T
        pts = pts + np.array([t.x, t.y, t.z], dtype=np.float32)

        rng = np.hypot(pts[:, 0], pts[:, 1])
        ang = np.arctan2(pts[:, 1], pts[:, 0])
        sel = in_sector(ang, self.centre, self.half) & (rng > 0.05)
        if not sel.any():
            return None, f"no points in sector ({n} in cloud)", None
        s = pts[sel]
        srng = rng[sel]
        i = int(np.argmin(srng))
        band = (float(s[:, 2].min()), float(s[:, 2].max()))

        gated = (s[:, 2] >= self.z_min) & (s[:, 2] <= self.z_max)
        note = (f"{int(sel.sum())}/{n} points in sector [{ref}], "
                f"nearest at z={s[i, 2]:+.2f} m")
        if gated.any():
            j = int(np.argmin(srng[gated]))
            note += (f"; nearest INSIDE gate {self.z_min:.2f}-{self.z_max:.2f} m "
                     f"at {srng[gated][j]:.2f} m (z={s[gated][j, 2]:+.2f})")
            self.gated_range = float(srng[gated][j])
        else:
            note += (f"; NO points inside gate {self.z_min:.2f}-{self.z_max:.2f} m "
                     f"-> voxel layer correctly marks nothing here")
            self.gated_range = None
        return float(srng[i]), note, band

    # -- costmap ----------------------------------------------------------
    def costmap_range(self):
        g = self.grid
        # Absent topic and present-but-unmarked mean completely different
        # things, so flag the difference for the verdict logic.
        self.costmap_missing = g is None
        if g is None:
            return None, ("no /local_costmap/costmap -- is Nav2 running? "
                          "without it only the lidar/oak rows are meaningful")
        try:
            tf = self.tf_buffer.lookup_transform(
                g.header.frame_id, "base_link", rclpy.time.Time())
        except Exception as e:  # noqa: BLE001
            return None, f"TF {g.header.frame_id} <- base_link failed: {e}"
        t = tf.transform.translation
        yaw = math.atan2(*(quat_to_mat(tf.transform.rotation)[1, 0],
                           quat_to_mat(tf.transform.rotation)[0, 0]))

        data = np.asarray(g.data, dtype=np.int16).reshape(g.info.height,
                                                          g.info.width)
        res = g.info.resolution

        def nearest(threshold):
            ys, xs = np.nonzero(data >= threshold)
            if xs.size == 0:
                return None, 0
            wx = g.info.origin.position.x + (xs + 0.5) * res
            wy = g.info.origin.position.y + (ys + 0.5) * res
            dx, dy = wx - t.x, wy - t.y
            rng = np.hypot(dx, dy)
            ang = np.arctan2(dy, dx) - yaw
            sel = in_sector(ang, self.centre, self.half)
            if not sel.any():
                return None, 0
            return float(rng[sel].min()), int(sel.sum())

        lethal, n_lethal = nearest(LETHAL)
        infl, n_infl = nearest(1)
        if lethal is None and infl is None:
            return None, "no marked cells at all in sector"
        note = f"{n_lethal} lethal cells in sector"
        if infl is not None:
            note += f"; nearest cell of ANY cost at {infl:.2f} m ({n_infl} cells)"
        return lethal, note


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bearing", type=float, default=0.0,
                    help="sector centre in degrees, 0 = straight ahead, +CCW")
    ap.add_argument("--width", type=float, default=20.0,
                    help="full sector width in degrees")
    ap.add_argument("--duration", type=float, default=8.0,
                    help="seconds to collect before reporting")
    ap.add_argument("--scan-topic", default="/scan_fixed")
    ap.add_argument("--z-min", type=float, default=0.12,
                    help="must match oak_voxel_layer min_obstacle_height")
    ap.add_argument("--z-max", type=float, default=1.60,
                    help="must match oak_voxel_layer max_obstacle_height")
    ap.add_argument("--watch", action="store_true",
                    help="repeat forever instead of reporting once")
    args = ap.parse_args()

    rclpy.init()
    node = Checker(args.bearing, args.width, args.scan_topic,
                   args.z_min, args.z_max)

    print(f"sector: {args.bearing:+.0f} deg +/- {args.width / 2:.0f} deg, "
          f"scan topic {args.scan_topic}")
    first = True
    try:
        while True:
            t0 = time.time()
            while time.time() - t0 < (args.duration if first else 2.0):
                rclpy.spin_once(node, timeout_sec=0.1)
            first = False

            lr, lnote = node.lidar_range()
            orr, onote, band = node.oak_range()
            cr, cnote = node.costmap_range()

            print()
            print(f"  lidar   {'%6.2f m' % lr if lr else '     --':>9}   {lnote}")
            if band:
                onote += f"; sector height band {band[0]:+.2f}..{band[1]:+.2f} m"
            print(f"  oak     {'%6.2f m' % orr if orr else '     --':>9}   {onote}")
            print(f"  costmap {'%6.2f m' % cr if cr else '     --':>9}   {cnote}")

            # Only gated points can ever reach the costmap, so the verdict has
            # to be built from those, not from the raw nearest return.
            gr = node.gated_range
            if lr and gr:
                gap = lr - gr
                if gap > 0.30:
                    print(f"  -> CAMERA-ONLY OBSTACLE: oak sees something at "
                          f"{gr:.2f} m, {gap:.2f} m nearer than the lidar")
                else:
                    print(f"  -> both sensors agree within {abs(gap):.2f} m")
            if gr and cr is None and not node.costmap_missing:
                print("  -> REGRESSION: oak has in-gate points but the costmap "
                      "marks nothing (check obstacle_max_range / TF / layer enabled)")
            elif gr and cr and cr - gr > 0.40:
                print(f"  -> costmap lags the camera by {cr - gr:.2f} m")
            if not args.watch:
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
