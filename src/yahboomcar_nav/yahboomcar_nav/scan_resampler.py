#!/usr/bin/env python3
"""scan_resampler — republish /scan on a fixed-size angular grid.

Why this exists
---------------
slam_toolbox's Karto backend registers the LaserRangeFinder once, from the first
scan it sees, and then REJECTS every later scan whose beam count differs:

    LaserRangeScan contains 2807 range readings, expected 2791

The YDLidar X3 spins at a slightly variable rate, so with the driver's
`fixed_resolution: false` each revolution yields a different sample count
(observed 2853-2858). The map therefore builds from the first scan or two and
then silently stops updating forever.

The driver's own `fixed_resolution: true` does produce a constant count, but its
re-binning silently DISCARDS about a third of the scan: measured on the robot,
coverage fell from 96.4% valid across all bearings to 63.8%, with a contiguous
110-130 degree dead arc centred on the sensor's 0 degree mark. That is far worse
than the problem it solves.

So: leave the driver alone (full data, variable count) and resample here onto a
fixed grid. Every output bin takes the MINIMUM valid range of the input samples
falling inside it — conservative, since under-reporting an obstacle's distance is
the safe direction. Output resolution is deliberately coarser than the input so
that every bin receives several samples and holes cannot appear.

Publishes a constant-width scan that Karto accepts, without throwing data away.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


class ScanResampler(Node):
    def __init__(self):
        super().__init__("scan_resampler")

        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_fixed")
        # 720 bins = 0.5 deg. The X3 delivers ~2855 samples/rev, so each output bin
        # gets ~4 input samples and empty bins are effectively impossible. At 4 m
        # range 0.5 deg is 3.5 cm, finer than the 5 cm SLAM map cells, so nothing
        # useful is lost by being coarser than the raw scan.
        self.declare_parameter("output_beams", 720)

        self.in_topic = self.get_parameter("input_topic").value
        self.out_topic = self.get_parameter("output_topic").value
        self.n_out = int(self.get_parameter("output_beams").value)

        self.out_min = -math.pi
        self.out_inc = (2.0 * math.pi) / self.n_out

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(LaserScan, self.out_topic, qos)
        self.sub = self.create_subscription(LaserScan, self.in_topic, self.cb, qos)

        self._logged = False
        self._empty_warned = False
        self.get_logger().info(
            f"scan_resampler: {self.in_topic} -> {self.out_topic}, "
            f"{self.n_out} beams ({math.degrees(self.out_inc):.2f} deg)")

    def cb(self, msg: LaserScan):
        n_in = len(msg.ranges)
        if n_in == 0:
            return

        rng = np.asarray(msg.ranges, dtype=np.float32)
        ang = msg.angle_min + np.arange(n_in, dtype=np.float32) * msg.angle_increment

        # A range is usable only if finite and inside the sensor's own limits.
        valid = np.isfinite(rng) & (rng >= msg.range_min) & (rng <= msg.range_max)

        # Wrap into [-pi, pi) then bin. Wrapping matters: the raw scan spans
        # -180..180 and the endpoints must not fall outside the grid.
        a = np.mod(ang[valid] + math.pi, 2.0 * math.pi) - math.pi
        idx = np.floor((a - self.out_min) / self.out_inc).astype(np.int32)
        np.clip(idx, 0, self.n_out - 1, out=idx)

        out = np.full(self.n_out, np.inf, dtype=np.float32)
        np.minimum.at(out, idx, rng[valid])

        # Bins that received no sample: mark invalid using the driver's own
        # convention (0.0, since invalid_range_is_inf is false), which every
        # consumer here already skips via its own minimum-range gate.
        empty = ~np.isfinite(out)
        out[empty] = 0.0

        o = LaserScan()
        # Keep the ORIGINAL stamp. Restamping with "now" would desynchronise the
        # scan from the odom->laser TF and quietly corrupt scan matching.
        o.header.stamp = msg.header.stamp
        o.header.frame_id = msg.header.frame_id
        o.angle_min = float(self.out_min)
        o.angle_max = float(self.out_min + (self.n_out - 1) * self.out_inc)
        o.angle_increment = float(self.out_inc)
        o.time_increment = float(msg.time_increment * n_in / self.n_out)
        o.scan_time = msg.scan_time
        o.range_min = msg.range_min
        o.range_max = msg.range_max
        o.ranges = out.tolist()
        o.intensities = []
        self.pub.publish(o)

        n_empty = int(empty.sum())
        if not self._logged:
            self._logged = True
            self.get_logger().info(
                f"scan_resampler: first scan {n_in} -> {self.n_out} beams, "
                f"{int(valid.sum())} valid in, {n_empty} empty bins out")
        if n_empty > self.n_out // 10 and not self._empty_warned:
            self._empty_warned = True
            self.get_logger().warn(
                f"scan_resampler: {n_empty}/{self.n_out} output bins empty — "
                f"consider lowering output_beams")


def main():
    rclpy.init()
    node = ScanResampler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
