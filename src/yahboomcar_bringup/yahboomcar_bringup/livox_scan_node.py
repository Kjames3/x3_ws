#!/usr/bin/env python3
"""Slice the Livox Mid-360S cloud into the 2D /scan the rest of the stack uses.

The YDLidar was replaced by the Livox (2026-10-29) at nearly the same height
(optical centre 0.31 m vs the old scan plane's 0.3185 m), so a thin height
slice around the sensor reproduces the old scan and keeps the existing maps,
AMCL, slam_toolbox and the CBF working unchanged.  The scan is published in
laser_link, the OLD lidar frame (180 deg + 4.85 deg yaw), because
server_x3._scan_cb, the auto-site check and the maps all hard-code that
convention.  The livox_frame -> laser_link transform comes from TF.

    /livox/lidar (CustomMsg) + /odom yaw rate  ->  /scan and /scan_fixed (720 beams)

/scan_fixed is the constant-beam-count topic AMCL reads; the slice already has
a fixed beam count, so it is the same message (scan_resampler is not needed).

CustomMsg is decoded straight from the CDR bytes: rclpy's deserializer takes
~177 ms for one 20k-point message on the Orin, the numpy view 0.3 ms.  Points
are the message's last field, 20-byte stride (19 bytes + alignment padding).

Each point is de-skewed for yaw with the /odom yaw rate (the frame spans
100 ms; at 0.6 rad/s that is 3.4 deg).  Not /livox/imu: its 200 Hz callbacks
cost ~20% of a core in rclpy executor overhead alone.  And the previous frame is merged in (min range
per bin) because one frame of the non-repetitive pattern fills only ~91-97%
of 0.5-1 deg bins.
"""
import math
import struct

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from collections import deque
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from tf2_ros import Buffer, TransformListener

from livox_ros_driver2.msg import CustomMsg

POINT_DT = np.dtype({'names': ['t', 'x', 'y', 'z'],
                     'formats': ['<u4', '<f4', '<f4', '<f4'],
                     'offsets': [0, 4, 8, 12], 'itemsize': 20})


def decode_points(data):
    """CustomMsg CDR bytes -> structured array (offset_time ns, x, y, z)."""
    n = (len(data) + 1) // 20
    for n in range(n, max(n - 200, 0), -1):
        start = len(data) - (n * 20 - 1)
        if start >= 4 and struct.unpack_from('<I', data, start - 4)[0] == n:
            return np.frombuffer(bytes(data[start:]) + b'\0', dtype=POINT_DT, count=n)
    return None


def quat_to_mat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class LivoxScanNode(Node):
    def __init__(self):
        super().__init__('livox_scan_node')
        self.declare_parameter('cloud_topic', '/livox/lidar')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('source_frame', 'livox_frame')
        self.declare_parameter('scan_frame', 'laser_link')
        # Slice in the SENSOR frame (z up).  +-0.05 m is 0.26-0.36 m above the
        # floor; the floor itself is only seen beyond ~2.3 m and ~0.31 m down.
        self.declare_parameter('z_min', -0.05)
        self.declare_parameter('z_max', 0.05)
        self.declare_parameter('range_min', 0.12)
        self.declare_parameter('range_max', 12.0)
        self.declare_parameter('beams', 720)
        self.declare_parameter('merge_frames', 2)
        self.declare_parameter('deskew', True)

        gp = lambda n: self.get_parameter(n).value
        self.src_frame, self.scan_frame = gp('source_frame'), gp('scan_frame')
        self.z_min, self.z_max = float(gp('z_min')), float(gp('z_max'))
        self.r_min, self.r_max = float(gp('range_min')), float(gp('range_max'))
        self.beams = int(gp('beams'))
        self.merge = max(1, int(gp('merge_frames')))
        self.deskew = bool(gp('deskew'))
        self.inc = 2 * math.pi / self.beams

        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.cloud_R = self.cloud_t = None
        self.cloud_last_ns = 0
        self.pose_history = deque(maxlen=200)
        self.cloud_pose_pub = self.create_publisher(PoseStamped, "/surroundings/pose", qos_profile_sensor_data)
        self.cloud_pub = self.create_publisher(PointCloud2, "/surroundings/points", qos_profile_sensor_data)
        self.R = None                  # livox_frame -> scan_frame rotation
        self.t = None
        self.gyro_z = 0.0
        self.history = []

        self.pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.pub_fixed = self.create_publisher(LaserScan, '/scan_fixed', qos_profile_sensor_data)
        self.create_subscription(Odometry, gp('odom_topic'), self.on_odom, 10)
        self.create_subscription(CustomMsg, gp('cloud_topic'), self.on_cloud,
                                 qos_profile_sensor_data, raw=True)
        self.get_logger().info(
            f'slicing {gp("cloud_topic")} z {self.z_min:+.2f}..{self.z_max:+.2f} m '
            f'-> /scan, /scan_fixed in {self.scan_frame} ({self.beams} beams)')

    def on_odom(self, msg):
        self.gyro_z = msg.twist.twist.angular.z
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_footprint':
            return
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.pose_history and stamp <= self.pose_history[-1][0]:
            self.pose_history.clear()
        p, v = msg.pose.pose.position, msg.twist.twist.linear
        self.pose_history.append((stamp, p.x, p.y, yaw, v.x, v.y, self.gyro_z))

    def pose_at(self, stamps):
        """Interpolate acquisition-time odom; extrapolate at most 80 ms.

        Reject older/out-of-sync data rather than accumulating it at a wrong pose.
        """
        if len(self.pose_history) < 2:
            return None
        h = np.asarray(self.pose_history)
        if np.min(stamps) < h[0, 0] or np.max(stamps) > h[-1, 0] + .08:
            return None
        yaw = np.unwrap(h[:, 3])
        x, y = np.interp(stamps, h[:, 0], h[:, 1]), np.interp(stamps, h[:, 0], h[:, 2])
        a = np.interp(stamps, h[:, 0], yaw)
        dt = np.maximum(0, stamps - h[-1, 0])
        c, sn = math.cos(yaw[-1]), math.sin(yaw[-1])
        x += dt * (c * h[-1, 4] - sn * h[-1, 5])
        y += dt * (sn * h[-1, 4] + c * h[-1, 5])
        a += dt * h[-1, 6]
        return x, y, a

    def lookup(self):
        try:
            tf = self.tf_buf.lookup_transform(self.scan_frame, self.src_frame, Time())
            cloud_tf = self.tf_buf.lookup_transform("base_footprint", self.src_frame, Time())
        except Exception as e:
            self.get_logger().warn(f'waiting for TF {self.src_frame} -> {self.scan_frame}: {e}',
                                   throttle_duration_sec=5.0)
            return False
        self.cloud_R = quat_to_mat(cloud_tf.transform.rotation)
        ct = cloud_tf.transform.translation
        self.cloud_t = np.array([ct.x, ct.y, ct.z])
        tr = tf.transform.translation
        self.R = quat_to_mat(tf.transform.rotation)
        self.t = np.array([tr.x, tr.y, tr.z])
        self.get_logger().info(f'{self.src_frame} -> {self.scan_frame}: t={np.round(self.t, 4)}')
        # Both frames are URDF-fixed.  Keeping the listener would deserialize
        # /tf at 50 Hz in Python for nothing (~1/3 of this node's CPU).
        self.tf_listener.unregister()
        self.tf_listener = None
        return True

    def publish_surroundings(self, pts, data):
        """Bounded display-only cloud: 5 Hz, 5 cm voxels, <=6000 XYZ points.

        Acquisition-time odometry de-skews the display cloud when available.
        Matching /surroundings/pose enables bounded browser-side memory.
        Never feeds the scan, navigation, or safety controller.
        """
        now_ns = self.get_clock().now().nanoseconds
        if self.cloud_pub.get_subscription_count() == 0 or now_ns - self.cloud_last_ns < 200_000_000:
            return
        self.cloud_last_ns = now_ns
        xyz = np.column_stack((pts['x'], pts['y'], pts['z']))
        valid = np.isfinite(xyz).all(axis=1)
        radius = np.linalg.norm(xyz, axis=1)
        keep = valid & (radius > 0.12) & (radius < 6.0)
        xyz = xyz[keep]
        xyz = xyz @ self.cloud_R.T + self.cloud_t
        sec, nsec = struct.unpack_from('<iI', data, 4)
        start = sec + nsec * 1e-9
        end = start + (int(pts['t'].max()) * 1e-9 if len(pts) else 0)
        frame_pose = self.pose_at(np.array([end]))
        if frame_pose is not None:
            point_poses = self.pose_at(start + pts['t'][keep].astype(np.float64) * 1e-9) if len(xyz) else None
            if point_poses is None:
                frame_pose = None
            else:
                # Full planar de-skew including the sensor's offset from base.
                px, py, pa = point_poses
                c, sn = np.cos(pa), np.sin(pa)
                wx = c * xyz[:, 0] - sn * xyz[:, 1] + px
                wy = sn * xyz[:, 0] + c * xyz[:, 1] + py
                fx, fy, fa = (float(v[0]) for v in frame_pose)
                c, sn = math.cos(fa), math.sin(fa)
                xyz[:, 0] = c * (wx - fx) + sn * (wy - fy)
                xyz[:, 1] = -sn * (wx - fx) + c * (wy - fy)
        if len(xyz):
            _, indices = np.unique(np.floor(xyz / 0.05).astype(np.int32), axis=0, return_index=True)
            xyz = xyz[indices]
            if len(xyz) > 6000:
                xyz = xyz[np.linspace(0, len(xyz) - 1, 6000, dtype=int)]
        xyz = np.asarray(xyz, dtype='<f4')
        msg = PointCloud2()
        sec, nsec = struct.unpack_from('<iI', data, 4)
        msg.header.stamp.sec, msg.header.stamp.nanosec = sec, nsec
        msg.header.frame_id = 'base_footprint'
        if frame_pose is not None:
            stamp_ns = round(end * 1_000_000_000)
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
            pose = PoseStamped()
            pose.header.stamp = msg.header.stamp
            pose.header.frame_id = 'odom'
            pose.pose.position.x, pose.pose.position.y = fx, fy
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(fa / 2), math.cos(fa / 2)
            self.cloud_pose_pub.publish(pose)
        msg.height, msg.width = 1, len(xyz)
        msg.fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                      for i, name in enumerate(('x', 'y', 'z'))]
        msg.is_bigendian, msg.is_dense = False, True
        msg.point_step, msg.row_step = 12, len(xyz) * 12
        msg.data = xyz.tobytes()
        self.cloud_pub.publish(msg)

    def on_cloud(self, data):
        if self.R is None and not self.lookup():
            return
        pts = decode_points(data)
        if pts is None or len(pts) == 0:
            return
        try:
            self.publish_surroundings(pts, data)
        except Exception as e:
            # A visualization failure must never stop the safety scan publisher.
            self.get_logger().warn(f'surroundings display skipped: {e}', throttle_duration_sec=5.0)
        x = pts['x'].astype(np.float64)
        y = pts['y'].astype(np.float64)
        z = pts['z'].astype(np.float64)
        keep = (z > self.z_min) & (z < self.z_max) & ((x != 0) | (y != 0))
        x, y, z, toff = x[keep], y[keep], z[keep], pts['t'][keep]
        if len(x) == 0:
            return
        t_end = int(pts['t'].max())
        if self.deskew and abs(self.gyro_z) > 1e-3:
            # Rotate each point into the sensor pose at the END of the frame.
            a = -self.gyro_z * (t_end - toff.astype(np.int64)) * 1e-9
            c, s = np.cos(a), np.sin(a)
            x, y = c * x - s * y, s * x + c * y
        p = self.R @ np.vstack((x, y, z)) + self.t[:, None]
        r = np.hypot(p[0], p[1])
        ok = (r > self.r_min) & (r < self.r_max)
        idx = ((np.arctan2(p[1][ok], p[0][ok]) + math.pi) / self.inc).astype(np.int64) % self.beams
        ranges = np.full(self.beams, np.inf)
        np.minimum.at(ranges, idx, r[ok])

        self.history.append(ranges)
        self.history = self.history[-self.merge:]
        merged = np.minimum.reduce(self.history) if len(self.history) > 1 else ranges

        # header.stamp sits at the start of the CDR header: sec int32, nanosec uint32
        sec, nsec = struct.unpack_from('<iI', data, 4)
        stamp_ns = sec * 1_000_000_000 + nsec + t_end
        msg = LaserScan()
        msg.header.frame_id = self.scan_frame
        msg.header.stamp.sec = stamp_ns // 1_000_000_000
        msg.header.stamp.nanosec = stamp_ns % 1_000_000_000
        msg.angle_min = -math.pi
        msg.angle_max = math.pi - self.inc
        msg.angle_increment = self.inc
        msg.time_increment = 0.0          # de-skewed to the frame end
        msg.scan_time = 0.1
        msg.range_min = self.r_min
        msg.range_max = self.r_max
        msg.ranges = merged.astype(np.float32).tolist()
        self.pub.publish(msg)
        self.pub_fixed.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LivoxScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
