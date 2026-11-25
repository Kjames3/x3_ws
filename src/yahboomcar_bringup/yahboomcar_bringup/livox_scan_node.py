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
import time

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

from yahboomcar_bringup.person_candidates import (FIELDS as CANDIDATE_FIELDS, extract as extract_candidates,
                                                  seeded_candidates, ray_candidates, RAY_IN_FIELDS)

# Side cameras (X3_SIDE_PERSONS in x3_server): role -> index in server_x3.CAMERA_ROLES.
SIDE_CAMERAS = (('left', 2), ('right', 3), ('rear', 1))  # rear: X3_REAR_PERSONS
# A ray is searched on every Livox frame (10 Hz) until it is this old.  The cameras detect at
# 3-5 Hz, so 0.3 s left frames with no ray; a body only the ray finds (crouched among
# furniture, 2026-11-23 rear test) then missed the tracker's 0.4 s lidar gap and was reborn
# under a new id every ~2 s.  The body XY still comes from the current frame.
SIDE_RAY_MAX_AGE_S = 0.6
# /livox/person_candidates_side rows: the body's CANDIDATE_FIELDS, the anchor XY (the body,
# or the ordinary candidate it duplicates), the camera index, its per-camera person id and
# the image age at this frame's end (s).
SIDE_FIELDS = CANDIDATE_FIELDS + ('ax', 'ay', 'cam', 'tid', 'age')

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


# Display cloud (/surroundings/points): 2.5 cm cells within NEAR_RADIUS of the
# robot (horizontal), 5 cm beyond, at most DISPLAY_CAP points. The browser
# re-voxelizes this at 2.5/5, 5 or 10 cm and uses the same radius.
NEAR_RADIUS = 3.0
NEAR_CELL, FAR_CELL = 0.025, 0.05
DISPLAY_CAP = 12000  # 12 B/pt at 5 Hz: ~700 KB/s, 2x the old 6000 cap


def _first_per_cell(xyz, cell):
    if not len(xyz):
        return xyz
    _, idx = np.unique(np.floor(xyz / cell).astype(np.int32), axis=0, return_index=True)
    return xyz[np.sort(idx)]


def display_voxels(xyz, near_radius=NEAR_RADIUS, cap=DISPLAY_CAP):
    """One point per 2.5 cm cell near the robot and per 5 cm cell beyond.

    Over the cap, the far shell is thinned first but keeps at least a third of
    the budget, so distant walls never vanish behind a cluttered near field.
    """
    xyz = np.asarray(xyz)
    near_mask = np.hypot(xyz[:, 0], xyz[:, 1]) < near_radius
    near = _first_per_cell(xyz[near_mask], NEAR_CELL)
    far = _first_per_cell(xyz[~near_mask], FAR_CELL)
    if len(near) + len(far) > cap:
        far_keep = min(len(far), max(cap - len(near), cap // 3))
        near_keep = min(len(near), cap - far_keep)
        far = far[np.linspace(0, len(far) - 1, far_keep, dtype=int)] if far_keep else far[:0]
        near = near[np.linspace(0, len(near) - 1, near_keep, dtype=int)] if near_keep else near[:0]
    return np.concatenate((near, far)) if len(far) else near


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
        # Point budget of /surroundings/points; x3_server raises it in its
        # "high" stream profile (a fast local link, e.g. the Slate router).
        self.display_cap = int(self.declare_parameter('display_cap', DISPLAY_CAP).value)
        # Split one cluster of several close-standing people by their heads
        # (person_candidates._peak_split). On 2026-11-24 lab crowd runs: crowd-r2 lidar ids
        # 14 -> 6, losses 11 -> 5; empty runs still 0 tracks. False = the old behaviour.
        self.person_peaks = bool(self.declare_parameter('person_peaks', True).value)
        self.add_on_set_parameters_callback(self._on_params)
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
        # Low-band obstacles (/livox/obstacles): everything the 0.26-0.36 m
        # slice misses below it (table/truss base bars, chair star bases) and up
        # to the robot's height.  Heights are above base_footprint (the floor).
        # Lab carpet floor fit (2026-11-22, parked): floor returns peak at
        # z -0.02..0 AND 0.02..0.05 out to 4.5 m, so 0.06 m plus a range slope
        # for pitch error.  Bars under ~6 cm are not caught.
        self.declare_parameter('obstacle_z_min', 0.06)
        self.declare_parameter('obstacle_z_slope', 0.015)   # m per m beyond 2 m
        self.declare_parameter('obstacle_z_max', 0.45)
        self.declare_parameter('obstacle_range_max', 4.0)
        self.declare_parameter('obstacle_cell', 0.05)
        # Chassis box (Design_v2 extent +-0.32 x +-0.293) plus margin: no self-returns.
        self.declare_parameter('obstacle_memory_s', 20.0)
        self.declare_parameter('min_elevation_deg', 7.0)   # spec 7.5; 7 forgets later
        self.declare_parameter('self_x', 0.37)
        self.declare_parameter('self_y', 0.34)

        gp = lambda n: self.get_parameter(n).value
        self.src_frame, self.scan_frame = gp('source_frame'), gp('scan_frame')
        self.z_min, self.z_max = float(gp('z_min')), float(gp('z_max'))
        self.r_min, self.r_max = float(gp('range_min')), float(gp('range_max'))
        self.beams = int(gp('beams'))
        self.merge = max(1, int(gp('merge_frames')))
        self.deskew = bool(gp('deskew'))
        self.inc = 2 * math.pi / self.beams
        self.ob_zmin, self.ob_zmax = float(gp('obstacle_z_min')), float(gp('obstacle_z_max'))
        self.ob_zslope = float(gp('obstacle_z_slope'))
        self.ob_rmax, self.ob_cell = float(gp('obstacle_range_max')), float(gp('obstacle_cell'))
        self.self_x, self.self_y = float(gp('self_x')), float(gp('self_y'))
        self.ob_memory = {}
        self.ob_memory_s = float(gp('obstacle_memory_s'))
        self.min_elev_tan = math.tan(math.radians(float(gp('min_elevation_deg'))))
        self.ob_pub = self.create_publisher(PointCloud2, '/livox/obstacles', qos_profile_sensor_data)

        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.cloud_R = self.cloud_t = None
        self.cloud_last_ns = 0
        self.pose_history = deque(maxlen=200)
        self.cloud_pose_pub = self.create_publisher(PoseStamped, "/surroundings/pose", qos_profile_sensor_data)
        self.cloud_pub = self.create_publisher(PointCloud2, "/surroundings/points", qos_profile_sensor_data)
        # Person-sized clusters for the camera-seeded lidar person tracker (server_x3).
        self.cand_pub = self.create_publisher(PointCloud2, "/livox/person_candidates", qos_profile_sensor_data)
        # Camera-seeded candidates (shadow mode, 2026-11-22): x3_server sends C3 people
        # (odom XY + velocity) on /c3/person_seeds; bodies cut out along each camera
        # ray go out on their own topic, so /livox/person_candidates is unchanged.
        self.seed_pub = self.create_publisher(PointCloud2, "/livox/person_candidates_seeded",
                                              qos_profile_sensor_data)
        self.seeds = None            # (stamp_s, (K, 4) x, y, vx, vy)
        self.create_subscription(PointCloud2, "/c3/person_seeds", self.on_seeds, 5)
        # Bearing-only side-camera rays (/side/<role>/person_rays, one topic per camera so
        # an empty message clears that camera's people): bodies found along each ray go
        # out on /livox/person_candidates_side, before /livox/person_candidates.
        self.side_pub = self.create_publisher(PointCloud2, "/livox/person_candidates_side",
                                              qos_profile_sensor_data)
        self.side_rays = {}          # camera index -> (stamp_s, (K, 9) RAY_IN_FIELDS + tid)
        for role, k in SIDE_CAMERAS:
            self.create_subscription(PointCloud2, f"/side/{role}/person_rays",
                                     lambda m, k=k: self.on_side_rays(k, m), 5)
        self.cand_ms = []
        self.cand_n = []
        self.cand_logged_ns = 0
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

    def _on_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'display_cap':
                if int(p.value) < 1000:
                    return SetParametersResult(successful=False, reason='display_cap < 1000')
                self.display_cap = int(p.value)
        return SetParametersResult(successful=True)

    def publish_surroundings(self, pts, data):
        """Bounded display-only cloud: 5 Hz, 2.5 cm voxels within 3 m and 5 cm
        beyond, <=12000 XYZ points (see display_voxels).

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
            xyz = display_voxels(xyz, cap=self.display_cap)
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
        try:
            self.publish_low_obstacles(pts, data)
        except Exception as e:
            self.get_logger().warn(f'low obstacles skipped: {e}', throttle_duration_sec=5.0)
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
        try:
            self.publish_person_candidates(pts, data)
        except Exception as e:
            # Runs after /scan is out, and must never stop the next one.
            self.get_logger().warn(f'person candidates skipped: {e}', throttle_duration_sec=5.0)

    def publish_low_obstacles(self, pts, data):
        """Points obstacle_z_min..obstacle_z_max above the floor -> /livox/obstacles (odom).

        The lidar's lowest ray is -7.5 deg, so an object h tall drops out of view
        inside ~(sensor_z - h) / tan(7.5 deg) (a 5 cm bar at ~2 m), and /scan
        raytracing would then clear it from the costmap.  So voxels are REMEMBERED
        in odom and only forgotten when their spot is back in view and empty,
        when they are older than obstacle_memory_s, or farther than range_max + 1.
        Use as a mark-only costmap source with observation_persistence 0.
        """
        if self.cloud_R is None or self.ob_pub.get_subscription_count() == 0:
            return
        sec, nsec = struct.unpack_from('<iI', data, 4)
        t_end = int(pts['t'].max())
        end = sec + nsec * 1e-9 + t_end * 1e-9
        pose = self.pose_at(np.array([end]))
        if pose is None:
            return
        fx, fy, fa = (float(v[0]) for v in pose)
        xyz = np.column_stack((pts['x'], pts['y'], pts['z'])).astype(np.float64)
        ok = np.isfinite(xyz).all(axis=1) & ((xyz[:, 0] != 0) | (xyz[:, 1] != 0))
        xyz, toff = xyz[ok], pts['t'][ok]
        if self.deskew and abs(self.gyro_z) > 1e-3:
            a = -self.gyro_z * (t_end - toff.astype(np.int64)) * 1e-9
            c, s = np.cos(a), np.sin(a)
            xyz[:, 0], xyz[:, 1] = c * xyz[:, 0] - s * xyz[:, 1], s * xyz[:, 0] + c * xyz[:, 1]
        xyz = xyz @ self.cloud_R.T + self.cloud_t
        r = np.hypot(xyz[:, 0], xyz[:, 1])
        zmin = self.ob_zmin + self.ob_zslope * np.maximum(r - 2.0, 0.0)
        keep = ((xyz[:, 2] > zmin) & (xyz[:, 2] < self.ob_zmax) & (r < self.ob_rmax)
                & ~((np.abs(xyz[:, 0]) < self.self_x) & (np.abs(xyz[:, 1]) < self.self_y)))
        cur = _first_per_cell(xyz[keep], self.ob_cell)
        c, sn = math.cos(fa), math.sin(fa)
        wx = c * cur[:, 0] - sn * cur[:, 1] + fx
        wy = sn * cur[:, 0] + c * cur[:, 1] + fy
        cell = self.ob_cell
        seen = set()
        for x, y, z in zip(wx, wy, cur[:, 2]):
            k = (int(math.floor(x / cell)), int(math.floor(y / cell)), int(math.floor(z / cell)))
            seen.add(k)
            self.ob_memory[k] = (x, y, z, end)
        # Forget what is visible-and-empty, stale, or far away.
        sz = float(self.cloud_t[2])
        for k, (x, y, z, t) in list(self.ob_memory.items()):
            if k in seen:
                continue
            dx, dy = x - fx, y - fy
            d = math.hypot(dx, dy)
            visible = (d >= (sz - z) / self.min_elev_tan + 0.15) and d < self.ob_rmax
            if visible or end - t > self.ob_memory_s or d > self.ob_rmax + 1.0:
                del self.ob_memory[k]
        out = np.array([v[:3] for v in self.ob_memory.values()], dtype='<f4').reshape(-1, 3)
        msg = PointCloud2()
        stamp_ns = round(end * 1_000_000_000)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        msg.header.frame_id = 'odom'
        msg.height, msg.width = 1, len(out)
        msg.fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                      for i, name in enumerate(('x', 'y', 'z'))]
        msg.is_bigendian, msg.is_dense = False, True
        msg.point_step, msg.row_step = 12, len(out) * 12
        msg.data = out.tobytes()
        self.ob_pub.publish(msg)

    def on_seeds(self, msg):
        rows = np.frombuffer(bytes(msg.data), dtype='<f4').reshape(-1, 4) if msg.width else np.zeros((0, 4))
        self.seeds = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, rows.astype(np.float64))

    def seeds_at(self, t):
        """Camera seeds predicted to t, or None when stale (> 0.3 s) or empty."""
        if self.seeds is None:
            return None
        st, rows = self.seeds
        dt = t - st
        if not len(rows) or abs(dt) > 0.3:
            return None
        return rows[:, :2] + rows[:, 2:4] * max(dt, 0.0)

    def on_side_rays(self, cam, msg):
        n = len(RAY_IN_FIELDS) + 1
        rows = (np.frombuffer(bytes(msg.data), dtype='<f4').reshape(-1, n) if msg.width
                else np.zeros((0, n), dtype=np.float32))
        self.side_rays[cam] = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                               rows.astype(np.float64))

    def side_rays_at(self, t):
        """(rays (R, 8), meta (R, 3) cam, tid, age) of the cameras' fresh rays, or None."""
        rays, meta = [], []
        for cam, (st, rows) in self.side_rays.items():
            age = t - st
            if not len(rows) or age > SIDE_RAY_MAX_AGE_S or age < -0.15:
                continue
            rays.append(rows[:, :-1])
            meta.append(np.c_[np.full(len(rows), cam), rows[:, -1], np.full(len(rows), age)])
        if not rays:
            return None
        return np.vstack(rays), np.vstack(meta)

    def publish_person_candidates(self, pts, data):
        """Person-sized clusters of the full frame -> /livox/person_candidates.

        Every point is placed in odom with the robot pose at its own capture time
        (the same de-skew as the display cloud), so the tracker downstream works in
        one fixed frame.  Fields CANDIDATE_FIELDS, one row per candidate, stamped
        at the frame end.  Skipped without odom: an unplaced cloud cannot be
        tracked.  Shapes only; nothing here says a cluster is a person.
        """
        if self.cand_pub.get_subscription_count() == 0 or self.cloud_R is None:
            return
        t0 = time.perf_counter()
        xyz = np.column_stack((pts['x'], pts['y'], pts['z'])).astype(np.float64)
        ok = np.isfinite(xyz).all(axis=1) & (np.abs(xyz).sum(axis=1) > 0)
        xyz = xyz[ok] @ self.cloud_R.T + self.cloud_t
        keep = (xyz[:, 2] > 0.1) & (xyz[:, 2] < 2.3)
        xyz, toff = xyz[keep], pts['t'][ok][keep]
        sec, nsec = struct.unpack_from('<iI', data, 4)
        start = sec + nsec * 1e-9
        end = start + (int(pts['t'].max()) * 1e-9 if len(pts) else 0.0)
        frame = self.pose_at(np.array([end]))
        poses = self.pose_at(start + toff.astype(np.float64) * 1e-9) if len(xyz) else None
        if frame is None or poses is None:
            return
        px, py, pa = poses
        c, sn = np.cos(pa), np.sin(pa)
        wx = c * xyz[:, 0] - sn * xyz[:, 1] + px
        wy = sn * xyz[:, 0] + c * xyz[:, 1] + py
        origin = (float(frame[0][0]), float(frame[1][0]))
        cands = extract_candidates(wx, wy, xyz[:, 2], origin=origin, peaks=self.person_peaks)
        seeds = self.seeds_at(end)
        if seeds is not None and self.seed_pub.get_subscription_count() > 0:
            more = seeded_candidates(wx, wy, xyz[:, 2], seeds, origin, existing=cands)
            self.seed_pub.publish(self._candidate_msg(more, end))
        rays = self.side_rays_at(end)
        if rays is not None and self.side_pub.get_subscription_count() > 0:
            found = ray_candidates(wx, wy, xyz[:, 2], rays[0], existing=cands)
            meta = rays[1][found[:, -1].astype(int)]
            out = np.column_stack((found[:, :-1], meta)).astype(np.float32)
            self.side_pub.publish(self._candidate_msg(out, end, SIDE_FIELDS))
        self.cand_pub.publish(self._candidate_msg(cands, end))
        self.cand_ms.append((time.perf_counter() - t0) * 1e3)
        self.cand_n.append(len(cands))
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.cand_logged_ns > 60_000_000_000:
            self.cand_logged_ns = now_ns
            self.get_logger().info(
                f'person candidates: {np.mean(self.cand_n):.1f}/frame, '
                f'{np.mean(self.cand_ms):.1f} ms avg, {np.max(self.cand_ms):.1f} ms max '
                f'over {len(self.cand_ms)} frames')
            self.cand_ms, self.cand_n = [], []


    @staticmethod
    def _candidate_msg(cands, end, fields=None):
        """CANDIDATE_FIELDS (or `fields`) rows -> PointCloud2 in odom, stamped at the frame end."""
        fields = CANDIDATE_FIELDS if fields is None else fields
        msg = PointCloud2()
        stamp_ns = round(end * 1_000_000_000)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        msg.header.frame_id = 'odom'
        msg.height, msg.width = 1, len(cands)
        msg.fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                      for i, name in enumerate(fields)]
        msg.is_bigendian, msg.is_dense = False, True
        msg.point_step = 4 * len(fields)
        msg.row_step = msg.point_step * len(cands)
        msg.data = np.ascontiguousarray(cands, dtype='<f4').tobytes()
        return msg


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
