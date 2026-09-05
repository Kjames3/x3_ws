#!/usr/bin/env python3
"""lidar_3d_processor_node — split the tilting lidar's scans two ways.

    /scan_raw  ──┬─→ /scan            2D SLAM / Nav2, ONLY while level
                 └─→ /pointcloud_raw  3D projection for octomap_server

The gate matters: slam_toolbox and AMCL both assume a horizontal scan plane, so
feeding them a tilted scan silently corrupts the map and the pose.  The gate is
on |tilt|. The Dynamixel calibration uses direction +1 (full-frame verification 2026-09-04).

Tilt comes from lidar_tilt_node on /lidar_tilt/joint_states, NOT from the
merged /joint_states — joint_state_publisher happily publishes
`lidar_tilt_joint: 0.0` when nothing else does, which would look like a fresh,
level reading forever.  If no authoritative source is heard within
`tilt_timeout`, behaviour depends on whether one was EVER heard:

  * heard, then went silent -> the last known angle is LATCHED.  A mount that
    stops reporting has not moved back to level; it has almost always stopped
    reporting *while tilted* (server_x3.py only publishes a JointState inside
    `if pos is not None`, so a run of failed LX-16A serial reads produces
    exactly this).  Assuming level here would un-gate /scan and hand
    slam_toolbox and AMCL a 40-degree-tilted scan plane labelled horizontal,
    silently corrupting both the 2D map and the pose.  3D clouds are also
    withheld while stale, because the true angle is no longer known.
  * never heard at all -> `assume_level_if_no_source` (default FALSE).
"""

import math
import time
from collections import deque
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, JointState, PointCloud, PointField
from std_msgs.msg import Bool
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from .lidar_deskew import interpolate_pitch, deskew


class Lidar3dProcessorNode(Node):

    def __init__(self):
        super().__init__('lidar_3d_processor_node')

        self.declare_parameter('scan_in_topic', '/scan_raw')
        self.declare_parameter('scan_out_topic', '/scan')
        self.declare_parameter('cloud_out_topic', '/pointcloud_raw')
        self.declare_parameter('tilt_topic', '/lidar_tilt/joint_states')
        self.declare_parameter('joint_name', 'lidar_tilt_joint')
        # ~2.9 deg.  At 8 m that is 0.40 m of height error, which slam_toolbox
        # will absorb as range noise; much beyond this and walls start to bend.
        self.declare_parameter('level_threshold_rad', 0.05)
        self.declare_parameter('tilt_timeout_s', 2.0)
        # Applies ONLY when a tilt source has never been heard at all.  Once a
        # source has been seen and then goes silent, the last known angle is
        # latched instead (see _tilt_state) -- assuming level there would
        # un-gate /scan with the mount still pitched over.  Default false: a
        # workspace that can sweep must not guess "level" on silence.
        self.declare_parameter('assume_level_if_no_source', False)
        # Drop clouds captured while the mount is still travelling: a
        # step-and-stare sweep is only distortion-free if the moving part of
        # each step is discarded.
        self.declare_parameter('require_settled', True)
        # Chassis self-returns.  Masked ONLY when tilted; never on /scan.
        # The laser sits at z=0.340 m above base_footprint -- MEASURED via TF
        # 2026-08-23, a level scan projecting to a flat plane at +0.340 across
        # 81872 points.  (0.191 m appears in older comments here and elsewhere;
        # that is the pre-tilting-mount height and is stale.)  At the steepest
        # sweep angle of 45 deg the floor intercept is at slant range
        # 0.34/sin45 = 0.481 m, comfortably outside this mask -- the mask only
        # starts eating floor beyond 76 deg of tilt, which the sweep never
        # reaches.  So this does NOT cost near-floor coverage.
        self.declare_parameter('tilted_min_range_m', 0.35)
        # 6 m: 5/6/8 m measured identical OctoMap CPU at the continuous
        # cloud rate, and only 4 m lost endpoints (82.8% kept).  See
        # params/lidar_3d_processor.yaml for the table.
        self.declare_parameter('cloud_max_range_m', 6.0)
        self.declare_parameter('publish_cloud_when_level', True)

        self.joint_name = self.get_parameter('joint_name').value
        self.level_threshold = float(
            self.get_parameter('level_threshold_rad').value)
        self.tilt_timeout = float(self.get_parameter('tilt_timeout_s').value)
        self.assume_level = bool(
            self.get_parameter('assume_level_if_no_source').value)
        self.tilted_min_range = float(
            self.get_parameter('tilted_min_range_m').value)
        self.cloud_max_range = float(
            self.get_parameter('cloud_max_range_m').value)
        self.publish_cloud_when_level = bool(
            self.get_parameter('publish_cloud_when_level').value)

        self.declare_parameter('timed_cloud_topic', '/lidar/points_timed')
        self.declare_parameter('deskew_max_gap_s', 0.12)
        self.declare_parameter('deskew_wait_s', 0.3)
        self.declare_parameter('deskew_mount_frame', 'lidar_mount_link')
        self.declare_parameter('deskew_tilt_frame', 'lidar_tilt_link')
        self.history = deque()
        self.pending = deque(maxlen=8)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._deskew_ok = self._deskew_drop = 0
        self._deskew_received = 0
        self._deskew_last_error = "none"
        self.create_subscription(PointCloud,
            self.get_parameter('timed_cloud_topic').value,
            self.timed_callback, qos_profile_sensor_data)
        self.create_timer(0.02, self.process_pending)
        self.current_pitch = 0.0
        self.pitch_stamp = None
        self.settled = True
        # A LaserScan describes an acquisition window, not an instantaneous
        # sample.  Remember when the mount first became still so we can reject
        # the first scan delivered after a move (it usually straddles that
        # move even though the servo is stationary by callback time).
        self.settled_since = self.get_clock().now()
        self._warned_no_source = False
        self._n_scans = 0
        self._n_gated = 0

        self.scan_pub = self.create_publisher(
            LaserScan, self.get_parameter('scan_out_topic').value,
            qos_profile_sensor_data)
        # RELIABLE (default) on purpose: a BEST_EFFORT publisher would not
        # match octomap_server's RELIABLE subscription at all.
        self.pc_pub = self.create_publisher(
            PointCloud2, self.get_parameter('cloud_out_topic').value, 10)
        self.level_pub = self.create_publisher(Bool, '/lidar_tilt/is_level', 10)

        self.create_subscription(
            JointState, self.get_parameter('tilt_topic').value,
            self.joint_callback, 10)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_in_topic').value,
            self.scan_callback, qos_profile_sensor_data)

        self.create_timer(5.0, self._report)

        self.get_logger().info(
            "%s -> %s (gated at |tilt| < %.3f rad) + %s"
            % (self.get_parameter('scan_in_topic').value,
               self.get_parameter('scan_out_topic').value,
               self.level_threshold,
               self.get_parameter('cloud_out_topic').value))
        self.get_logger().info(
            "tilt source: %s (timeout %.1fs, assume level if NEVER seen: %s; "
            "a source that goes silent latches its last angle instead)"
            % (self.get_parameter('tilt_topic').value, self.tilt_timeout,
               self.assume_level))

    @property
    def require_settled(self):
        # Mode changes are applied through the ROS parameter service.
        return bool(self.get_parameter("require_settled").value)

    def joint_callback(self, msg):
        if self.joint_name not in msg.name:
            return
        idx = msg.name.index(self.joint_name)
        if idx < len(msg.position):
            pitch = msg.position[idx]
        else:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not math.isfinite(pitch) or stamp <= 0:
            return
        self.current_pitch = pitch
        if self.history and stamp <= self.history[-1][0]:
            self.history.clear()
            self.pending.clear()
        self.history.append((stamp, self.current_pitch,
                             not (idx < len(msg.velocity) and msg.velocity[idx] > 0.5)))
        while self.history and stamp - self.history[0][0] > 2.0:
            self.history.popleft()
        # lidar_tilt_node encodes "still moving" as velocity 1.0.
        was_settled = self.settled
        self.settled = not (idx < len(msg.velocity) and msg.velocity[idx] > 0.5)
        now = self.get_clock().now()
        if not self.settled:
            self.settled_since = None
        elif not was_settled or self.settled_since is None:
            self.settled_since = now
        self.pitch_stamp = now
        self._warned_no_source = False

    def _scan_captured_while_settled(self, msg):
        """True only after one complete scan window has elapsed at rest."""
        if not self.settled or self.settled_since is None:
            return False
        scan_window = float(msg.scan_time)
        if scan_window <= 0.0 and msg.time_increment > 0.0:
            scan_window = float(msg.time_increment) * max(0, len(msg.ranges) - 1)
        # A broken/missing driver duration must not disable the interlock.
        scan_window = max(0.001, scan_window)
        still_for = (self.get_clock().now() - self.settled_since).nanoseconds / 1e9
        return still_for >= scan_window

    def _tilt_state(self):
        """(pitch_rad, state) with state in {'fresh', 'stale', 'never'}.

        'stale' latches `current_pitch` rather than falling back to zero --
        see the module docstring for why zero is the dangerous answer.
        """
        if self.pitch_stamp is None:
            return (0.0, 'never')
        age = (self.get_clock().now() - self.pitch_stamp).nanoseconds / 1e9
        if age > self.tilt_timeout:
            return (self.current_pitch, 'stale')
        return (self.current_pitch, 'fresh')

    def scan_callback(self, msg):
        self._n_scans += 1
        pitch, state = self._tilt_state()
        captured_settled = self._scan_captured_while_settled(msg)

        if state == 'never':
            if not self._warned_no_source:
                self._warned_no_source = True
                self.get_logger().warn(
                    "nothing has ever published %s — %s.  Start "
                    "lidar_tilt_node (or x3_server's sweep loop) if the mount "
                    "can move."
                    % (self.get_parameter('tilt_topic').value,
                       "assuming level" if self.assume_level
                       else "withholding /scan"))
            if not self.assume_level:
                self._n_gated += 1
                self.level_pub.publish(Bool(data=False))
                return
            pitch = 0.0
        elif state == 'stale':
            # Latched, not zeroed.  See the module docstring: a silent source
            # means "the angle is no longer known", and the last thing it said
            # is a far better estimate of the mount's real position than level.
            if not self._warned_no_source:
                self._warned_no_source = True
                self.get_logger().error(
                    "%s went silent (>%.1fs) — latching the last known tilt "
                    "%+.2f deg.  /scan follows that latched angle (so a mount "
                    "that died level keeps feeding 2D SLAM, and one that died "
                    "tilted stays gated); 3D clouds are withheld either way, "
                    "because the true angle is no longer known."
                    % (self.get_parameter('tilt_topic').value,
                       self.tilt_timeout, math.degrees(pitch)))

        is_level = abs(pitch) < self.level_threshold
        self.level_pub.publish(Bool(data=bool(is_level)))

        # 3D projection consumes acquisition-ordered points independently.
        # LaserScan bins are angle ordered and cannot supply per-return times.

        # ── 2D branch ───────────────────────────────────────────────────
        # Do not leak a scan captured while the mount crossed level into 2D
        # SLAM.  That scan is pitched for most of its acquisition window even
        # if its callback-time angle happens to be inside the level threshold.
        if is_level and captured_settled:
            self.scan_pub.publish(msg)
        else:
            self._n_gated += 1

    def timed_callback(self, msg):
        self._deskew_received += 1
        if len(self.pending) == self.pending.maxlen:
            self._deskew_drop += 1
        self.pending.append((time.monotonic(), msg))

    @staticmethod
    def _pose(transform):
        t, q = transform.transform.translation, transform.transform.rotation
        return np.array([t.x, t.y, t.z]), np.array([q.x, q.y, q.z, q.w])

    def process_pending(self):
        while self.pending:
            received, msg = self.pending[0]
            try:
                self._project_timed(msg)
            except (ValueError, TransformException) as exc:
                if time.monotonic() - received < self.get_parameter('deskew_wait_s').value:
                    return
                self._deskew_drop += 1
                self._deskew_last_error = str(exc)
                self.get_logger().debug('deskew withheld: %s' % exc)
            self.pending.popleft()

    def _project_timed(self, msg):
        if not msg.points:
            return
        channels = {c.name: c.values for c in msg.channels}
        offsets = np.asarray(channels.get('acquisition_time', []), dtype=float)
        durations = np.asarray(channels.get('scan_duration', []), dtype=float)
        n = len(msg.points)
        if (len(offsets) != n or len(durations) != n
                or not np.isfinite(offsets).all() or not np.isfinite(durations).all()):
            raise ValueError('missing or invalid acquisition times')
        duration = float(durations[0])
        if (duration <= 0 or duration >= 1 or np.any(durations != duration)
                or offsets[0] < 0 or offsets[-1] > duration + 1e-6
                or np.any(np.diff(offsets) <= 0)):
            raise ValueError('invalid acquisition window')
        start_ns = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        start = start_ns * 1e-9
        if start_ns <= 0 or abs(self.get_clock().now().nanoseconds * 1e-9 - start) > 2:
            raise ValueError('stale cloud or clock mismatch')
        # Include the full scan window, even if first/last returns were invalid.
        times = np.r_[start, start + offsets, start + duration]
        pitches = interpolate_pitch(list(self.history), times,
            self.get_parameter('deskew_max_gap_s').value, self.require_settled)
        if not self.publish_cloud_when_level and np.all(np.abs(pitches) < self.level_threshold):
            return
        points = np.array([(p.x, p.y, p.z) for p in msg.points])
        ranges = np.linalg.norm(points, axis=1)
        minimum = np.where(np.abs(pitches[1:-1]) >= self.level_threshold,
                           self.tilted_min_range, 0.0)
        valid = np.isfinite(points).all(axis=1) & (ranges >= minimum)
        valid &= (ranges > 0) & (ranges < self.cloud_max_range)
        if not np.any(valid):
            return
        begin = Time(nanoseconds=start_ns, clock_type=self.get_clock().clock_type)
        end = Time(nanoseconds=start_ns + round(duration * 1e9),
                   clock_type=self.get_clock().clock_type)
        mount = self.get_parameter('deskew_mount_frame').value
        tilt = self.get_parameter('deskew_tilt_frame').value
        lookup = self.tf_buffer.lookup_transform
        m0 = self._pose(lookup('odom', mount, begin))
        m1 = self._pose(lookup('odom', mount, end))
        # X3's revolute joint origin has identity rotation and axis +Y.
        # Read its translation from TF, avoiding duplicated URDF dimensions.
        pivot = self._pose(lookup(mount, tilt, begin))[0]
        laser = self._pose(lookup(tilt, msg.header.frame_id, begin))
        reference = self._pose(lookup('odom', msg.header.frame_id, begin))
        xyz = deskew(points[valid], offsets[valid] / duration,
                     pitches[1:-1][valid], m0, m1, pivot, laser, reference)
        if not np.isfinite(xyz).all():
            raise ValueError('nonfinite transform result')
        cloud = PointCloud2()
        cloud.header = msg.header
        cloud.height, cloud.width = 1, len(xyz)
        cloud.fields = [PointField(name=name, offset=i * 4,
                        datatype=PointField.FLOAT32, count=1)
                        for i, name in enumerate(('x', 'y', 'z'))]
        cloud.is_bigendian = False
        cloud.point_step, cloud.row_step = 12, 12 * len(xyz)
        cloud.is_dense = True
        cloud.data = xyz.tobytes()
        self.pc_pub.publish(cloud)
        self._deskew_ok += 1

    def _report(self):
        self.get_logger().info("deskew: %d received, %d published, %d withheld, %d pending; last: %s"
                               % (self._deskew_received, self._deskew_ok, self._deskew_drop,
                                  len(self.pending), self._deskew_last_error))
        if not self._n_scans:
            self.get_logger().warn(
                "no scans on %s in the last 5s — is the ydlidar driver up and "
                "remapped to it?" % self.get_parameter('scan_in_topic').value)
            return
        pitch, state = self._tilt_state()
        self.get_logger().info(
            "%d scans, %d withheld from /scan; tilt %+.2f deg (%s%s)"
            % (self._n_scans, self._n_gated, math.degrees(pitch),
               {'fresh': 'live', 'stale': 'LATCHED, source silent',
                'never': 'no source'}[state],
               "" if self.settled else ", moving"))
        self._n_scans = 0
        self._n_gated = 0


def main(args=None):
    rclpy.init(args=args)
    node = Lidar3dProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
