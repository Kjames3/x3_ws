#!/usr/bin/env python3
"""lidar_3d_processor_node — split the tilting lidar's scans two ways.

    /scan_raw  ──┬─→ /scan            2D SLAM / Nav2, ONLY while level
                 └─→ /pointcloud_raw  3D projection for octomap_server

The gate matters: slam_toolbox and AMCL both assume a horizontal scan plane, so
feeding them a tilted scan silently corrupts the map and the pose.  The gate is
on |tilt|, which stays correct even though the servo's sign convention is still
unverified (see lidar_tilt_node._resolve_direction).

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

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, JointState
from std_msgs.msg import Bool
from laser_geometry import LaserProjection


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
        self.declare_parameter('cloud_max_range_m', 8.0)
        self.declare_parameter('publish_cloud_when_level', True)

        self.joint_name = self.get_parameter('joint_name').value
        self.level_threshold = float(
            self.get_parameter('level_threshold_rad').value)
        self.tilt_timeout = float(self.get_parameter('tilt_timeout_s').value)
        self.assume_level = bool(
            self.get_parameter('assume_level_if_no_source').value)
        self.require_settled = bool(
            self.get_parameter('require_settled').value)
        self.tilted_min_range = float(
            self.get_parameter('tilted_min_range_m').value)
        self.cloud_max_range = float(
            self.get_parameter('cloud_max_range_m').value)
        self.publish_cloud_when_level = bool(
            self.get_parameter('publish_cloud_when_level').value)

        self.projector = LaserProjection()
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

    def joint_callback(self, msg):
        if self.joint_name not in msg.name:
            return
        idx = msg.name.index(self.joint_name)
        if idx < len(msg.position):
            self.current_pitch = msg.position[idx]
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

        # ── 3D branch ───────────────────────────────────────────────────
        # publish_cloud_when_level is FALSE wherever octomap runs for real (see
        # params/lidar_3d_processor.yaml): a level scan is a horizontal slice
        # 2D SLAM already covers, and re-projecting + re-inserting it every
        # 125 ms costs continuous CPU and DDS traffic for zero information.
        # Turn it on only to get something on screen during bringup.
        want_cloud = ((not is_level) or self.publish_cloud_when_level) \
            and state != 'stale'
        if want_cloud and (captured_settled or not self.require_settled):
            try:
                scan = msg
                if not is_level and self.tilted_min_range > msg.range_min:
                    scan = self._mask_near(msg, self.tilted_min_range)
                cloud = self.projector.projectLaser(
                    scan, range_cutoff=self.cloud_max_range)
                self.pc_pub.publish(cloud)
            except Exception as e:
                self.get_logger().warn("failed to project laser: %s" % e)

        # ── 2D branch ───────────────────────────────────────────────────
        # Do not leak a scan captured while the mount crossed level into 2D
        # SLAM.  That scan is pitched for most of its acquisition window even
        # if its callback-time angle happens to be inside the level threshold.
        if is_level and (captured_settled or not self.require_settled):
            self.scan_pub.publish(msg)
        else:
            self._n_gated += 1

    @staticmethod
    def _mask_near(msg, min_range):
        """Copy the scan with sub-`min_range` returns blanked to +inf.

        projectLaser drops anything below scan.range_min, so raising range_min
        alone would be enough — except that Nav2 and slam_toolbox read
        range_min off the same message elsewhere.  Blanking the ranges keeps
        the message's own limits honest.
        """
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.intensities = msg.intensities
        out.ranges = [r if r >= min_range else math.inf for r in msg.ranges]
        return out

    def _report(self):
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
