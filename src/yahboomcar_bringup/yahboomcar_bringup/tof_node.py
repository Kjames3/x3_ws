#!/usr/bin/env python3
"""VL53L5CX ToF array publisher: /tof/points + the tof_link static TF.

SINGLE-SENSOR i2c-1 BENCH PATH -- not how the robot runs.  Since 2026-09-22
the dual arrays are read by a Teensy over USB and server_x3.py publishes
/tof/{upper,lower}/points in the URDF's tof_{upper,lower}_link.  The server
holds the Teensy port exclusively, and nothing launches this node.

Fills the low blind band no other sensor on this robot covers -- the scan plane
is 0.340 m, the camera is floor-blind under ~0.57 m, and the costmap's
min_obstacle_height is 0.12 m, so a low box in front of the wheels is invisible
to lidar, camera and costmap alike.

The mount extrinsics are PARAMETERS, not URDF and not constants in the
geometry code, for one reason: `pitch_deg` here is a sensor-to-FLOOR angle, and
like camera_pitch_deg it has to be RE-MEASURED after any bracket change.  A
number baked into a file is a number nobody re-measures.  Check it with

    python3 src/tof_array.py --coverage <height_m> <pitch_deg>

against a real flat floor: the rows should land where the table says they do.
Disagreement is a mount-angle error, not sensor noise.

    ros2 run yahboomcar_bringup tof_node
    ros2 run yahboomcar_bringup tof_node --ros-args -p sim:=true -p pitch_deg:=15.0
"""
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster

try:
    from .tof_geometry import (DEFAULT_MAX_RANGE_M, DEFAULT_MIN_RANGE_M,
                              VALID_STATUS, frame_to_points)
except ImportError:                                   # run as a plain script
    from tof_geometry import (DEFAULT_MAX_RANGE_M, DEFAULT_MIN_RANGE_M,
                              VALID_STATUS, frame_to_points)


def cloud_msg(points, frame_id, stamp):
    """(N,3) float32 -> PointCloud2.  N == 0 is legal and meaningful."""
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = len(points)
    msg.fields = [PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(('x', 'y', 'z'))]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.is_dense = True
    msg.data = np.ascontiguousarray(points, dtype='<f4').tobytes()
    return msg


def _import_driver():
    """Find drivers_x3.VL53L5CXArray from either a source tree or an install.

    colcon installs this package under install/, where the repo's src/ is no
    longer two levels up, so the relative path alone is not enough -- hence the
    X3_WS override and the walk up from __file__.
    """
    import os
    import sys
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [os.environ.get('X3_WS_SRC', ''),
                  os.path.join(here, '..', '..')]
    d = here
    for _ in range(6):                      # walk up looking for a real src/
        d = os.path.dirname(d)
        candidates.append(os.path.join(d, 'src'))
    for c in candidates:
        if c and os.path.isfile(os.path.join(c, 'drivers_x3.py')):
            if c not in sys.path:
                sys.path.insert(0, c)
            break
    from drivers_x3 import VL53L5CXArray
    return VL53L5CXArray


class TofNode(Node):
    def __init__(self):
        super().__init__('tof_node')
        p = self.declare_parameter
        p('i2c_bus', 1)               # pins 27/28; NOT i2c-7 with the 200 Hz ICM
        p('i2c_address', 0x29)
        p('resolution', 8)            # 8x8 caps at 15 Hz, 4x4 at 60
        p('ranging_freq_hz', 10)
        p('sharpener_percent', 5)
        p('max_range_m', DEFAULT_MAX_RANGE_M)
        p('min_range_m', DEFAULT_MIN_RANGE_M)
        p('sim', False)
        p('frame_id', 'tof_link')
        p('parent_frame', 'base_footprint')
        # --- mount, RE-MEASURE after any bracket change ---
        p('mount_x_m', 0.10)          # forward of base_footprint
        p('mount_y_m', 0.0)
        p('mount_z_m', 0.055)         # optical centre above the FLOOR
        p('pitch_deg', 15.0)          # positive = nose down
        p('yaw_deg', 0.0)             # for a second, outward-canted unit
        # --- zone ordering, VERIFY with tof_array.py --stream ---
        p('transpose', False)
        p('flip_h', False)
        p('flip_v', False)

        g = self.get_parameter
        self.frame_id = g('frame_id').value
        self.min_range = float(g('min_range_m').value)
        self.max_range = float(g('max_range_m').value)
        self.resolution = int(g('resolution').value)
        self.order = dict(transpose=g('transpose').value,
                          flip_h=g('flip_h').value,
                          flip_v=g('flip_v').value)

        self.pub = self.create_publisher(PointCloud2, 'tof/points', qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, 'tof/status', 10)
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform(self._mount_tf())

        self.sensor = self._open()
        # Poll at 3x the ranging rate: data_ready() is cheap, and undersampling
        # the ULD's buffer costs whole frames.
        hz = float(g('ranging_freq_hz').value)
        self.create_timer(1.0 / (3.0 * hz), self.tick)
        self.create_timer(2.0, self.report)
        self.n_frames = 0
        self.n_empty = 0
        self.n_valid = 0

    def _open(self):
        g = self.get_parameter
        VL53L5CXArray = _import_driver()
        return VL53L5CXArray(
            i2c_bus=int(g('i2c_bus').value), i2c_addr=int(g('i2c_address').value),
            resolution=self.resolution, ranging_freq_hz=int(g('ranging_freq_hz').value),
            sharpener_percent=int(g('sharpener_percent').value),
            max_range_m=self.max_range, sim_mode=bool(g('sim').value), **self.order)

    def _mount_tf(self):
        g = self.get_parameter
        pitch = math.radians(float(g('pitch_deg').value))
        yaw = math.radians(float(g('yaw_deg').value))
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = g('parent_frame').value
        tf.child_frame_id = self.frame_id
        tf.transform.translation.x = float(g('mount_x_m').value)
        tf.transform.translation.y = float(g('mount_y_m').value)
        tf.transform.translation.z = float(g('mount_z_m').value)
        # roll=0, pitch, yaw -> quaternion.  Positive pitch tips +x downward,
        # which is what a nose-down bracket does.
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        tf.transform.rotation.w = cp * cy
        tf.transform.rotation.x = -sp * sy
        tf.transform.rotation.y = sp * cy
        tf.transform.rotation.z = cp * sy
        return tf

    def tick(self):
        if not self.sensor.available or not self.sensor.data_ready():
            return
        frame = self.sensor.read_frame()
        if frame is None:
            return
        pts, keep = frame_to_points(
            frame[0], frame[1], resolution=self.resolution,
            valid_status=VALID_STATUS, min_range_m=self.min_range,
            max_range_m=self.max_range, **self.order)
        self.n_frames += 1
        self.n_valid += int(keep.sum())
        if len(pts) == 0:
            self.n_empty += 1
        # An empty cloud is published, not skipped.  A consumer that stops
        # receiving must be able to tell "measured nothing" from "publisher
        # died" -- a gated topic that silently freezes a downstream obstacle
        # set is exactly the F7 failure, where a stale obstacle it could never
        # re-measure drove the robot into a wall.
        self.pub.publish(cloud_msg(pts, self.frame_id, self.get_clock().now().to_msg()))

    def report(self):
        if self.n_frames == 0:
            self.get_logger().warn(
                'no ToF frames in 2 s — check `python3 src/tof_array.py --scan`')
            return
        msg = String()
        msg.data = (f'{{"frames": {self.n_frames}, "empty": {self.n_empty}, '
                    f'"mean_valid_zones": '
                    f'{self.n_valid / self.n_frames:.1f}}}')
        self.status_pub.publish(msg)
        self.n_frames = self.n_empty = self.n_valid = 0


def main(args=None):
    rclpy.init(args=args)
    node = TofNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.sensor.cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
