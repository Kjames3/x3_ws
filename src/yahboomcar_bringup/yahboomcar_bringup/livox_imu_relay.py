"""Relay the Livox Mid-360S IMU onto imu/data_raw.

The rebuilt chassis (2026-11-13) has no ICM-42688-P; the Livox's own IMU, at
the robot's centre, replaces it as the source the rest of the stack reads:
imu_filter_madgwick (-> /imu/data -> EKF) and the driver's vel_raw yaw rate
(use_external_imu_yaw).

Differences from what icm42688_node published, all fixed here:
  - accel is in g, not m/s^2  -> multiplied by 9.80665
  - header stamps are the lidar's clock, which is not synced to the Jetson
    (no PTP/GPS), so they are replaced with the receive time
  - orientation fields are unset -> orientation_covariance[0] = -1

The frame stays livox_frame (x forward, z up, verified 2026-10-29), so the
driver can take angular_velocity.z directly and robot_localization rotates
the rest through TF.

Gyro bias: the Livox gyro has a ~0.003 rad/s z bias (0.17 deg/s, ~10 deg/min of
false yaw while parked, measured 2026-11-20; it dragged AMCL off the map). It is
estimated whenever the robot is plainly still with the same BiasTracker the ICM
node used (gyro_bias.py) and subtracted (track_bias, default on). Replaying the
parked lab bag through it left 0.003 deg/s. Under heavy load the Orin Nano delivered /livox/imu at
13 Hz instead of 200; the rate is logged every 10 s so a drop is visible.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

try:
    from .gyro_bias import BiasTracker
except ImportError:  # run as a script
    from gyro_bias import BiasTracker

G = 9.80665


class LivoxImuRelay(Node):
    def __init__(self):
        super().__init__('livox_imu_relay')
        self.declare_parameter('input_topic', '/livox/imu')
        self.declare_parameter('output_topic', 'imu/data_raw')
        self.declare_parameter('frame_id', 'livox_frame')
        self.declare_parameter('restamp', True)
        self.declare_parameter('gyro_var', 1e-4)
        self.declare_parameter('accel_var', 1e-2)
        self._frame = self.get_parameter('frame_id').value
        self._restamp = self.get_parameter('restamp').value
        gv = self.get_parameter('gyro_var').value
        av = self.get_parameter('accel_var').value
        self.declare_parameter('track_bias', True)
        self._track = bool(self.get_parameter('track_bias').value)
        self._tracker = BiasTracker(window=200)  # ~1 s at 200 Hz
        self._bias = (0.0, 0.0, 0.0)
        self._logged_bias = self._bias
        self._gcov = [gv, 0.0, 0.0, 0.0, gv, 0.0, 0.0, 0.0, gv]
        self._acov = [av, 0.0, 0.0, 0.0, av, 0.0, 0.0, 0.0, av]
        self._pub = self.create_publisher(Imu, self.get_parameter('output_topic').value, 50)
        self.create_subscription(Imu, self.get_parameter('input_topic').value,
                                 self._cb, qos_profile_sensor_data)
        self._n = 0
        self.create_timer(10.0, self._report)

    def _cb(self, msg):
        out = Imu()
        out.header.stamp = self.get_clock().now().to_msg() if self._restamp else msg.header.stamp
        out.header.frame_id = self._frame
        out.orientation_covariance[0] = -1.0
        w = msg.angular_velocity
        acc = (msg.linear_acceleration.x * G, msg.linear_acceleration.y * G, msg.linear_acceleration.z * G)
        if self._track and self._tracker.update((w.x, w.y, w.z), acc):
            self._bias = tuple(self._tracker.bias)
            if max(abs(b - l) for b, l in zip(self._bias, self._logged_bias)) > 2.0e-4:
                self._logged_bias = self._bias
                self.get_logger().info('gyro bias now [%+.5f, %+.5f, %+.5f] rad/s (%d still windows)'
                                       % (*self._bias, self._tracker.updates))
        out.angular_velocity = Vector3(x=w.x - self._bias[0], y=w.y - self._bias[1], z=w.z - self._bias[2])
        out.angular_velocity_covariance = self._gcov
        out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z = acc
        out.linear_acceleration_covariance = self._acov
        self._pub.publish(out)
        self._n += 1

    def _report(self):
        rate = self._n / 10.0
        self._n = 0
        # Two call sites: rclpy refuses one call site switching severity
        # ('Logger severity cannot be changed between calls'), which killed
        # this node the first time the rate crossed 150 Hz.
        if rate < 150.0:
            self.get_logger().warn('livox imu relay: %.0f Hz (expected ~200)' % rate)
        else:
            self.get_logger().info('livox imu relay: %.0f Hz' % rate)


def main(args=None):
    rclpy.init(args=args)
    node = LivoxImuRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
