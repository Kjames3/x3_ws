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
the rest through TF. Under heavy load the Orin Nano delivered /livox/imu at
13 Hz instead of 200; the rate is logged every 10 s so a drop is visible.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

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
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = self._gcov
        out.linear_acceleration.x = msg.linear_acceleration.x * G
        out.linear_acceleration.y = msg.linear_acceleration.y * G
        out.linear_acceleration.z = msg.linear_acceleration.z * G
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
