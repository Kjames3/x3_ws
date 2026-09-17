#!/usr/bin/env python3
"""PAA5100JE optical flow publisher: /flow/twist + /flow/status.

Publishes base_link velocity (TwistWithCovarianceStamped, linear x/y only)
measured against the floor, which does not slip the way mecanum rollers do.
It is NOT fused into the EKF by this node; add it as a twist input there once
it has been drive-tested against wheel odometry.

The sensor sits ~13 mm behind base_link, so a pure rotation shows up as
sideways flow.  That term is removed using /imu/data's yaw rate; while the IMU
is stale the correction is skipped and the lateral variance is inflated.

    ros2 run yahboomcar_bringup flow_node
    ros2 run yahboomcar_bringup flow_node --ros-args -p sim:=true
"""
import json
import time

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import String

try:
    from .flow_geometry import (DEFAULT_COUNTS_PER_M, GOOD_SQUAL, Burst, counts_to_sensor_velocity,
                                parse_burst, sensor_to_body_velocity, velocity_variance)
except ImportError:                                   # run as a plain script
    from flow_geometry import (DEFAULT_COUNTS_PER_M, GOOD_SQUAL, Burst, counts_to_sensor_velocity,
                               parse_burst, sensor_to_body_velocity, velocity_variance)

REJECTED_VARIANCE = 1.0e3
IMU_STALE_S = 0.2


class FlowNode(Node):
    def __init__(self):
        super().__init__('flow_node')
        p = self.declare_parameter
        p('spi_bus', 0)
        p('spi_cs', 1)                 # /dev/spidev0.1, header pin 26
        p('rate_hz', 100.0)
        p('sim', False)
        p('frame_id', 'base_link')
        p('imu_topic', 'imu/data')
        # --- calibration, RE-MEASURE after any mount change ---
        p('counts_per_m', DEFAULT_COUNTS_PER_M)
        p('mount_x_m', -0.0134)        # paa5100je_link in base_link (URDF)
        p('mount_y_m', 0.0)
        p('good_squal', GOOD_SQUAL)

        g = self.get_parameter
        self.frame_id = g('frame_id').value
        self.counts_per_m = float(g('counts_per_m').value)
        self.mount_x = float(g('mount_x_m').value)
        self.mount_y = float(g('mount_y_m').value)
        self.good_squal = int(g('good_squal').value)

        self.pub = self.create_publisher(TwistWithCovarianceStamped, 'flow/twist', qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, 'flow/status', 10)
        self.create_subscription(Imu, g('imu_topic').value, self._on_imu, qos_profile_sensor_data)
        self.wz = 0.0
        self.imu_time = None

        self.sensor = None if g('sim').value else self._open()
        self.last_t = time.monotonic()
        self.create_timer(1.0 / float(g('rate_hz').value), self.tick)
        self.create_timer(2.0, self.report)
        self._reset_stats()

    def _open(self):
        try:
            from .paa5100je import PAA5100JE
        except ImportError:
            from paa5100je import PAA5100JE
        g = self.get_parameter
        sensor = PAA5100JE(spi_bus=int(g('spi_bus').value), spi_cs=int(g('spi_cs').value))
        self.get_logger().info('PAA5100JE initialised')
        return sensor

    def _on_imu(self, msg):
        self.wz = msg.angular_velocity.z
        self.imu_time = time.monotonic()

    def _reset_stats(self):
        self.n = self.n_motion = self.n_rejected = self.n_low_squal = 0
        self.squal_min = 255
        self.squal_sum = 0
        self.shutter = 0
        self.count_x = self.count_y = 0

    def tick(self):
        now = time.monotonic()
        dt, self.last_t = now - self.last_t, now
        if dt <= 0.0:
            return
        if self.sensor is None:
            burst = Burst(False, 0, 0, 220, 0, 0, 0, 7000)
        else:
            try:
                burst = parse_burst(self.sensor.read_burst())
            except OSError as exc:
                self.get_logger().warn(f'SPI read failed: {exc}', throttle_duration_sec=5.0)
                return

        vx_s, vy_s = counts_to_sensor_velocity(burst.dx, burst.dy, dt, self.counts_per_m)
        imu_fresh = self.imu_time is not None and now - self.imu_time < IMU_STALE_S
        if imu_fresh:
            vx, vy = sensor_to_body_velocity(vx_s, vy_s, self.wz, self.mount_x, self.mount_y)
        else:
            vx, vy = vx_s, vy_s

        if burst.rejected:
            var_x = var_y = REJECTED_VARIANCE
        else:
            var_x = velocity_variance(vx, burst.squal, good_squal=self.good_squal)
            var_y = velocity_variance(vy, burst.squal, good_squal=self.good_squal)
            if not imu_fresh:
                # Uncorrected rotation leaks into lateral flow: 1 rad/s at the
                # mount offset is ~|mount_x| m/s.
                var_y += max(abs(self.mount_x), abs(self.mount_y)) ** 2

        msg = TwistWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        cov = [0.0] * 36
        cov[0], cov[7] = var_x, var_y
        # z and all angular terms are unmeasured by this sensor.
        for i in (14, 21, 28, 35):
            cov[i] = REJECTED_VARIANCE
        msg.twist.covariance = cov
        # Published every tick, including zeros and rejects: a consumer must be
        # able to tell "robot is still" from "publisher died".
        self.pub.publish(msg)

        self.n += 1
        self.n_motion += burst.motion
        self.n_rejected += burst.rejected
        self.n_low_squal += burst.squal < self.good_squal
        self.squal_min = min(self.squal_min, burst.squal)
        self.squal_sum += burst.squal
        self.shutter = burst.shutter
        self.count_x += burst.dx
        self.count_y += burst.dy

    def report(self):
        if self.n == 0:
            self.get_logger().warn('no flow reads in 2 s')
            return
        msg = String()
        msg.data = json.dumps({
            'reads': self.n, 'motion': self.n_motion, 'rejected': self.n_rejected,
            'low_squal': self.n_low_squal, 'squal_min': self.squal_min,
            'squal_mean': round(self.squal_sum / self.n, 1), 'shutter': self.shutter,
            'counts_dx': self.count_x, 'counts_dy': self.count_y,
            'imu_fresh': self.imu_time is not None and time.monotonic() - self.imu_time < IMU_STALE_S,
        })
        self.status_pub.publish(msg)
        self._reset_stats()


def main(args=None):
    rclpy.init(args=args)
    node = FlowNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.sensor is not None:
            node.sensor.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
