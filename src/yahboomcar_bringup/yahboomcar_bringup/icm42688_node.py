#!/usr/bin/env python3
"""ICM-42688-P IMU publisher (i2c-7 @ 0x68).

Replaces the MPU9250 that reaches ROS via the Rosmaster's 115200-baud serial
bridge on Mcnamu_driver_X3's 0.1 s pub_data timer. That bridge, not the sensor,
is what capped the old IMU at 10 Hz; reading this part directly off i2c-7 gives
200 Hz with Jetson-side timestamps.

Mount (measured on hardware 2026-09-01, NOT assumed from a description):
gravity sits on sensor Z, so Z is up. The verified sensor->ROS map is

    ros_x (fwd)  = -sensor_y
    ros_y (left) = +sensor_x
    ros_z (up)   = +sensor_z

a pure +90 deg yaw (URDF rpy="0 0 1.5708"), det +1. Yaw needs NO sign flip --
do not copy the `-gz` negation from Mcnamu_driver_X3, which exists only because
the MPU9250 is mounted upside down.

Run standalone for an A/B against the old IMU:
    python3 icm42688_node.py --ros-args -p topic:=/imu/icm/data_raw
"""
import json
import math
import os

import rclpy
import smbus2
from rclpy.node import Node
from sensor_msgs.msg import Imu

WHO_AM_I, EXPECTED = 0x75, 0x47
PWR_MGMT0, GYRO_CONFIG0, ACCEL_CONFIG0 = 0x4E, 0x4F, 0x50
ACCEL_DATA_X1 = 0x1F
REG_BANK_SEL = 0x76
# Anti-alias filter. Bank 1 = gyro, bank 2 = accel. The AAF runs BEFORE
# decimation to the output rate, so it is the only thing that can stop
# out-of-band vibration folding into the passband; the UI filter is applied
# after decimation and cannot.
GYRO_CONFIG_STATIC2, GYRO_CONFIG_STATIC3 = 0x0B, 0x0C
GYRO_CONFIG_STATIC4, GYRO_CONFIG_STATIC5 = 0x0D, 0x0E
ACCEL_CONFIG_STATIC2, ACCEL_CONFIG_STATIC3, ACCEL_CONFIG_STATIC4 = 0x03, 0x04, 0x05
# bandwidth Hz -> (DELT, DELTSQR, BITSHIFT), from the ICM-42688-P AAF table.
AAF_TABLE = {
    42:  (1, 1, 15),
    84:  (2, 4, 13),
    126: (3, 9, 12),
    170: (4, 16, 11),
    213: (5, 25, 10),
    258: (6, 36, 10),      # part default -- far above a 200 Hz Nyquist
}
GYRO_LSB_PER_DPS, ACCEL_LSB_PER_G = 16.4, 2048.0
DEG2RAD, G = math.pi / 180.0, 9.80665

# ODR code -> Hz, for ACCEL_CONFIG0/GYRO_CONFIG0 bits [3:0]
ODR_CODES = {1000.0: 0x06, 200.0: 0x07, 100.0: 0x08, 50.0: 0x09}


class ICM42688Node(Node):
    def __init__(self):
        super().__init__('icm42688_node')
        self.declare_parameter('i2c_bus', 7)
        self.declare_parameter('i2c_addr', 0x68)
        self.declare_parameter('topic', 'imu/data_raw')
        self.declare_parameter('frame_id', 'icm_imu_link')
        self.declare_parameter('rate', 200.0)
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('apply_bias', True)
        self.declare_parameter('apply_accel_scale', True)
        # Measured on this robot 2026-09-01 at 1 kHz: driving at 0.30 m/s puts
        # 85.6% of gyro energy above 100 Hz (peak 177 Hz). Sampled at 200 Hz
        # that folds to ~23 Hz and is indistinguishable from real rotation.
        # 42 Hz is well under the 100 Hz Nyquist and ~10x any real robot yaw.
        self.declare_parameter('aaf_bandwidth_hz', 42)

        p = self.get_parameter
        self.frame_id = p('frame_id').value
        rate = float(p('rate').value)
        self.addr = int(p('i2c_addr').value)

        self._load_calibration()

        self.bus = smbus2.SMBus(int(p('i2c_bus').value))
        wai = self.bus.read_byte_data(self.addr, WHO_AM_I)
        if wai != EXPECTED:
            raise SystemExit(
                f'WHO_AM_I 0x{wai:02X} != 0x{EXPECTED:02X} on i2c-'
                f'{p("i2c_bus").value} @ 0x{self.addr:02X}')

        # AAF first: the STATIC registers must be written while the sensors are
        # still off, which is why PWR_MGMT0 is the last write in this block.
        self._configure_aaf(int(p('aaf_bandwidth_hz').value))

        odr = ODR_CODES.get(rate, ODR_CODES[200.0])
        self.bus.write_byte_data(self.addr, GYRO_CONFIG0, odr)    # +/-2000 dps
        self.bus.write_byte_data(self.addr, ACCEL_CONFIG0, odr)   # +/-16 g
        self.bus.write_byte_data(self.addr, PWR_MGMT0, 0x0F)      # accel+gyro LN

        self.pub = self.create_publisher(Imu, p('topic').value, 50)
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self._bad = 0

        self.get_logger().info(
            f'ICM-42688-P up: i2c-{p("i2c_bus").value} @ 0x{self.addr:02X}, '
            f'{rate:.0f} Hz -> {p("topic").value} (frame {self.frame_id})')
        self.get_logger().info(
            f'  gyro bias {"applied" if self.apply_bias else "NOT applied"}: '
            f'[{self.gbias[0]:+.5f}, {self.gbias[1]:+.5f}, {self.gbias[2]:+.5f}] rad/s')

    def _configure_aaf(self, bw):
        """Set the gyro+accel anti-alias filter to `bw` Hz (see AAF_TABLE)."""
        if bw not in AAF_TABLE:
            self.get_logger().warn(
                f'aaf_bandwidth_hz={bw} not one of {sorted(AAF_TABLE)}; '
                'leaving the part default (258 Hz) -- expect vibration aliasing')
            return
        delt, dsqr, shift = AAF_TABLE[bw]
        try:
            # ---- gyro AAF (bank 1) ----
            self.bus.write_byte_data(self.addr, REG_BANK_SEL, 1)
            cfg2 = self.bus.read_byte_data(self.addr, GYRO_CONFIG_STATIC2)
            self.bus.write_byte_data(self.addr, GYRO_CONFIG_STATIC2,
                                     cfg2 & ~0x02)          # clear GYRO_AAF_DIS
            self.bus.write_byte_data(self.addr, GYRO_CONFIG_STATIC3, delt & 0x3F)
            self.bus.write_byte_data(self.addr, GYRO_CONFIG_STATIC4, dsqr & 0xFF)
            self.bus.write_byte_data(self.addr, GYRO_CONFIG_STATIC5,
                                     ((shift & 0x0F) << 4) | ((dsqr >> 8) & 0x0F))
            # ---- accel AAF (bank 2) ----
            self.bus.write_byte_data(self.addr, REG_BANK_SEL, 2)
            self.bus.write_byte_data(self.addr, ACCEL_CONFIG_STATIC2,
                                     (delt & 0x3F) << 1)    # bit0 = AAF_DIS, 0
            self.bus.write_byte_data(self.addr, ACCEL_CONFIG_STATIC3, dsqr & 0xFF)
            self.bus.write_byte_data(self.addr, ACCEL_CONFIG_STATIC4,
                                     ((shift & 0x0F) << 4) | ((dsqr >> 8) & 0x0F))
        finally:
            self.bus.write_byte_data(self.addr, REG_BANK_SEL, 0)

        # read back, because a silently-ignored filter looks exactly like a
        # working one until the robot drives
        self.bus.write_byte_data(self.addr, REG_BANK_SEL, 1)
        rb = (self.bus.read_byte_data(self.addr, GYRO_CONFIG_STATIC3) & 0x3F,
              self.bus.read_byte_data(self.addr, GYRO_CONFIG_STATIC4),
              self.bus.read_byte_data(self.addr, GYRO_CONFIG_STATIC5) >> 4)
        dis = self.bus.read_byte_data(self.addr, GYRO_CONFIG_STATIC2) & 0x02
        self.bus.write_byte_data(self.addr, REG_BANK_SEL, 0)
        if rb == (delt, dsqr, shift) and not dis:
            self.get_logger().info(f'AAF set to {bw} Hz (DELT={delt}, '
                                   f'DELTSQR={dsqr}, BITSHIFT={shift}), enabled')
        else:
            self.get_logger().error(
                f'AAF READBACK MISMATCH: wanted {(delt, dsqr, shift)} got {rb}, '
                f'AAF_DIS={bool(dis)} -- vibration WILL alias')

    def _load_calibration(self):
        """Bias/scale from the calibration file; zeros if absent, never fatal."""
        self.gbias = [0.0, 0.0, 0.0]
        self.ascale = 1.0
        self.apply_bias = bool(self.get_parameter('apply_bias').value)
        path = self.get_parameter('calibration_file').value
        if not path:
            here = os.path.dirname(os.path.abspath(__file__))
            for cand in (os.path.join(here, '..', '..', '..', 'config',
                                      'icm42688_calibration.json'),
                         '/home/jetson/x3_ws/config/icm42688_calibration.json'):
                if os.path.exists(cand):
                    path = cand
                    break
        if not path or not os.path.exists(path):
            self.get_logger().warn(
                'no calibration file found -- publishing UNCALIBRATED '
                '(gyro bias will show as heading drift)')
            return
        try:
            with open(path) as f:
                cal = json.load(f)
            self.gbias = [float(v) for v in cal.get('gyro_bias_rad_s', self.gbias)]
            if self.get_parameter('apply_accel_scale').value:
                self.ascale = float(cal.get('accel_scale_correction', 1.0))
            self.get_logger().info(f'calibration: {os.path.normpath(path)}')
        except Exception as e:                      # noqa: BLE001 - never fatal
            self.get_logger().warn(f'calibration file unreadable ({e}); using zeros')

    def tick(self):
        try:
            d = self.bus.read_i2c_block_data(self.addr, ACCEL_DATA_X1, 12)
        except OSError as e:
            self._bad += 1
            if self._bad in (1, 10, 100) or self._bad % 1000 == 0:
                self.get_logger().warn(f'i2c read failed x{self._bad}: {e}')
            return

        def s16(i):
            v = (d[i] << 8) | d[i + 1]
            return v - 65536 if v & 0x8000 else v

        k_a = G / ACCEL_LSB_PER_G * self.ascale
        ax, ay, az = s16(0) * k_a, s16(2) * k_a, s16(4) * k_a
        k_g = DEG2RAD / GYRO_LSB_PER_DPS
        gx, gy, gz = s16(6) * k_g, s16(8) * k_g, s16(10) * k_g
        if self.apply_bias:
            gx -= self.gbias[0]; gy -= self.gbias[1]; gz -= self.gbias[2]

        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame_id
        # measured sensor->ROS map; NO sign flips beyond the -y forward term
        m.linear_acceleration.x = -ay
        m.linear_acceleration.y = ax
        m.linear_acceleration.z = az
        m.angular_velocity.x = -gy
        m.angular_velocity.y = gx
        m.angular_velocity.z = gz
        # orientation is not estimated here; Madgwick downstream does that
        m.orientation_covariance[0] = -1.0
        v = 4.0e-7        # ~ (0.00067 rad/s)^2, the measured per-axis noise
        m.angular_velocity_covariance[0] = v
        m.angular_velocity_covariance[4] = v
        m.angular_velocity_covariance[8] = v
        a = 1.0e-4
        m.linear_acceleration_covariance[0] = a
        m.linear_acceleration_covariance[4] = a
        m.linear_acceleration_covariance[8] = a
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = ICM42688Node()
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
