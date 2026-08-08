#!/usr/bin/env python3
# coding: utf-8

import logging
import struct
import time
import serial
import threading
from collections import namedtuple

_LOG = logging.getLogger(__name__)

# Longest legal frame is ext_len + 2; ext_len is one byte but real frames from
# this firmware are <= 23 B. Anything larger is a corrupt length byte that
# happened to follow a false 0xFF 0xFB header, not a frame.
_RX_MAX_FRAME = 64

# --- precompiled frame layouts -------------------------------------------
# The '<' prefix is MANDATORY on every one of these. Without a byte-order
# prefix struct uses *native alignment* and silently inserts padding:
#   calcsize('Bh')   == 4  (pads!)   calcsize('<Bh')   == 3
#   calcsize('Bhhh') == 8  (pads!)   calcsize('<Bhhh') == 7
# A padded 'Bh' would read the servo value from the wrong offset, and a padded
# 'Bhhh' shifts every PID gain by one byte. The firmware is little-endian, so
# '<' is also correct on its own merits rather than relying on the host being
# aarch64/x86.
_S_SPEED = struct.Struct('<hhhB')   # vx, vy mm/s; vz mrad/s; battery 0.1 V  (7 B)
_S_IMU9  = struct.Struct('<9h')     # gx gy gz ax ay az mx my mz            (18 B)
_S_ATT   = struct.Struct('<3h')     # roll pitch yaw, 1e-4 rad               (6 B)
_S_ENC   = struct.Struct('<4i')     # m1..m4 signed tick counts             (16 B)
_S_SERVO = struct.Struct('<Bh')     # id, value                              (3 B)
_S_ARM   = struct.Struct('<6h')     # 6 joint pulses                        (12 B)
_S_PID   = struct.Struct('<Bhhh')   # index, kp, ki, kd                      (7 B)
_S_2B    = struct.Struct('<2B')     # version / arm-offset / akm pairs       (2 B)
_S_1B    = struct.Struct('<B')      # car type                               (1 B)

# --- telemetry snapshots --------------------------------------------------
# Every auto-report packet is published as ONE immutable namedtuple stored with
# a single STORE_ATTR. A consumer that takes one reference sees a single, self
# consistent instant; the previous per-field attributes let a reader mix the
# accelerometer from packet N with the gyroscope from packet N+1 (measured at
# ~0.125% of publish cycles at 20 Hz, which is exactly the kind of correlated
# glitch a Madgwick filter feeding an EKF integrates rather than rejects).
# `t` is a time.monotonic() stamp, which is what makes staleness detectable.
ImuSample = namedtuple('ImuSample', 'gx gy gz ax ay az mx my mz t seq')
AttSample = namedtuple('AttSample', 'roll pitch yaw t seq')
MotionSample = namedtuple('MotionSample', 'vx vy vz battery t seq')
EncSample = namedtuple('EncSample', 'm1 m2 m3 m4 t seq')

_ZERO_IMU = ImuSample(0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0, 0)
_ZERO_ATT = AttSample(0, 0, 0, 0.0, 0)
_ZERO_MOTION = MotionSample(0, 0, 0, 0, 0.0, 0)
_ZERO_ENC = EncSample(0, 0, 0, 0, 0.0, 0)

# --- IMU frame convention -------------------------------------------------
# The stock library applied a *partial* axis flip in the MPU9250 branch (gyro y
# and z negated; accelerometer and magnetometer untouched) and no flip at all in
# the ICM20948 branch. That is not a coherent frame transform and the two
# branches disagreed with each other, so the same physical rotation produced a
# different sign depending on which sensor the board carried.
#
# This board is an MPU9250: it emits ext_type 0x0B only (732 frames vs 0 of 0x0E
# measured over 30 s on firmware V2.4), so the ICM branch is dead code here.
# We therefore adopt the MPU branch's convention as the single documented one
# and apply it uniformly in both branches. Published gyro/accel values are
# BIT-FOR-BIT UNCHANGED on this hardware - this is a de-duplication, not a
# recalibration.
#
# KNOWN-UNRESOLVED: Mcnamu_driver_X3.pub_data negates gz a *second* time, so the
# two negations cancel and published angular_velocity.z carries the raw sensor
# sign. Deciding which of the two negations is the redundant one requires
# physically rotating the chassis and checking /imu/data_raw.angular_velocity.z
# against REP-103 (CCW seen from above = positive). Until someone does that,
# both negations stay so that net behaviour is preserved.
_GYRO_SIGNS = (1.0, -1.0, -1.0)
_ACCEL_SIGNS = (1.0, 1.0, 1.0)
_MAG_SIGNS = (1.0, 1.0, 1.0)

# Gyro: +-500 dps full scale -> 32768 / (500*pi/180) = 3754.7 LSB per rad/s.
_GYRO_RATIO_MPU = 1 / 3754.9
# Accel: +-2 g -> 32768 / (2*9.8) = 1671.8 LSB per m/s^2.
_ACCEL_RATIO_MPU = 1 / 1671.84
# Mag: the MPU branch used a ratio of 1, i.e. it published raw AK8963 counts
# into a sensor_msgs/MagneticField field that ROS defines as TESLA - roughly
# 6.7e6x too large. The AK8963 is ~0.15 uT/LSB in 16-bit mode, which reproduces
# the measured ~61.2 uT ambient field from the observed counts.
_MAG_RATIO_MPU = 0.15e-6
# ICM20948 branch: never observed on this hardware, kept for other boards. Its
# firmware scales gyro/accel by 1/1000 (mrad/s, mm/s^2); the magnetometer is
# assumed to arrive in uT by the same logic and is converted to tesla. UNVERIFIED.
_GYRO_RATIO_ICM = 1 / 1000.0
_ACCEL_RATIO_ICM = 1 / 1000.0
_MAG_RATIO_ICM = 1e-6


# V3.3.1
class Rosmaster(object):
    __uart_state = 0

    def __init__(self, car_type=1, com="/dev/myserial", delay=.002, debug=False):
        # com = "COM30"
        # com="/dev/ttyTHS1"
        # com="/dev/ttyUSB0"
        # com="/dev/ttyAMA0"

        # A read timeout is REQUIRED, not cosmetic: it is what lets the receive
        # thread observe the stop flag, and what turns a dead/unplugged port
        # into a timeout instead of a permanent block.
        self.ser = serial.Serial(com, 115200, timeout=0.1)

        self.__delay_time = delay
        self.__debug = debug

        self.__HEAD = 0xFF
        self.__DEVICE_ID = 0xFC
        self.__COMPLEMENT = 257 - self.__DEVICE_ID
        self.__CAR_TYPE = car_type
        self.__CAR_ADJUST = 0x80

        self.FUNC_AUTO_REPORT = 0x01
        self.FUNC_BEEP = 0x02
        self.FUNC_PWM_SERVO = 0x03
        self.FUNC_PWM_SERVO_ALL = 0x04
        self.FUNC_RGB = 0x05
        self.FUNC_RGB_EFFECT = 0x06

        self.FUNC_REPORT_SPEED = 0x0A
        self.FUNC_REPORT_MPU_RAW = 0x0B
        self.FUNC_REPORT_IMU_ATT = 0x0C
        self.FUNC_REPORT_ENCODER = 0x0D
        self.FUNC_REPORT_ICM_RAW = 0x0E
        
        self.FUNC_RESET_STATE = 0x0F

        self.FUNC_MOTOR = 0x10
        self.FUNC_CAR_RUN = 0x11
        self.FUNC_MOTION = 0x12
        self.FUNC_SET_MOTOR_PID = 0x13
        self.FUNC_SET_YAW_PID = 0x14
        self.FUNC_SET_CAR_TYPE = 0x15

        self.FUNC_UART_SERVO = 0x20
        self.FUNC_UART_SERVO_ID = 0x21
        self.FUNC_UART_SERVO_TORQUE = 0x22
        self.FUNC_ARM_CTRL = 0x23
        self.FUNC_ARM_OFFSET = 0x24

        self.FUNC_AKM_DEF_ANGLE = 0x30
        self.FUNC_AKM_STEER_ANGLE = 0x31


        self.FUNC_REQUEST_DATA = 0x50
        self.FUNC_VERSION = 0x51

        self.FUNC_RESET_FLASH = 0xA0

        self.CARTYPE_X3 = 0x01
        self.CARTYPE_X3_PLUS = 0x02
        self.CARTYPE_X1 = 0x04
        self.CARTYPE_R2 = 0x05

        # Receive-thread lifecycle and health. Without these the RX thread dies
        # silently on the first port error and every getter keeps returning its
        # last value forever - the driver then republishes frozen IMU data with
        # a fresh timestamp, which an EKF fuses as "the robot is perfectly
        # still". A detectable failure is worth more than a fast one.
        self.__rx_thread = None
        self.__rx_stop = threading.Event()
        self.__rx_error = None
        self.__rx_last_frame_t = 0.0
        self.__rx_stats = {'frames': 0, 'checksum_err': 0, 'malformed': 0, 'errors': 0}

        # Atomic telemetry snapshots (see the module-level note).
        self.__imu = _ZERO_IMU
        self.__att = _ZERO_ATT
        self.__motion = _ZERO_MOTION
        self.__enc = _ZERO_ENC
        self.__seq_imu = 0
        self.__seq_att = 0
        self.__seq_motion = 0
        self.__seq_enc = 0
        # 'mpu9250' | 'icm20948' | None, latched from the first raw IMU packet.
        self.__imu_kind = None

        self.__read_id = 0
        self.__read_val = 0

        self.__read_arm_ok = 0
        self.__read_arm = [-1, -1, -1, -1, -1, -1]

        self.__version_H = 0
        self.__version_L = 0
        self.__version = 0
        self.__version_last_req = -1e9

        self.__pid_index = 0
        self.__kp1 = 0
        self.__ki1 = 0
        self.__kd1 = 0

        self.__arm_offset_state = 0
        self.__arm_offset_id = 0
        self.__arm_ctrl_enable = True

        self.__akm_def_angle = 100
        self.__akm_readed_angle = False
        self.__AKM_SERVO_ID = 0x01

        self.__read_car_type = 0

        if self.__debug:
            print("cmd_delay=" + str(self.__delay_time) + "s")

        if self.ser.isOpen():
            print("Rosmaster Serial Opened! Baudrate=115200")
        else:
            print("Serial Open Failed!")
        # 打开机械臂的扭矩力，避免6号舵机首次插上去读不到角度。
        self.set_uart_servo_torque(1)
        time.sleep(.002)

    def __del__(self):
        # The stock version closed the fd while the receive thread was blocked
        # in select() on it, which is undefined at the POSIX level and on Linux
        # yields EBADF -> a traceback (or a hang) during interpreter shutdown.
        # stop() wakes the reader first, then closes.
        try:
            if hasattr(self, 'ser'):
                self.stop()
        except Exception:
            pass

    # 根据数据帧的类型来做出对应的解析
    # According to the type of data frame to make the corresponding parsing
    def __parse_data(self, ext_type, ext_data):
        # print("parse_data:", ext_data, ext_type)
        if ext_type == self.FUNC_REPORT_SPEED:
            vx, vy, vz, battery = _S_SPEED.unpack_from(ext_data, 0)
            self.__seq_motion += 1
            self.__motion = MotionSample(vx / 1000.0, vy / 1000.0, vz / 1000.0,
                                         battery, time.monotonic(), self.__seq_motion)
        # 解析MPU9250原始陀螺仪、加速度计、磁力计数据
        # (MPU9250)the original gyroscope, accelerometer, magnetometer data
        elif ext_type == self.FUNC_REPORT_MPU_RAW:
            self.__imu_kind = 'mpu9250'
            self.__store_imu(ext_data, _GYRO_RATIO_MPU, _ACCEL_RATIO_MPU, _MAG_RATIO_MPU)
        # 解析ICM20948原始陀螺仪、加速度计、磁力计数据
        # (ICM20948)the original gyroscope, accelerometer, magnetometer data
        elif ext_type == self.FUNC_REPORT_ICM_RAW:
            self.__imu_kind = 'icm20948'
            self.__store_imu(ext_data, _GYRO_RATIO_ICM, _ACCEL_RATIO_ICM, _MAG_RATIO_ICM)
        # 解析板子的姿态角
        # the attitude Angle of the board
        elif ext_type == self.FUNC_REPORT_IMU_ATT:
            roll, pitch, yaw = _S_ATT.unpack_from(ext_data, 0)
            self.__seq_att += 1
            self.__att = AttSample(roll / 10000.0, pitch / 10000.0, yaw / 10000.0,
                                   time.monotonic(), self.__seq_att)
        # 解析四个轮子的编码器数据
        # Encoder data on all four wheels
        elif ext_type == self.FUNC_REPORT_ENCODER:
            m1, m2, m3, m4 = _S_ENC.unpack_from(ext_data, 0)
            self.__seq_enc += 1
            self.__enc = EncSample(m1, m2, m3, m4, time.monotonic(), self.__seq_enc)

        else:
            if ext_type == self.FUNC_UART_SERVO:
                self.__read_id, self.__read_val = _S_SERVO.unpack_from(ext_data, 0)
                if self.__debug:
                    print("FUNC_UART_SERVO:", self.__read_id, self.__read_val)

            elif ext_type == self.FUNC_ARM_CTRL:
                self.__read_arm = list(_S_ARM.unpack_from(ext_data, 0))
                self.__read_arm_ok = 1
                if self.__debug:
                    print("FUNC_ARM_CTRL:", self.__read_arm)

            elif ext_type == self.FUNC_VERSION:
                self.__version_H, self.__version_L = _S_2B.unpack_from(ext_data, 0)
                if self.__debug:
                    print("FUNC_VERSION:", self.__version_H, self.__version_L)

            elif ext_type == self.FUNC_SET_MOTOR_PID:
                (self.__pid_index, self.__kp1,
                 self.__ki1, self.__kd1) = _S_PID.unpack_from(ext_data, 0)
                if self.__debug:
                    print("FUNC_SET_MOTOR_PID:", self.__pid_index, [self.__kp1, self.__ki1, self.__kd1])

            elif ext_type == self.FUNC_SET_YAW_PID:
                (self.__pid_index, self.__kp1,
                 self.__ki1, self.__kd1) = _S_PID.unpack_from(ext_data, 0)
                if self.__debug:
                    print("FUNC_SET_YAW_PID:", self.__pid_index, [self.__kp1, self.__ki1, self.__kd1])

            elif ext_type == self.FUNC_ARM_OFFSET:
                self.__arm_offset_id, self.__arm_offset_state = _S_2B.unpack_from(ext_data, 0)
                if self.__debug:
                    print("FUNC_ARM_OFFSET:", self.__arm_offset_id, self.__arm_offset_state)

            elif ext_type == self.FUNC_AKM_DEF_ANGLE:
                id, self.__akm_def_angle = _S_2B.unpack_from(ext_data, 0)
                self.__akm_readed_angle = True
                if self.__debug:
                    print("FUNC_AKM_DEF_ANGLE:", id, self.__akm_def_angle)

            elif ext_type == self.FUNC_SET_CAR_TYPE:
                self.__read_car_type = _S_1B.unpack_from(ext_data, 0)[0]

    # 解析9轴IMU原始数据，两个分支共用同一套符号约定
    # Decode a 9-axis raw IMU packet. Both the MPU9250 and ICM20948 branches
    # route through here so they cannot drift apart in sign or units again.
    def __store_imu(self, ext_data, gyro_ratio, accel_ratio, mag_ratio):
        v = _S_IMU9.unpack_from(ext_data, 0)
        gsx, gsy, gsz = _GYRO_SIGNS
        asx, asy, asz = _ACCEL_SIGNS
        msx, msy, msz = _MAG_SIGNS
        self.__seq_imu += 1
        self.__imu = ImuSample(
            v[0] * gyro_ratio * gsx,      # gyro, rad/s
            v[1] * gyro_ratio * gsy,
            v[2] * gyro_ratio * gsz,
            v[3] * accel_ratio * asx,     # accel, m/s^2
            v[4] * accel_ratio * asy,
            v[5] * accel_ratio * asz,
            v[6] * mag_ratio * msx,       # magnetic field, tesla
            v[7] * mag_ratio * msy,
            v[8] * mag_ratio * msz,
            time.monotonic(), self.__seq_imu)


    # 接收数据 receive data
    def __receive_data(self):
        """Read bytes in bursts and hand complete frames to the parser.

        Replaces a byte-at-a-time reader that (a) had no exception handling at
        all, so any port error killed the thread and froze every getter at its
        last value forever, and (b) resynchronised by discarding TWO bytes, so a
        0xFF that was itself the start of the next frame was thrown away along
        with the false header.
        """
        buf = bytearray()
        backoff = 0.0
        while not self.__rx_stop.is_set():
            try:
                # in_waiting is a cheap ioctl. When it reports nothing pending we
                # fall back to a 1-byte read that blocks up to the port timeout,
                # so the loop both sleeps efficiently and wakes often enough to
                # re-check the stop flag.
                pending = self.ser.in_waiting
                chunk = self.ser.read(pending if pending else 1)
                backoff = 0.0
            except Exception as e:   # SerialException, OSError, ...
                if self.__rx_stop.is_set():
                    break
                self.__rx_error = e
                self.__rx_stats['errors'] += 1
                # Bounded backoff matters: read() on a disconnected fd returns
                # immediately, so a bare retry would spin a core at 100%.
                backoff = 0.05 if backoff == 0.0 else min(backoff * 2.0, 1.0)
                _LOG.warning("Rosmaster RX error: %s (retry in %.2fs)", e, backoff)
                self.__rx_stop.wait(backoff)   # interruptible sleep
                continue

            if not chunk:
                continue                       # read timeout, no data
            buf += chunk
            self.__consume_frames(buf)

            # Pure noise must never grow the buffer without bound.
            if len(buf) > 4 * _RX_MAX_FRAME:
                del buf[:-_RX_MAX_FRAME]

    def __consume_frames(self, buf):
        """Pull every complete, checksum-valid frame out of `buf`, in place."""
        HEAD = self.__HEAD
        DEV = self.__DEVICE_ID - 1
        while True:
            i = buf.find(HEAD)
            if i < 0:
                del buf[:]                     # no candidate header at all
                return
            if i:
                del buf[:i]                    # drop leading garbage
            if len(buf) < 4:
                return                         # need HEAD, DEV, len, type
            if buf[1] != DEV:
                del buf[:1]                    # resync by ONE byte, not two
                continue
            ext_len = buf[2]
            # ext_len < 3 would make the payload length negative; the stock
            # parser accepted it, produced an empty payload whose checksum
            # matched ~1 frame in 256, and then died in struct.unpack.
            if ext_len < 3 or ext_len > _RX_MAX_FRAME:
                del buf[:1]                    # implausible length -> false header
                continue
            total = ext_len + 2                # HEAD DEV len type payload... ck
            if len(buf) < total:
                return                         # incomplete: wait for more bytes
            ext_type = buf[3]
            payload = bytes(buf[4:total - 1])  # ext_len-3 bytes, checksum excluded
            rx_check = buf[total - 1]
            if (ext_len + ext_type + sum(payload)) & 0xFF == rx_check:
                del buf[:total]
                self.__rx_stats['frames'] += 1
                self.__rx_last_frame_t = time.monotonic()
                try:
                    self.__parse_data(ext_type, payload)
                except (struct.error, IndexError, ValueError) as e:
                    self.__rx_stats['malformed'] += 1
                    if self.__debug:
                        print("parse error:", ext_type, payload, e)
            else:
                self.__rx_stats['checksum_err'] += 1
                if self.__debug:
                    print("check sum error:", ext_len, ext_type, payload)
                # Resync by one byte. Dropping the whole candidate frame (what
                # the stock parser did) can eat a real header that was sitting
                # inside the bytes a desync had misread as payload.
                del buf[:1]

    # 接收线程健康状态 receive-thread health
    def rx_healthy(self, max_age=0.5):
        """True if a well-formed frame was parsed within `max_age` seconds.

        Consumers should gate publishing on this: a frozen sensor that keeps
        being republished with a fresh timestamp is far more damaging to an EKF
        than a gap in the data.
        """
        return (time.monotonic() - self.__rx_last_frame_t) < max_age

    def rx_stats(self):
        """Snapshot of RX counters: frames, checksum_err, malformed, errors."""
        stats = dict(self.__rx_stats)
        stats['last_frame_age'] = time.monotonic() - self.__rx_last_frame_t
        stats['last_error'] = repr(self.__rx_error) if self.__rx_error else None
        return stats

    # 请求数据， function：对应要返回数据的功能字，parm：传入的参数。
    # Request data, function: corresponding function word to return data, parm: parameter passed in
    def __request_data(self, function, param=0):
        cmd = [self.__HEAD, self.__DEVICE_ID, 0x05, self.FUNC_REQUEST_DATA, int(function) & 0xff, int(param) & 0xff]
        checksum = sum(cmd, self.__COMPLEMENT) & 0xff
        cmd.append(checksum)
        self.ser.write(cmd)
        if self.__debug:
            print("request:", cmd)
        time.sleep(0.002)

    # 机械臂转化角度成位置脉冲（写入角度）
    # Arm converts Angle to position pulse
    def __arm_convert_value(self, s_id, s_angle):
        value = -1
        if s_id == 1:
            value = int((3100 - 900) * (s_angle - 180) / (0 - 180) + 900)
        elif s_id == 2:
            value = int((3100 - 900) * (s_angle - 180) / (0 - 180) + 900)
        elif s_id == 3:
            value = int((3100 - 900) * (s_angle - 180) / (0 - 180) + 900)
        elif s_id == 4:
            value = int((3100 - 900) * (s_angle - 180) / (0 - 180) + 900)
        elif s_id == 5:
            value = int((3700 - 380) * (s_angle - 0) / (270 - 0) + 380)
        elif s_id == 6:
            value = int((3100 - 900) * (s_angle - 0) / (180 - 0) + 900)
        return value

    # 机械臂转化位置脉冲成角度（读取角度）
    # Arm converts position pulses into angles
    def __arm_convert_angle(self, s_id, s_value):
        s_angle = -1
        if s_id == 1:
            s_angle = int((s_value - 900) * (0 - 180) / (3100 - 900) + 180 + 0.5)
        elif s_id == 2:
            s_angle = int((s_value - 900) * (0 - 180) / (3100 - 900) + 180 + 0.5)
        elif s_id == 3:
            s_angle = int((s_value - 900) * (0 - 180) / (3100 - 900) + 180 + 0.5)
        elif s_id == 4:
            s_angle = int((s_value - 900) * (0 - 180) / (3100 - 900) + 180 + 0.5)
        elif s_id == 5:
            s_angle = int((270 - 0) * (s_value - 380) / (3700 - 380) + 0 + 0.5)
        elif s_id == 6:
            s_angle = int((180 - 0) * (s_value - 900) / (3100 - 900) + 0 + 0.5)
        return s_angle

    # 限制电机输入的PWM占空比数值，value=127则保持原来的数据，不修改当前电机速度
    # Limit the PWM duty ratio value of motor input, value=127, keep the original data, do not modify the current motor speed  
    def __limit_motor_value(self, value):
        if value == 127:
            return 127
        elif value > 100:
           return 100
        elif value < -100:
            return -100
        else:
            return int(value)

    # 开启接收和处理数据的线程
    # Start the thread that receives and processes data
    def create_receive_threading(self):
        if self.__uart_state != 0:
            return True
        self.__rx_stop.clear()
        try:
            self.__rx_thread = threading.Thread(
                target=self.__receive_data,
                name="rosmaster_rx",
                daemon=True)          # setDaemon() is deprecated since 3.10
            self.__rx_thread.start()
        except RuntimeError as e:
            # The stock version swallowed this and left __uart_state at 0, so a
            # failed start was indistinguishable from a successful one.
            _LOG.error("Rosmaster: failed to start receive thread: %s", e)
            self.__rx_thread = None
            return False
        self.__uart_state = 1
        _LOG.info("Rosmaster receive thread started")
        return True

    # 停止接收线程并关闭串口 stop the receive thread and close the port
    def stop(self, timeout=1.0):
        """Stop the receive thread and close the port. Idempotent."""
        self.__rx_stop.set()
        try:
            # pyserial >=3.1: writes to the abort pipe that read()'s select()
            # also waits on, which is the only clean way to wake a blocked read.
            self.ser.cancel_read()
        except Exception:
            pass
        t = self.__rx_thread
        if t is not None and t.is_alive():
            t.join(timeout)
            if t.is_alive():
                _LOG.warning("Rosmaster receive thread did not exit in %.1fs", timeout)
        self.__rx_thread = None
        self.__uart_state = 0
        try:
            if self.ser.is_open:
                self.ser.close()
        except Exception as e:
            _LOG.warning("Rosmaster: error closing serial port: %s", e)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.stop()
        return False


    # 单片机自动返回数据状态位，默认为开启，如果设置关闭会影响部分读取数据功能。
    # enable=True,底层扩展板会每隔10毫秒发送一包数据，总共四包不同数据，所以每包数据每40毫秒刷新一次。enable=False，则不发送。
    # forever=True永久保存，=False临时作用。
    # The MCU automatically returns the data status bit, which is enabled by default. If the switch is closed, the data reading function will be affected.  
    # enable=True, The underlying expansion board sends four different packets of data every 10 milliseconds, so each packet is refreshed every 40 milliseconds. 
    # If enable=False, the report is not sent.  
    # forever=True for permanent, =False for temporary
    def set_auto_report_state(self, enable, forever=False):
        try:
            state1 = 0
            state2 = 0
            if enable:
                state1 = 1
            if forever:
                state2 = 0x5F
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x05, self.FUNC_AUTO_REPORT, state1, state2]
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("report:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_auto_report_state error!---')
            pass

    # 蜂鸣器开关，on_time=0：关闭，on_time=1：一直响，
    # on_time>=10：响xx毫秒后自动关闭（on_time是10的倍数）。
    # Buzzer switch. On_time =0: the buzzer is off. On_time =1: the buzzer keeps ringing
    # On_time >=10: automatically closes after xx milliseconds (on_time is a multiple of 10)
    def set_beep(self, on_time):
        try:
            if on_time < 0:
                print("beep input error!")
                return
            value = bytearray(struct.pack('h', int(on_time)))

            cmd = [self.__HEAD, self.__DEVICE_ID, 0x05, self.FUNC_BEEP, value[0], value[1]]
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("beep:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_beep error!---')
            pass

    # 舵机控制，servo_id：对应ID编号，angle：对应舵机角度值
    # servo_id=[1, 4], angle=[0, 180]
    # Servo control, servo_id: corresponding, Angle: corresponding servo Angle value
    def set_pwm_servo(self, servo_id, angle):
        try:
            if servo_id < 1 or servo_id > 4:
                if self.__debug:
                    print("set_pwm_servo input invalid")
                return
            if angle > 180:
                angle = 180
            elif angle < 0:
                angle = 0
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_PWM_SERVO, int(servo_id), int(angle)]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("pwmServo:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_pwm_servo error!---')
            pass

    # 同时控制四路PWM的角度，angle_sX=[0, 180]
    # At the same time control four PWM Angle, angle_sX=[0, 180]
    def set_pwm_servo_all(self, angle_s1, angle_s2, angle_s3, angle_s4):
        try:
            if angle_s1 < 0 or angle_s1 > 180:
                angle_s1 = 255
            if angle_s2 < 0 or angle_s2 > 180:
                angle_s2 = 255
            if angle_s3 < 0 or angle_s3 > 180:
                angle_s3 = 255
            if angle_s4 < 0 or angle_s4 > 180:
                angle_s4 = 255
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_PWM_SERVO_ALL, \
                   int(angle_s1), int(angle_s2), int(angle_s3), int(angle_s4)]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("all Servo:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_pwm_servo_all error!---')
            pass
    
    # RGB可编程灯带控制，可单独控制或全体控制，控制前需要先停止RGB灯特效。
    # led_id=[0, 13]，控制对应编号的RGB灯；led_id=0xFF, 控制所有灯。
    # red,green,blue=[0, 255]，表示颜色RGB值。
    # RGB programmable light belt control, can be controlled individually or collectively, before control need to stop THE RGB light effect.
    # Led_id =[0, 13], control the CORRESPONDING numbered RGB lights;  Led_id =0xFF, controls all lights.
    # Red,green,blue=[0, 255], indicating the RGB value of the color.
    def set_colorful_lamps(self, led_id, red, green, blue):
        try:
            id = int(led_id) & 0xff
            r = int(red) & 0xff
            g = int(green) & 0xff
            b = int(blue) & 0xff
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_RGB, id, r, g, b]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("rgb:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_colorful_lamps error!---')
            pass

    # RGB可编程灯带特效展示。
    # effect=[0, 6]，0：停止灯效，1：流水灯，2：跑马灯，3：呼吸灯，4：渐变灯，5：星光点点，6：电量显示
    # speed=[1, 10]，数值越小速度变化越快。
    # parm，可不填，作为附加参数。用法1：呼吸灯效果传入[0, 6]可修改呼吸灯颜色。
    # RGB programmable light band special effects display.
    # Effect =[0, 6], 0: stop light effect, 1: running light, 2: running horse light, 3: breathing light, 4: gradient light, 5: starlight, 6: power display 
    # Speed =[1, 10], the smaller the value, the faster the speed changes
    # Parm, left blank, as an additional argument.  Usage 1: The color of breathing lamp can be modified by the effect of breathing lamp [0, 6]
    def set_colorful_effect(self, effect, speed=255, parm=255):
        try:
            eff = int(effect) & 0xff
            spe = int(speed) & 0xff
            par = int(parm) & 0xff
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_RGB_EFFECT, eff, spe, par]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("rgb_effect:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_colorful_effect error!---')
            pass


    # 控制电机PWM脉冲，从而控制速度（未使用编码器测速）。speed_X=[-100, 100]
    # Control PWM pulse of motor to control speed (speed measurement without encoder). speed_X=[-100, 100]
    def set_motor(self, speed_1, speed_2, speed_3, speed_4):
        try:
            t_speed_a = bytearray(struct.pack('b', self.__limit_motor_value(speed_1)))
            t_speed_b = bytearray(struct.pack('b', self.__limit_motor_value(speed_2)))
            t_speed_c = bytearray(struct.pack('b', self.__limit_motor_value(speed_3)))
            t_speed_d = bytearray(struct.pack('b', self.__limit_motor_value(speed_4)))
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_MOTOR,
                   t_speed_a[0], t_speed_b[0], t_speed_c[0], t_speed_d[0]]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("motor:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_motor error!---')
            pass


    # 控制小车向前、向后、向左、向右等运动。
    # state=[0, 7],=0停止,=1前进,=2后退,=3向左,=4向右,=5左旋,=6右旋,=7停车
    # speed=[-100, 100]，=0停止。
    # adjust=True开启陀螺仪辅助运动方向。=False则不开启。(此功能未开通)
    # Control the car forward, backward, left, right and other movements.
    # State =[0~6],=0 stop,=1 forward,=2 backward,=3 left,=4 right,=5 spin left,=6 spin right
    # Speed =[-100, 100], =0 Stop.
    # Adjust =True Activate the gyroscope auxiliary motion direction.  If =False, the function is disabled.(This function is not enabled)
    def set_car_run(self, state, speed, adjust=False):
        try:
            car_type = self.__CAR_TYPE
            if adjust:
                car_type = car_type | self.__CAR_ADJUST
            t_speed = bytearray(struct.pack('h', int(speed)))
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_CAR_RUN, \
                car_type, int(state&0xff), t_speed[0], t_speed[1]]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("car_run:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_car_run error!---')
            pass

    # 小车运动控制, 
    # Car movement control
    def set_car_motion(self, v_x, v_y, v_z):
        '''
        输入范围 input range: 
        X3: v_x=[-1.0, 1.0], v_y=[-1.0, 1.0], v_z=[-5, 5]
        X3PLUS: v_x=[-0.7, 0.7], v_y=[-0.7, 0.7], v_z=[-3.2, 3.2]
        R2/R2L: v_x=[-1.8, 1.8], v_y=[-0.045, 0.045], v_z=[-3, 3]
        '''
        try:
            vx_parms = bytearray(struct.pack('h', int(v_x*1000)))
            vy_parms = bytearray(struct.pack('h', int(v_y*1000)))
            vz_parms = bytearray(struct.pack('h', int(v_z*1000)))
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_MOTION, self.__CAR_TYPE, \
                vx_parms[0], vx_parms[1], vy_parms[0], vy_parms[1], vz_parms[0], vz_parms[1]]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("motion:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_car_motion error!---')
            pass


    # PID 参数控制，会影响set_car_motion函数控制小车的运动速度变化情况。默认情况下可不调整。
    # kp ki kd = [0, 10.00], 可输入小数。
    # forever=True永久保存，=False临时作用。
    # 由于永久保存需要写入芯片flash中，操作时间较长，所以加入delay延迟时间，避免导致单片机丢包的问题。
    # 临时作用反应快，单次有效，重启单片后数据不再保持。
    # PID parameter control will affect the set_CAR_motion function to control the speed change of the car.  This parameter is optional by default.  
    # KP ki kd = [0, 10.00]  
    # forever=True for permanent, =False for temporary.  
    # Since permanent storage needs to be written into the chip flash, which takes a long time to operate, delay is added to avoid packet loss caused by MCU.  
    # Temporary effect fast response, single effective, data will not be maintained after restarting the single chip
    def set_pid_param(self, kp, ki, kd, forever=False):
        try:
            state = 0
            if forever:
                state = 0x5F
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x0A, self.FUNC_SET_MOTOR_PID]
            if kp > 10 or ki > 10 or kd > 10 or kp < 0 or ki < 0 or kd < 0:
                print("PID value must be:[0, 10.00]")
                return
            kp_params = bytearray(struct.pack('h', int(kp * 1000)))
            ki_params = bytearray(struct.pack('h', int(ki * 1000)))
            kd_params = bytearray(struct.pack('h', int(kd * 1000)))
            cmd.append(kp_params[0])  # low
            cmd.append(kp_params[1])  # high
            cmd.append(ki_params[0])  # low
            cmd.append(ki_params[1])  # high
            cmd.append(kd_params[0])  # low
            cmd.append(kd_params[1])  # high
            cmd.append(state)
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("pid:", cmd)
            time.sleep(self.__delay_time)
            if forever:
                time.sleep(.1)
        except:
            print('---set_pid_param error!---')
            pass
    
    # 设置偏航角调节的PID
    # Set the PID for yaw Angle adjustment
    # def set_yaw_pid_param(self, kp, ki, kd, forever=False):
    #     try:
    #         state = 0
    #         if forever:
    #             state = 0x5F
    #         cmd = [self.__HEAD, self.__DEVICE_ID, 0x0A, self.FUNC_SET_YAW_PID]
    #         if kp > 10 or ki > 10 or kd > 10 or kp < 0 or ki < 0 or kd < 0:
    #             print("YAW PID value must be:[0, 10.00]")
    #             return
    #         kp_params = bytearray(struct.pack('h', int(kp * 1000)))
    #         ki_params = bytearray(struct.pack('h', int(ki * 1000)))
    #         kd_params = bytearray(struct.pack('h', int(kd * 1000)))
    #         cmd.append(kp_params[0])  # low
    #         cmd.append(kp_params[1])  # high
    #         cmd.append(ki_params[0])  # low
    #         cmd.append(ki_params[1])  # high
    #         cmd.append(kd_params[0])  # low
    #         cmd.append(kd_params[1])  # high
    #         cmd.append(state)
    #         checksum = sum(cmd, self.__COMPLEMENT) & 0xff
    #         cmd.append(checksum)
    #         self.ser.write(cmd)
    #         if self.__debug:
    #             print("pid:", cmd)
    #         time.sleep(self.__delay_time)
    #         if forever:
    #             time.sleep(.1)
    #     except:
    #         print('---set_pid_param error!---')
    #         pass

    # 设置小车类型
    # Set car Type
    def set_car_type(self, car_type):
        if str(car_type).isdigit():
            self.__CAR_TYPE = int(car_type) & 0xff
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_SET_CAR_TYPE, self.__CAR_TYPE, 0x5F]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("car_type:", cmd)
            time.sleep(.1)
        else:
            print("set_car_type input invalid")

    # 控制总线舵机。servo_id:[1-255],表示要控制的舵机的ID号, id=254时, 控制所有已连接舵机。
    # pulse_value=[96,4000]表示舵机要运行到的位置。
    # run_time表示运行的时间(ms),时间越短,舵机转动越快。最小为0，最大为2000
    # Control bus steering gear.  Servo_id :[1-255], indicating the ID of the steering gear to be controlled. If ID =254, control all connected steering gear.  
    # pulse_value=[96,4000] indicates the position to which the steering gear will run.  
    # run_time indicates the running time (ms). The shorter the time, the faster the steering gear rotates.  The minimum value is 0 and the maximum value is 2000
    def set_uart_servo(self, servo_id, pulse_value, run_time=500):
        try:
            if not self.__arm_ctrl_enable:
                return
            if servo_id < 1 or pulse_value < 96 or pulse_value > 4000 or run_time < 0:
                print("set uart servo input error")
                return
            if run_time > 2000:
                run_time = 2000
            if run_time < 0:
                run_time = 0
            s_id = int(servo_id) & 0xff
            value = bytearray(struct.pack('h', int(pulse_value)))
            r_time = bytearray(struct.pack('h', int(run_time)))

            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_UART_SERVO, \
                s_id, value[0], value[1], r_time[0], r_time[1]]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("uartServo:", servo_id, int(pulse_value), cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_uart_servo error!---')
            pass

    # 设置总线舵机角度接口：s_id:[1,6], s_angle: 1-4:[0, 180], 5:[0, 270], 6:[0, 180], 设置舵机要运动到的角度。
    # run_time表示运行的时间(ms),时间越短,舵机转动越快。最小为0，最大为2000
    # Set bus steering gear Angle interface: s_id:[1,6], s_angle: 1-4:[0, 180], 5:[0, 270], 6:[0, 180], set steering gear to move to the Angle.  
    # run_time indicates the running time (ms). The shorter the time, the faster the steering gear rotates.  The minimum value is 0 and the maximum value is 2000
    def set_uart_servo_angle(self, s_id, s_angle, run_time=500):
        try:
            if s_id == 1:
                if 0 <= s_angle <= 180:
                    value = self.__arm_convert_value(s_id, s_angle)
                    self.set_uart_servo(s_id, value, run_time)
                else:
                    print("angle_1 set error!")
            elif s_id == 2:
                if 0 <= s_angle <= 180:
                    value = self.__arm_convert_value(s_id, s_angle)
                    self.set_uart_servo(s_id, value, run_time)
                else:
                    print("angle_2 set error!")
            elif s_id == 3:
                if 0 <= s_angle <= 180:
                    value = self.__arm_convert_value(s_id, s_angle)
                    self.set_uart_servo(s_id, value, run_time)
                else:
                    print("angle_3 set error!")
            elif s_id == 4:
                if 0 <= s_angle <= 180:
                    value = self.__arm_convert_value(s_id, s_angle)
                    self.set_uart_servo(s_id, value, run_time)
                else:
                    print("angle_4 set error!")
            elif s_id == 5:
                if 0 <= s_angle <= 270:
                    value = self.__arm_convert_value(s_id, s_angle)
                    self.set_uart_servo(s_id, value, run_time)
                else:
                    print("angle_5 set error!")
            elif s_id == 6:
                if 0 <= s_angle <= 180:
                    value = self.__arm_convert_value(s_id, s_angle)
                    self.set_uart_servo(s_id, value, run_time)
                else:
                    print("angle_6 set error!")
        except:
            print('---set_uart_servo_angle error! ID=%d---' % s_id)
            pass

    # 设置总线舵机的ID号(谨慎使用)，servo_id=[1-250]。
    # 运行此函数前请确认只连接一个总线舵机，否则会把所有已连接的总线舵机都设置成同一个ID，造成控制混乱。
    # Set the bus servo ID(Use with caution), servo_id=[1-250].  
    # Before running this function, please confirm that only one bus actuator is connected. Otherwise, all connected bus actuators will be set to the same ID, resulting in confusion of control
    def set_uart_servo_id(self, servo_id):
        try:
            if servo_id < 1 or servo_id > 250:
                print("servo id input error!")
                return
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x04, self.FUNC_UART_SERVO_ID, int(servo_id)]
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("uartServo_id:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_uart_servo_id error!---')
            pass

    # 关闭/打开总线舵机扭矩力, enable=[0, 1]。
    # enable=0:关闭舵机扭矩力，可以用手转动舵机，但命令无法控制转动；
    # enable=1：打开扭矩力，命令可以控制转动，不可以用手转动舵机。
    # Turn off/on the bus steering gear torque force, enable=[0, 1].  
    # enable=0: Turn off the torque force of the steering gear, the steering gear can be turned by hand, but the command cannot control the rotation;  
    # enable=1: Turn on torque force, command can control rotation, can not turn steering gear by hand
    def set_uart_servo_torque(self, enable):
        try:
            if enable > 0:
                on = 1
            else:
                on = 0
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x04, self.FUNC_UART_SERVO_TORQUE, on]
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("uartServo_torque:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_uart_servo_torque error!---')
            pass

    # 设置机械臂控制开关，enable=True正常发送控制协议，=False不发送控制协议
    # Set the control switch of the manipulator. Enable =True Indicates that the control protocol is normally sent; False indicates that the control protocol is not sent
    def set_uart_servo_ctrl_enable(self, enable):
        if enable:
            self.__arm_ctrl_enable = True
        else:
            self.__arm_ctrl_enable = False



    # 同时控制机械臂所有舵机的角度。
    # Meanwhile, the Angle of all steering gear of the manipulator is controlled
    def set_uart_servo_angle_array(self, angle_s=[90, 90, 90, 90, 90, 180], run_time=500):
        try:
            if not self.__arm_ctrl_enable:
                return
            if 0 <= angle_s[0] <= 180 and 0 <= angle_s[1] <= 180 and 0 <= angle_s[2] <= 180 and \
                0 <= angle_s[3] <= 180 and 0 <= angle_s[4] <= 270 and 0 <= angle_s[5] <= 180:
                if run_time > 2000:
                    run_time = 2000
                if run_time < 0:
                    run_time = 0
                temp_val = [0, 0, 0, 0, 0, 0]
                for i in range(6):
                    temp_val[i] = self.__arm_convert_value(i+1, angle_s[i])
                    
                value_s1 = bytearray(struct.pack('h', int(temp_val[0])))
                value_s2 = bytearray(struct.pack('h', int(temp_val[1])))
                value_s3 = bytearray(struct.pack('h', int(temp_val[2])))
                value_s4 = bytearray(struct.pack('h', int(temp_val[3])))
                value_s5 = bytearray(struct.pack('h', int(temp_val[4])))
                value_s6 = bytearray(struct.pack('h', int(temp_val[5])))

                r_time = bytearray(struct.pack('h', int(run_time)))
                cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_ARM_CTRL, \
                       value_s1[0], value_s1[1], value_s2[0], value_s2[1], value_s3[0], value_s3[1], \
                       value_s4[0], value_s4[1], value_s5[0], value_s5[1], value_s6[0], value_s6[1], \
                       r_time[0], r_time[1]]
                cmd[2] = len(cmd) - 1
                checksum = sum(cmd, self.__COMPLEMENT) & 0xff
                cmd.append(checksum)
                self.ser.write(cmd)
                if self.__debug:
                    print("arm:", cmd)
                    print("value:", temp_val)
                time.sleep(self.__delay_time)
            else:
                print("angle_s input error!")
        except:
            print('---set_uart_servo_angle_array error!---')
            pass


    # 设置机械臂的中位偏差，servo_id=0~6， =0全部恢复出厂默认值
    # Run the following command to set the mid-bit deviation of the manipulator: servo_id=0 to 6, =0 Restore the factory default values
    def set_uart_servo_offset(self, servo_id):
        try:
            self.__arm_offset_id = 0xff
            self.__arm_offset_state = 0
            s_id = int(servo_id) & 0xff
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_ARM_OFFSET, s_id]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("uartServo_offset:", cmd)
            time.sleep(self.__delay_time)
            for i in range(200):
                if self.__arm_offset_id == servo_id:
                    if self.__debug:
                        if self.__arm_offset_id == 0:
                            print("Arm Reset Offset Value")
                        else:
                            print("Arm Offset State:", self.__arm_offset_id, self.__arm_offset_state, i)
                    return self.__arm_offset_state
                time.sleep(.001)
            return self.__arm_offset_state
        except:
            print('---set_uart_servo_offset error!---')
            pass

    # 设置阿克曼类型(R2)小车前轮的默认角度，angle=[60, 120]
    # forever=True永久保存，=False临时作用。
    # 由于永久保存需要写入芯片flash中，操作时间较长，所以加入delay延迟时间，避免导致单片机丢包的问题。
    # 临时作用反应快，单次有效，重启单片后数据不再保持。
    # Set the default Angle of akerman type (R2) car front wheel, Angle =[60, 120]
    # forever=True for permanent, =False for temporary.
    # Since permanent storage needs to be written into the chip flash, which takes a long time to operate, delay is added to avoid packet loss caused by MCU.  
    # Temporary effect fast response, single effective, data will not be maintained after restarting the single chip
    def set_akm_default_angle(self, angle, forever=False):
        try:
            if int(angle) > 120 or int(angle) < 60:
                return
            id = self.__AKM_SERVO_ID
            state = 0
            if forever:
                state = 0x5F
                self.__akm_def_angle = angle
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_AKM_DEF_ANGLE, id, int(angle), state]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("akm set def angle:", cmd)
            time.sleep(self.__delay_time)
            if forever:
                time.sleep(.1)
        except:
            print('---set_akm_default_angle error!---')
            pass

    # 控制阿克曼类型(R2)小车相对于默认角度的转向角，向左为负数，向右为正数，angle=[-45, 45]
    # ctrl_car=False，只控制舵机角度，=True，控制舵机角度同时修改左右电机的速度。
    # Control the steering Angle of ackman type (R2) car relative to the default Angle, negative for left and positive for right, Angle =[-45, 45]
    # ctrl_car=False, only control the steering gear Angle, =True, control the steering gear Angle and modify the speed of the left and right motors.
    def set_akm_steering_angle(self, angle, ctrl_car=False):
        try:
            if int(angle) > 45 or int(angle) < -45:
                return
            id = self.__AKM_SERVO_ID
            if ctrl_car:
                id = self.__AKM_SERVO_ID + 0x80
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x00, self.FUNC_AKM_STEER_ANGLE, id, int(angle)&0xFF]
            cmd[2] = len(cmd) - 1
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("akm_steering_angle:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---set_akm_steering_angle error!---')
            pass


    # 重置小车flash保存的数据，恢复出厂默认值。
    # Reset the car flash saved data, restore the factory default value
    def reset_flash_value(self):
        try:
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x04, self.FUNC_RESET_FLASH, 0x5F]
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("flash:", cmd)
            time.sleep(self.__delay_time)
            time.sleep(.1)
        except:
            print('---reset_flash_value error!---')
            pass
    
    # 重置小车状态，包括停车，关灯，关蜂鸣器
    # Reset car status, including parking, lights off, buzzer off
    def reset_car_state(self):
        try:
            cmd = [self.__HEAD, self.__DEVICE_ID, 0x04, self.FUNC_RESET_STATE, 0x5F]
            checksum = sum(cmd, self.__COMPLEMENT) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            if self.__debug:
                print("reset_car_state:", cmd)
            time.sleep(self.__delay_time)
        except:
            print('---reset_car_state error!---')
            pass

    # 清除单片机自动发送过来的缓存数据
    # Clear the cache data automatically sent by the MCU
    def clear_auto_report_data(self):
        # Matches the original, which deliberately does NOT clear the encoders.
        self.__imu = _ZERO_IMU
        self.__att = _ZERO_ATT
        self.__motion = _ZERO_MOTION

    # 读取阿克曼类型(R2)小车前轮舵机默认角度。
    def get_akm_default_angle(self):
        if not self.__akm_readed_angle:
            self.__request_data(self.FUNC_AKM_DEF_ANGLE, self.__AKM_SERVO_ID)
            akm_count = 0
            while True:
                if self.__akm_readed_angle:
                    break
                akm_count = akm_count + 1
                if akm_count > 100:
                    return -1
                time.sleep(.01)
        return self.__akm_def_angle



    # 读取总线舵机位置参数, servo_id=[1-250], 返回：读到的ID，当前位置参数
    # Read bus servo position parameters, servo_id=[1-250], return: read ID, current position parameters
    def get_uart_servo_value(self, servo_id):
        try:
            if servo_id < 1 or servo_id > 250:
                print("get servo id input error!")
                return
            self.__read_id = 0
            self.__read_val = 0
            self.__request_data(self.FUNC_UART_SERVO, int(servo_id) & 0xff)
            timeout = 30
            while timeout > 0:
                if self.__read_id > 0:
                    return self.__read_id, self.__read_val
                timeout = timeout - 1
                time.sleep(.001)
            return -1, -1
        except:
            print('---get_uart_servo_value error!---')
            return -2, -2

    # 读取总线舵机的角度，s_id表示要读取的舵机的ID号，s_id=[1-6]
    # Read the Angle of the bus steering gear, s_id indicates the ID number of the steering gear to be read, s_id=[1-6]
    def get_uart_servo_angle(self, s_id):
        try:
            angle = -1
            read_id, value = self.get_uart_servo_value(s_id)
            if s_id == 1 and read_id == 1:
                angle = self.__arm_convert_angle(s_id, value)
                if angle > 180 or angle < 0:
                    if self.__debug:
                        print("read servo:%d out of range!" % s_id)
                    angle = -1
            elif s_id == 2 and read_id == 2:
                angle = self.__arm_convert_angle(s_id, value)
                if angle > 180 or angle < 0:
                    if self.__debug:
                        print("read servo:%d out of range!" % s_id)
                    angle = -1
            elif s_id == 3 and read_id == 3:
                angle = self.__arm_convert_angle(s_id, value)
                if angle > 180 or angle < 0:
                    if self.__debug:
                        print("read servo:%d out of range!" % s_id)
                    angle = -1
            elif s_id == 4 and read_id == 4:
                angle = self.__arm_convert_angle(s_id, value)
                if angle > 180 or angle < 0:
                    if self.__debug:
                        print("read servo:%d out of range!" % s_id)
                    angle = -1
            elif s_id == 5 and read_id == 5:
                angle = self.__arm_convert_angle(s_id, value)
                if angle > 270 or angle < 0:
                    if self.__debug:
                        print("read servo:%d out of range!" % s_id)
                    angle = -1
            elif s_id == 6 and read_id == 6:
                angle = self.__arm_convert_angle(s_id, value)
                if angle > 180 or angle < 0:
                    if self.__debug:
                        print("read servo:%d out of range!" % s_id)
                    angle = -1
            else:
                if self.__debug:
                    print("read servo:%d error!" % s_id)
            if self.__debug:
                print("request angle %d: %d, %d" % (s_id, read_id, value))
            return angle
        except:
            print('---get_uart_servo_angle error!---')
            return -2

    # 一次性读取六个舵机的角度[xx, xx, xx, xx, xx, xx]，如果某个舵机错误则那一位为-1
    # Read the angles of three steering gear [xx, xx, xx, xx, xx, xx] at one time. If one steering gear is wrong, that one is -1
    def get_uart_servo_angle_array(self):
        try:
            # angle = [-1, -1, -1, -1, -1, -1]
            # for i in range(6):
            #     temp1 = self.get_uart_servo_angle(i + 1)
            #     if temp1 >= 0:
            #         angle[i] = temp1
            #     else:
            #         break
            # return angle

            angle = [-1, -1, -1, -1, -1, -1]
            self.__read_arm = [-1, -1, -1, -1, -1, -1]
            self.__read_arm_ok = 0
            self.__request_data(self.FUNC_ARM_CTRL, 1)
            timeout = 30
            while timeout > 0:
                if self.__read_arm_ok == 1:
                    for i in range(6):
                        if self.__read_arm[i] > 0:
                            angle[i] = self.__arm_convert_angle(i+1, self.__read_arm[i])
                    if self.__debug:
                        print("angle_array:", 30-timeout, angle)
                    break
                timeout = timeout - 1
                time.sleep(.001)
            return angle
        except:
            print('---get_uart_servo_angle_array error!---')
            return [-2, -2, -2, -2, -2, -2]

    # 获取加速度计三轴数据，返回a_x, a_y, a_z
    # Get accelerometer triaxial data, return a_x, a_y, a_z
    def get_accelerometer_data(self):
        s = self.__imu
        return s.ax, s.ay, s.az

    # 获取陀螺仪三轴数据，返回g_x, g_y, g_z
    # Get the gyro triaxial data, return g_x, g_y, g_z
    def get_gyroscope_data(self):
        s = self.__imu
        return s.gx, s.gy, s.gz

    # 获取磁力计三轴数据，返回m_x, m_y, m_z (tesla)
    def get_magnetometer_data(self):
        s = self.__imu
        return s.mx, s.my, s.mz

    # 获取板子姿态角，返回yaw, roll, pitch
    # ToAngle=True返回角度，ToAngle=False返回弧度。
    # NOTE: firmware V2.4 never emits FUNC_REPORT_IMU_ATT (0x0C), so on this
    # board this getter returns zeros forever. Check get_attitude_sample().seq
    # before trusting it.
    def get_imu_attitude_data(self, ToAngle=True):
        s = self.__att
        if ToAngle:
            RtA = 57.2957795
            return s.roll * RtA, s.pitch * RtA, s.yaw * RtA
        return s.roll, s.pitch, s.yaw

    # 获取小车速度，val_vx, val_vy, val_vz
    # Get the car speed, val_vx, val_vy, val_vz
    def get_motion_data(self):
        s = self.__motion
        return s.vx, s.vy, s.vz

    # 获取电池电压值
    # Get the battery voltage
    def get_battery_voltage(self):
        return self.__motion.battery / 10.0

    # 获取四路电机编码器数据
    # Obtain data of four-channel motor encoder
    def get_motor_encoder(self):
        s = self.__enc
        return s.m1, s.m2, s.m3, s.m4

    # --- consistent, timestamped snapshots -------------------------------
    # Prefer these over the individual getters above: each returns one packet's
    # worth of fields captured at a single instant, plus a monotonic stamp `t`
    # and a `seq` counter. Calling get_accelerometer_data() and then
    # get_gyroscope_data() can straddle two packets; this cannot.
    def get_imu_sample(self):
        """Latest ImuSample(gx gy gz ax ay az mx my mz t seq)."""
        return self.__imu

    def get_attitude_sample(self):
        """Latest AttSample(roll pitch yaw t seq), radians."""
        return self.__att

    def get_motion_sample(self):
        """Latest MotionSample(vx vy vz battery t seq); battery is 0.1 V units."""
        return self.__motion

    def get_encoder_sample(self):
        """Latest EncSample(m1 m2 m3 m4 t seq)."""
        return self.__enc

    def get_imu_kind(self):
        """'mpu9250', 'icm20948', or None if no raw IMU packet has arrived."""
        return self.__imu_kind

    # 获取小车的运动PID参数, 返回[kp, ki, kd]
    # Get the motion PID parameters of the dolly and return [kp, ki, kd]
    def get_motion_pid(self):
        self.__kp1 = 0
        self.__ki1 = 0
        self.__kd1 = 0
        self.__pid_index = 0
        self.__request_data(self.FUNC_SET_MOTOR_PID, int(1))
        for i in range(20):
            if self.__pid_index > 0:
                kp = float(self.__kp1 / 1000.0)
                ki = float(self.__ki1 / 1000.0)
                kd = float(self.__kd1 / 1000.0)
                if self.__debug:
                    print("get_motion_pid: {0}, {1}, {2}".format(self.__pid_index, [kp, ki, kd], i))
                return [kp, ki, kd]
            time.sleep(.001)
        return [-1, -1, -1]

    # 获取小车偏航角PID参数
    # PID parameters of trolley yaw Angle were obtained
    # def get_yaw_pid(self):
    #     self.__kp1 = 0
    #     self.__ki1 = 0
    #     self.__kd1 = 0
    #     self.__pid_index = 0
    #     self.__request_data(self.FUNC_SET_YAW_PID, int(5))
    #     for i in range(20):
    #         if self.__pid_index > 0:
    #             kp = float(self.__kp1 / 1000.0)
    #             ki = float(self.__ki1 / 1000.0)
    #             kd = float(self.__kd1 / 1000.0)
    #             if self.__debug:
    #                 print("get_yaw_pid: {0}, {1}, {2}".format(self.__pid_index, [kp, ki, kd], i))
    #             return [kp, ki, kd]
    #         time.sleep(.001)
    #     return [-1, -1, -1]

    # 获取当前底层小车类型。
    # Gets the current car type from machine
    def get_car_type_from_machine(self):
        self.__request_data(self.FUNC_SET_CAR_TYPE)
        for i in range(0, 20):
            if self.__read_car_type != 0:
                car_type = self.__read_car_type
                self.__read_car_type = 0
                return car_type
            time.sleep(.001)
        return -1


    # 获取底层单片机版本号，如1.1
    # Get the underlying microcontroller version number, such as 1.1
    def get_version(self):
        # Mcnamu_driver_X3.pub_data calls this every publish cycle (20 Hz). The
        # stock version re-requested and then busy-polled for 20 ms on EVERY
        # call whenever the version had not arrived yet - 20 spurious request
        # frames/s onto the shared UART plus ~44% of the driver's wall clock
        # burnt in a sleep loop, which is precisely the state you are in after
        # the receive thread has died. Rate-limit the retry instead.
        if self.__version_H != 0:
            return self.__version
        now = time.monotonic()
        if now - self.__version_last_req < 5.0:
            return -1
        self.__version_last_req = now
        self.__request_data(self.FUNC_VERSION)
        for i in range(0, 20):
            if self.__version_H != 0:
                self.__version = self.__version_H * 1.0 + self.__version_L / 10.0
                if self.__debug:
                    print("get_version:V{0}, i:{1}".format(self.__version, i))
                return self.__version
            time.sleep(.001)
        return -1


if __name__ == '__main__':
    # 小车底层处理库
    # com_index = 1
    # while True:
    #     com_index = com_index + 1
    #     try:
    #         print("try COM%d" % com_index)
    #         com = 'COM%d' % com_index
    #         bot = Rosmaster(1, com, debug=True)
    #         break
    #     except:
    #         if com_index > 256:
    #             print("-----------------------No COM Open--------------------------")
    #             exit(0)
    #         continue
    # print("--------------------Open %s---------------------" % com)

    bot = Rosmaster(car_type=5, debug=True)
    bot.create_receive_threading()
    time.sleep(.1)
    bot.set_beep(50)
    time.sleep(.1)

    version = bot.get_version()
    print("version=", version)

    bot.set_car_type(5)
    time.sleep(.1)
    car_type = bot.get_car_type_from_machine()
    print("car_type:", car_type)
    # bot.set_uart_servo_angle(1, 100)

    # s_id, value = bot.get_uart_servo_value(1)
    # print("value:", s_id, value)
    
    # bot.set_uart_servo_torque(1)
    # time.sleep(.1)
    # state = bot.set_uart_servo_offset(6)
    # print("state=", state)

    # bot.set_pid_param(0.5, 0.1, 0.3, 8, True)
    # bot.set_yaw_pid_param(0.4, 2, 0.2, False)
    # bot.reset_flash_value()

    # pid = bot.get_motion_pid(5)
    # pid = bot.get_yaw_pid()
    # print("pid:", pid)

    # angle= bot.get_uart_servo_angle(1)
    # print("angle:", angle)

    # angle_array = bot.get_uart_servo_angle_array()
    # print("angle_array:", angle_array)

    bot.set_car_motion(0.5, 0, 0)

    # bot.send_ip_addr("192.168.1.2")

    # bot.set_uart_servo_angle(6, 150)

    # bot.set_auto_report_state(0, False)

    # bot.set_pwm_servo_all(50, 50, 50, 50)
    # time.sleep(1)
    # bot.set_car_motion(0, 0, -3.5)
    # bot.set_car_run(6, 50)
    try:
        while True:
            ax, ay, az = bot.get_accelerometer_data()
            gx, gy, gz = bot.get_gyroscope_data()
            mx, my, mz = bot.get_magnetometer_data()
            # print(ax, ay, az)
            # print(ax, ay, az, gx, gy, gz, mx, my, mz)
            # print("%3.3f, %3.3f, %3.3f,      %3.3f, %3.3f, %3.3f" % 
            # (ax, ay, az, gx, gy, gz))
            print("%3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f, %3.3f" % 
            (ax, ay, az, gx, gy, gz, mx, my, mz))
            # roll, pitch, yaw = bot.get_imu_attitude_data()
            # print("roll:%f, pitch:%f, yaw:%f" % (roll, pitch, yaw))
            # m1, m2, m3, m4 = bot.get_motor_encoder()
            # print("encoder:", m1, m2, m3, m4)

            v = bot.get_motion_data()
            print("v:", v)

            # pid = bot.get_motion_pid()
            # print(pid)
            # version = bot.get_version()
            # print("version=", version)
            # vx, vy, vz = bot.get_motion_data()
            # print("V:", vx, vy, vz)
            time.sleep(.1)
    except KeyboardInterrupt:
        bot.set_car_motion(0, 0, 0)
        pass
    exit(0)
