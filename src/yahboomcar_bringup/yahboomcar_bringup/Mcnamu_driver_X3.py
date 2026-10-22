#!/usr/bin/env python
# encoding: utf-8

#public lib
import os
import sys
import json
import math
import time as _time
import random
import threading
from math import pi
from time import sleep
from .Rosmaster_Lib import Rosmaster
from .wheel_speed_control import (WheelSpeedController, mecanum_targets,
	limit_targets)

#ros lib
import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float32,Int32,Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu,MagneticField, JointState
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

#from dynamic_reconfigure.server import Server
car_type_dic={
    'R2':5,
    'X3':1,
    'NONE':-1
}
class yahboomcar_driver(Node):
	def __init__(self, name):
		super().__init__(name)
		global car_type_dic
		self.RA2DE = 180 / pi
		#get parameter
		self.declare_parameter('serial_port', '/dev/ttyCH341USB0')
		serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
		self.car = Rosmaster(com=serial_port)
		self.car.set_car_type(1)
		self.declare_parameter('car_type', 'X3')
		self.car_type = self.get_parameter('car_type').get_parameter_value().string_value
		print (self.car_type)
		self.declare_parameter('imu_link', 'imu_link')
		self.imu_link = self.get_parameter('imu_link').get_parameter_value().string_value
		print (self.imu_link)
		self.declare_parameter('Prefix', "")
		self.Prefix = self.get_parameter('Prefix').get_parameter_value().string_value
		print (self.Prefix)
		self.declare_parameter('xlinear_limit', 1.0)
		self.xlinear_limit = self.get_parameter('xlinear_limit').get_parameter_value().double_value
		print (self.xlinear_limit)
		self.declare_parameter('ylinear_limit', 1.0)
		self.ylinear_limit = self.get_parameter('ylinear_limit').get_parameter_value().double_value
		print (self.ylinear_limit)
		self.declare_parameter('angular_limit', 5.0)
		self.angular_limit = self.get_parameter('angular_limit').get_parameter_value().double_value
		print (self.angular_limit)

		# Kinematic geometry: (half_wheelbase + half_track_width) from URDF.
		# URDF wheel positions: x=±0.08 m, y=±0.0845 m → factor = 0.08+0.0845 = 0.1645
		self.declare_parameter('wheel_separation_factor', 0.165)
		# Per-wheel gain corrections for motor asymmetry.
		# Robot drifts right → right motors are stronger → lower gain_fr / gain_rr.
		# Tune in ~0.02 steps: drive straight, reduce the fast side until drift stops.
		# Live-tune without restart: ros2 param set /driver_node gain_fr 0.93
		self.declare_parameter('gain_fl', 1.00)
		self.declare_parameter('gain_fr', 0.95)
		self.declare_parameter('gain_rl', 1.00)
		self.declare_parameter('gain_rr', 0.95)

		# Publish the Rosmaster's on-board MPU9250 to imu/data_raw + imu/mag?
		# Set False when the ICM-42688-P node (icm42688_node) owns imu/data_raw,
		# so the two IMUs do not both publish to the same topic. Leaving this
		# True and running both is the A/B configuration.
		# NOTE: this gates ONLY the imu/mag topics. The MPU9250 gyro is also the
		# angular-velocity source for vel_raw below (the firmware's own value is
		# wrong -- M2/M3 encoder cables are swapped), and that path is untouched.
		self.declare_parameter('publish_imu', True)

		# Source for vel_raw's angular.z, which base_node_X3 integrates into
		# /odom_raw and the EKF then takes as its ABSOLUTE yaw (odom0 supplies
		# both yaw and vyaw; imu0 supplies vyaw only). Left False the MPU9250's
		# -6.84 deg/min bias anchors heading no matter how good the new IMU is.
		# The ICM-42688-P was verified against the MPU9250 to 0.2% over a full
		# turn and against a physical protractor to <=0.35%, so this is a pure
		# bias improvement with no sign or scale risk.
		self.declare_parameter('use_external_imu_yaw', False)
		self._ext_wz = None
		self._ext_t = 0.0
		self._ext_warned = False
		self.sub_ext_imu = self.create_subscription(
			Imu, 'imu/data_raw', self.ext_imu_callback, 20)

		#create subcriber
		# BEST_EFFORT, not the default RELIABLE: with Nav2 driving, a reliable reader
		# stalled for 3-4 s (the writer's heartbeat period) after any dropped datagram,
		# holding back every command behind it.  Measured 2026-10-22 with identical probe
		# subscribers: depth-1 reliable lost ~17% of /cmd_vel and the driver itself ~50%,
		# depth-1 best-effort lost ~0%.  A 20 Hz command stream does not need
		# retransmission; a lost stop is covered by the 0.5 s watchdog.
		cmd_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
			history=HistoryPolicy.KEEP_LAST)
		self.sub_cmd_vel = self.create_subscription(Twist,"cmd_vel",self.cmd_vel_callback,cmd_qos)
		self.sub_RGBLight = self.create_subscription(Int32,"RGBLight",self.RGBLightcallback,100)
		self.sub_BUzzer = self.create_subscription(Bool,"Buzzer",self.Buzzercallback,100)

		#create publisher
		self.EdiPublisher = self.create_publisher(Float32,"edition",100)
		self.volPublisher = self.create_publisher(Float32,"voltage",100)
		self.staPublisher = self.create_publisher(JointState,"joint_states",100)
		self.velPublisher = self.create_publisher(Twist,"vel_raw",50)
		self.imuPublisher = self.create_publisher(Imu,"imu/data_raw",100)
		self.magPublisher = self.create_publisher(MagneticField,"imu/mag",100)

		#create timer
		self.timer = self.create_timer(0.1, self.pub_data)

		# Closed-loop wheel speed (wheel_speed_control.py). False keeps the
		# original open-loop PWM mixing in cmd_vel_callback. Switchable live:
		#   ros2 param set /driver_node speed_control true
		self.declare_parameter('speed_control', False)
		# Rim speed per rad/s of yaw, for the closed loop. The geometric
		# 0.1645 (wheel_separation_factor) turned at 0.77 of the command with
		# every wheel on its target (gyro, 2026-10-11: 0.467 for 0.6 rad/s,
		# 0.92 for 1.2), so the effective value is 0.1645 / 0.766.
		self.declare_parameter('yaw_wheel_separation', 0.215)
		# Fastest rim speed asked of any wheel; PWM 100 gives about 1.0 m/s.
		self.declare_parameter('max_wheel_speed', 0.8)
		self.speed_ctrl = WheelSpeedController()
		self._speed_control_on = False
		self._enc_seq = None
		self._enc_time = self.get_clock().now()

		# Wheel encoders, once per board report (every 40 ms), in the board's
		# motor order M1..M4 = FL, RL, FR, RR. position = raw counts (they
		# DEcrease driving forward), velocity = rim speed m/s (forward +),
		# effort = [PWM sent M1..M4, then the ramped speed targets M1..M4]
		# while speed_control is on.
		self.encPublisher = self.create_publisher(JointState, "wheel_encoders", 50)
		self.enc_timer = self.create_timer(0.01, self.pub_encoders)

		# Delivery diagnostics, 1 Hz on /driver_diag and (when something is
		# happening) a line in ~/driver_diag.log.  Added 2026-10-22 for the
		# stalls where Nav2's /cmd_vel reached a recorder but the wheels got
		# PWM 0: cmd_cb counts the commands THIS node received, wd_trips the
		# times its 0.5 s watchdog zeroed a live target, max_tick_gap_ms how
		# long its executor went without running the 100 Hz encoder timer.
		self.diagPublisher = self.create_publisher(String, "driver_diag", 10)
		self._diag_path = os.path.expanduser('~/driver_diag.log')
		self._diag_reset()
		self.diag_timer = self.create_timer(1.0, self.pub_diag)

		#create and init variable
		self.edition = Float32()
		self.edition.data = 1.0
		self.car.create_receive_threading()
		self.last_cmd_time = self.get_clock().now()
		self.watchdog_timeout = 0.5
	def ext_imu_callback(self, msg):
		# Only meaningful when another node owns imu/data_raw. If publish_imu is
		# still True this is our OWN message coming back, which is harmless but
		# pointless -- the launch sets publish_imu False alongside this.
		self._ext_wz = msg.angular_velocity.z
		self._ext_t = self.get_clock().now().nanoseconds / 1e9

	#callback function
	def cmd_vel_callback(self, msg):
		# Compute mecanum kinematics here and send per-wheel PWM via set_motor().
		# set_car_motion() has a firmware limitation where only the first non-zero
		# axis is applied (priority: vx > vy > omega), which breaks combined inputs
		# such as driving forward while rotating or strafing while rotating.
		if not isinstance(msg, Twist): return
		self.last_cmd_time = self.get_clock().now()
		_mono = _time.monotonic()
		if self._diag_last_cb is not None:
			self._diag_cb_gap = max(self._diag_cb_gap, _mono - self._diag_last_cb)
		self._diag_last_cb = _mono
		self._diag_cb += 1
		if msg.linear.x or msg.linear.y or msg.angular.z:
			self._diag_cb_nz += 1
			self._diag_last_cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
		vx    = msg.linear.x
		vy    = msg.linear.y
		omega = msg.angular.z

		# Mecanum wheel mixing: M1=FL, M2=FR, M3=RL, M4=RR
		L     = self.get_parameter('wheel_separation_factor').value
		if self.get_parameter('speed_control').value:
			# Closed loop: only set the wheel speed targets here; pub_encoders
			# sends the PWM, once per encoder report.
			self.speed_ctrl.set_targets(limit_targets(
				mecanum_targets(vx, vy, omega,
					self.get_parameter('yaw_wheel_separation').value),
				self.get_parameter('max_wheel_speed').value))
			return
		# SCALE: maps m/s → PWM units [-100, 100].  At SCALE=200, 0.5 m/s → 100 PWM.
		SCALE = 200.0
		GAIN_FL = self.get_parameter('gain_fl').value
		GAIN_FR = self.get_parameter('gain_fr').value
		GAIN_RL = self.get_parameter('gain_rl').value
		GAIN_RR = self.get_parameter('gain_rr').value

		# Correct Mecanum equations for the X3 chassis layout:
		# vy > 0 (left strafe): FL -, RL +, FR +, RR -
		# omega > 0 (CCW left turn): FL -, RL -, FR +, RR +
		fl = (vx - vy - omega * L) * SCALE * GAIN_FL
		fr = (vx + vy + omega * L) * SCALE * GAIN_FR
		rl = (vx + vy - omega * L) * SCALE * GAIN_RL
		rr = (vx - vy + omega * L) * SCALE * GAIN_RR

		# Apply deadband compensation to overcome static friction on floor.
		# When commanded velocities are non-zero, active motors must receive at least
		# min_pwm to physically overcome friction and start rotating smoothly.
		min_pwm = 28.0
		if abs(vx) > 0.001 or abs(vy) > 0.001 or abs(omega) > 0.001:
			fl = fl + min_pwm if fl > 0.1 else (fl - min_pwm if fl < -0.1 else 0.0)
			fr = fr + min_pwm if fr > 0.1 else (fr - min_pwm if fr < -0.1 else 0.0)
			rl = rl + min_pwm if rl > 0.1 else (rl - min_pwm if rl < -0.1 else 0.0)
			rr = rr + min_pwm if rr > 0.1 else (rr - min_pwm if rr < -0.1 else 0.0)

		# Proportional normalisation: if any wheel exceeds ±100, scale all down
		# together so the commanded ratio between wheels is preserved.
		max_val = max(abs(fl), abs(fr), abs(rl), abs(rr))
		if max_val > 100.0:
			factor = 100.0 / max_val
			fl *= factor; fr *= factor; rl *= factor; rr *= factor

		# Hardware wiring: M1=FL, M2=RL, M3=FR, M4=RR
		# (M2 and M3 cables are swapped on the Rosmaster board)
		self.car.set_motor(int(fl), int(rl), int(fr), int(rr))
	def RGBLightcallback(self,msg):
        # 流水灯控制，服务端回调函数 RGBLight control
		if not isinstance(msg, Int32): return
		# print ("RGBLight: ", msg.data)
		for i in range(3): self.car.set_colorful_effect(msg.data, 6, parm=1)
	def Buzzercallback(self,msg):
		if not isinstance(msg, Bool): return
		if msg.data:
			for i in range(3): self.car.set_beep(1)
		else:
			for i in range(3): self.car.set_beep(0)

	def pub_encoders(self):
		"""100 Hz poll; acts once per new encoder report from the board."""
		_mono = _time.monotonic()
		if self._diag_last_tick is not None:
			self._diag_tick_gap = max(self._diag_tick_gap, _mono - self._diag_last_tick)
		self._diag_last_tick = _mono
		self._diag_ticks += 1
		now = self.get_clock().now()
		counts, seq = self.car.get_motor_encoder_report()
		on = bool(self.get_parameter('speed_control').value)
		if on != self._speed_control_on:
			# Either way round, start from rest: stop the motors and forget
			# what the controller had measured.
			self._speed_control_on = on
			self.speed_ctrl.reset()
			self.speed_ctrl.set_targets([0.0] * 4)
			self.car.set_motor(0, 0, 0, 0)
			self.get_logger().info('wheel speed control %s' % ('ON' if on else 'OFF (open-loop PWM)'))
		if seq == self._enc_seq:
			# No report for a while with the loop closed: the speeds are
			# unknown, so do not keep driving on the last PWM.
			if on and (now - self._enc_time).nanoseconds > 0.3e9 and any(self.speed_ctrl.pwm):
				self.speed_ctrl.reset()
				self.car.set_motor(0, 0, 0, 0)
				self.get_logger().warn('no encoder report for 0.3 s; motors stopped')
			return
		self._enc_seq = seq
		self._enc_time = now
		self._diag_reports += 1

		msg = JointState()
		msg.header.stamp = now.to_msg()
		msg.name = ["m1_fl", "m2_rl", "m3_fr", "m4_rr"]
		msg.position = [float(c) for c in counts]
		if on:
			# Same 0.5 s watchdog as the open-loop path: no command, no motion.
			if (now - self.last_cmd_time).nanoseconds / 1e9 > self.watchdog_timeout:
				if any(self.speed_ctrl.targets):
					self._diag_wd_trips += 1
				self.speed_ctrl.set_targets([0.0] * 4)
			pwm = self.speed_ctrl.update(counts, seq)
			self.car.set_motor(*pwm)
			msg.velocity = [float(v) for v in self.speed_ctrl.speeds]
			msg.effort = [float(p) for p in pwm] + [float(r) for r in self.speed_ctrl.ramped]
		self.encPublisher.publish(msg)

	def _diag_reset(self):
		self._diag_cb = self._diag_cb_nz = 0
		self._diag_cb_gap = 0.0
		self._diag_ticks = self._diag_reports = self._diag_wd_trips = 0
		self._diag_tick_gap = 0.0
		if not hasattr(self, '_diag_last_cb'):
			self._diag_last_cb = self._diag_last_tick = None
			self._diag_last_cmd = (0.0, 0.0, 0.0)

	def pub_diag(self):
		age_ms = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e6
		ctrl = self.speed_ctrl
		rec = {
			"t": round(_time.time(), 2),
			"cmd_cb": self._diag_cb, "cmd_cb_nz": self._diag_cb_nz,
			"cmd_gap_max_ms": round(self._diag_cb_gap * 1e3),
			"cmd_age_ms": round(age_ms), "last_cmd": [round(v, 3) for v in self._diag_last_cmd],
			"pubs": self.count_publishers('cmd_vel'),
			"enc_ticks": self._diag_ticks, "enc_reports": self._diag_reports,
			"tick_gap_max_ms": round(self._diag_tick_gap * 1e3),
			"wd_trips": self._diag_wd_trips,
			"tgt_max": round(max(abs(t) for t in ctrl.targets), 3),
			"ramped_max": round(max(abs(r) for r in ctrl.ramped), 3),
			"pwm_max": max(abs(p) for p in ctrl.pwm),
		}
		msg = String()
		msg.data = json.dumps(rec)
		self.diagPublisher.publish(msg)
		if self._diag_cb or self._diag_wd_trips or rec["tgt_max"] or rec["pwm_max"]:
			try:
				if os.path.exists(self._diag_path) and os.path.getsize(self._diag_path) > 5_000_000:
					os.replace(self._diag_path, self._diag_path + '.1')
				with open(self._diag_path, 'a') as f:
					f.write(msg.data + '\n')
			except OSError:
				pass
		self._diag_reset()

	#pub data
	def pub_data(self):
		# Safety watchdog: if no command received for > 0.5s, stop motors
		dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
		if dt > self.watchdog_timeout and not self.get_parameter('speed_control').value:
			# (closed loop: pub_encoders zeroes the targets, which brakes the
			# wheels and then releases them)
			self.car.set_motor(0, 0, 0, 0)

		time_stamp = Clock().now()
		imu = Imu()
		twist = Twist()
		battery = Float32()
		edition = Float32()
		mag = MagneticField()
		state = JointState()
		state.header.stamp = time_stamp.to_msg()
		state.header.frame_id = "joint_states"
		if len(self.Prefix)==0:
			state.name = ["back_right_joint", "back_left_joint","front_left_steer_joint","front_left_wheel_joint",
							"front_right_steer_joint", "front_right_wheel_joint"]
		else:
			state.name = [self.Prefix+"back_right_joint",self.Prefix+ "back_left_joint",self.Prefix+"front_left_steer_joint",self.Prefix+"front_left_wheel_joint",
							self.Prefix+"front_right_steer_joint", self.Prefix+"front_right_wheel_joint"]
		
		#print ("mag: ",self.car.get_magnetometer_data())		
		edition.data = self.car.get_version()*1.0
		battery.data = self.car.get_battery_voltage()*1.0
		ax, ay, az = self.car.get_accelerometer_data()
		gx, gy, gz = self.car.get_gyroscope_data()
		mx, my, mz = self.car.get_magnetometer_data()
		mx = mx * 1.0
		my = my * 1.0
		mz = mz * 1.0
		vx, vy, angular = self.car.get_motion_data()
		'''print("vx: ",vx)
		print("vy: ",vy)
		print("angular: ",angular)'''
		# 发布陀螺仪的数据
		# Publish gyroscope data
		imu.header.stamp = time_stamp.to_msg()
		imu.header.frame_id = self.imu_link
		imu.linear_acceleration.x = ax*1.0
		imu.linear_acceleration.y = ay*1.0
		imu.linear_acceleration.z = az*1.0
		imu.angular_velocity.x = gx*1.0
		imu.angular_velocity.y = gy*1.0
		# We negate gz to align with the ROS CCW positive convention because the physical IMU sensor is inverted.
		imu.angular_velocity.z = -gz*1.0

		mag.header.stamp = time_stamp.to_msg()
		mag.header.frame_id = self.imu_link
		mag.magnetic_field.x = mx*1.0
		mag.magnetic_field.y = my*1.0
		mag.magnetic_field.z = mz*1.0
		
		# Publish the current linear vel and angular vel of the car
		# NOTE: The firmware's angular velocity (from get_motion_data) is incorrect
		# because M2/M3 encoder cables are swapped on the board — the firmware
		# computes ~0 angular velocity during pure rotation.  Use the IMU gyro gz
		# (which reads the physical sensor directly) as the angular velocity source.
		twist.linear.x = vx *1.0
		twist.linear.y = vy *1.0
		# Angular velocity source. The firmware's own value (get_motion_data) is
		# wrong because the M2/M3 encoder cables are swapped, so it must come
		# from a gyro either way.
		#   external : ICM-42688-P via imu/data_raw, already in the ROS frame,
		#              bias-corrected, and NOT negated (it is mounted upright;
		#              only the MPU9250 needs the flip).
		#   fallback : the Rosmaster's MPU9250, negated because it is inverted.
		# Falls back on staleness so a dead IMU node degrades to the old
		# behaviour instead of silently publishing zero rotation into odom.
		use_ext = self.get_parameter('use_external_imu_yaw').value
		age = self.get_clock().now().nanoseconds / 1e9 - self._ext_t
		if use_ext and self._ext_wz is not None and age < 0.5:
			twist.angular.z = self._ext_wz
			self._ext_warned = False
		else:
			if use_ext and not self._ext_warned:
				self.get_logger().warn(
					'use_external_imu_yaw is set but imu/data_raw is %s; '
					'falling back to the MPU9250 gyro'
					% ('stale by %.1fs' % age if self._ext_wz is not None else 'silent'))
				self._ext_warned = True
			twist.angular.z = -gz
		self.velPublisher.publish(twist)
		# print("ax: %.5f, ay: %.5f, az: %.5f" % (ax, ay, az))
		# print("gx: %.5f, gy: %.5f, gz: %.5f" % (gx, gy, gz))
		# print("mx: %.5f, my: %.5f, mz: %.5f" % (mx, my, mz))
		# rospy.loginfo("battery: {}".format(battery))
		# rospy.loginfo("vx: {}, vy: {}, angular: {}".format(twist.linear.x, twist.linear.y, twist.angular.z))
		if self.get_parameter('publish_imu').value:
			self.imuPublisher.publish(imu)
			self.magPublisher.publish(mag)
		self.volPublisher.publish(battery)
		self.EdiPublisher.publish(edition)
		
		
			
def main():
	rclpy.init() 
	driver = yahboomcar_driver('driver_node')
	try:
		rclpy.spin(driver)
	except BaseException:
		pass
	finally:
		driver.car.set_motor(0, 0, 0, 0)
		driver.destroy_node()
		rclpy.shutdown()

'''if __name__ == '__main__':
	main()'''

		
		
