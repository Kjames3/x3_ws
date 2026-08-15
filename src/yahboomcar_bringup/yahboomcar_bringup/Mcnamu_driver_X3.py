#!/usr/bin/env python
# encoding: utf-8

#public lib
import sys
import math
import random
import threading
import time
from math import pi
from time import sleep
from .Rosmaster_Lib import Rosmaster

#ros lib
import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Float32,Int32,Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu,MagneticField, JointState
from rclpy.clock import Clock

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

		# ── Gyro Z zero-rate bias compensation ──────────────────────────────
		# The MPU9250's gyro has a non-zero output when the chassis is perfectly
		# still. Because twist.angular.z below is taken from the gyro (the
		# firmware's encoder-derived vz is unusable — see pub_data), that bias is
		# integrated into heading by base_node_X3 and becomes unbounded yaw drift:
		# measured at -0.0034 rad/s, which spins /odom a full 360 deg every ~31 min
		# while the robot sits still. AMCL only corrects every update_min_a (0.2 rad
		# ~= once a minute at that rate), so the map-frame pose rotates, the lidar
		# scan sweeps with it, and the global costmap fills with arcs of phantom
		# obstacles that sit outside raytrace_max_range and can never be cleared.
		#
		# Fix: estimate the bias whenever the robot is known to be stationary and
		# subtract it, then hard-gate the residual to exactly zero so a still robot
		# integrates exactly zero heading change.
		#
		# Samples to average for the initial estimate. pub_data runs at 10 Hz, so
		# 50 samples is a 5 s settle. Bias sample std is ~0.00058 rad/s, so the mean
		# of 50 has a standard error near 8e-5 rad/s (~0.3 deg/min residual).
		self.declare_parameter('gyro_bias_calib_samples', 50)
		# Slow EMA that keeps tracking the bias after calibration, since MEMS
		# zero-rate offset moves with temperature over a session. 0.002 is a time
		# constant of ~500 samples (~50 s of stationary time).
		self.declare_parameter('gyro_bias_ema_alpha', 0.002)
		# Reject bias samples above this magnitude: the chassis is genuinely
		# turning (someone is pushing it), so the reading is not bias.
		self.declare_parameter('gyro_bias_max_rate', 0.05)
		# While stationary, treat |rate| below this as sensor noise and publish a
		# hard zero. 0.005 rad/s is 0.29 deg/s — far slower than any real motion,
		# so a hand-turn of the robot still reads through.
		self.declare_parameter('gyro_zero_deadband', 0.005)
		# How long after the last non-zero cmd_vel before the robot counts as
		# stationary, so the chassis has stopped coasting.
		self.declare_parameter('stationary_settle_time', 0.5)
		# Encoder speed below which the wheels count as not turning (m/s).
		self.declare_parameter('encoder_still_threshold', 0.01)
		# Set false to publish the bias-corrected rate without the hard zero gate.
		self.declare_parameter('gyro_zero_gate_enabled', True)

		#create subcriber
		self.sub_cmd_vel = self.create_subscription(Twist,"cmd_vel",self.cmd_vel_callback,1)
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

		#create and init variable
		self.edition = Float32()
		self.edition.data = 1.0
		if not self.car.create_receive_threading():
			self.get_logger().error("Rosmaster receive thread failed to start - no telemetry")
		self.last_cmd_time = self.get_clock().now()
		self.watchdog_timeout = 0.5
		self._rx_stale = False      # edge-trigger for the staleness warning
		self._edition = -1          # cached firmware version
		# Gyro Z bias state. Starts at zero and un-calibrated; the zero gate below
		# suppresses the raw bias during the calibration window, so the first few
		# seconds after boot do not leak drift into /odom.
		self._gyro_bias = 0.0
		self._gyro_bias_sum = 0.0
		self._gyro_bias_count = 0
		self._gyro_bias_ready = False
		# Assume stationary at boot: the robot is at rest until commanded to move.
		self._last_nonzero_cmd_time = self.get_clock().now()
	#callback function
	def cmd_vel_callback(self, msg):
		# Compute mecanum kinematics here and send per-wheel PWM via set_motor().
		# set_car_motion() has a firmware limitation where only the first non-zero
		# axis is applied (priority: vx > vy > omega), which breaks combined inputs
		# such as driving forward while rotating or strafing while rotating.
		if not isinstance(msg, Twist): return
		self.last_cmd_time = self.get_clock().now()
		vx    = msg.linear.x
		vy    = msg.linear.y
		omega = msg.angular.z

		# Track the last command that actually asked for motion. A stream of zero
		# Twists (Nav2 publishes these continuously while idle) must NOT keep
		# resetting the stationary timer, or the bias estimator never runs.
		if abs(vx) > 0.001 or abs(vy) > 0.001 or abs(omega) > 0.001:
			self._last_nonzero_cmd_time = self.last_cmd_time

		# Mecanum wheel mixing: M1=FL, M2=FR, M3=RL, M4=RR
		L     = self.get_parameter('wheel_separation_factor').value
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

	def _is_stationary(self, vx, vy):
		"""True when nothing has commanded motion recently and the wheels agree.

		Both halves matter. The cmd_vel timer alone would be fooled by the robot
		coasting or being pushed; the encoders alone would be fooled by a pure
		in-place rotation, during which vx and vy are both legitimately zero.
		"""
		idle_s = (self.get_clock().now() - self._last_nonzero_cmd_time).nanoseconds / 1e9
		settle = self.get_parameter('stationary_settle_time').value
		still = self.get_parameter('encoder_still_threshold').value
		return idle_s > settle and abs(vx) < still and abs(vy) < still

	def _gyro_z_compensated(self, gz_ros, vx, vy):
		"""Bias-corrected yaw rate, hard-zeroed while the robot is stationary.

		gz_ros is the yaw rate already in the ROS CCW-positive convention.
		"""
		stationary = self._is_stationary(vx, vy)

		# Only learn the bias from samples taken while stationary, and only when
		# the reading is small enough to be bias rather than real rotation.
		if stationary and abs(gz_ros - self._gyro_bias) < self.get_parameter('gyro_bias_max_rate').value:
			if not self._gyro_bias_ready:
				self._gyro_bias_sum += gz_ros
				self._gyro_bias_count += 1
				if self._gyro_bias_count >= self.get_parameter('gyro_bias_calib_samples').value:
					self._gyro_bias = self._gyro_bias_sum / self._gyro_bias_count
					self._gyro_bias_ready = True
					self.get_logger().info(
						"Gyro Z bias calibrated: %.6f rad/s (%.2f deg/min) from %d stationary samples"
						% (self._gyro_bias, math.degrees(self._gyro_bias) * 60.0,
						   self._gyro_bias_count))
			else:
				a = self.get_parameter('gyro_bias_ema_alpha').value
				self._gyro_bias = (1.0 - a) * self._gyro_bias + a * gz_ros

		gz_out = gz_ros - self._gyro_bias

		# Stop-motion gate: a stationary robot must integrate exactly zero heading.
		# Subtracting the bias leaves a small residual that would still accumulate
		# over the minutes the robot spends parked, so clamp it away entirely.
		if (stationary and self.get_parameter('gyro_zero_gate_enabled').value
				and abs(gz_out) < self.get_parameter('gyro_zero_deadband').value):
			gz_out = 0.0
		return gz_out

	#pub data
	def pub_data(self):
		# Safety watchdog: if no command received for > 0.5s, stop motors
		dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
		if dt > self.watchdog_timeout:
			self.car.set_motor(0, 0, 0, 0)

		# Telemetry staleness gate. If the serial receive thread has stopped
		# delivering frames, every getter keeps returning its last value.
		# Republishing that frozen reading with a fresh timestamp is worse than
		# publishing nothing: the Madgwick filter and the robot_localization EKF
		# fuse a constant IMU reading as "the robot is perfectly still", which is
		# exactly the input that makes an EKF confidently wrong about /odom.
		if not self.car.rx_healthy(0.5):
			if not self._rx_stale:
				self._rx_stale = True
				self.get_logger().warn(
					"Rosmaster telemetry stale (%s) - suspending /imu, /vel_raw, /mag"
					% (self.car.rx_stats(),))
			return
		if self._rx_stale:
			self._rx_stale = False
			self.get_logger().info("Rosmaster telemetry recovered")

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
		# Cache the firmware version instead of asking on every 20 Hz cycle.
		if self._edition <= 0:
			self._edition = self.car.get_version()
		edition.data = self._edition*1.0
		# ONE snapshot per packet, not five independent getters. Each getter call
		# is an eval-breaker checkpoint, so the receive thread could land a new
		# packet between them and produce an Imu message whose accelerometer and
		# gyroscope describe different instants (~0.125% of messages at 20 Hz,
		# roughly one bad message every 40 s).
		imu_s = self.car.get_imu_sample()
		mot_s = self.car.get_motion_sample()
		battery.data = mot_s.battery / 10.0
		ax, ay, az = imu_s.ax, imu_s.ay, imu_s.az
		gx, gy, gz = imu_s.gx, imu_s.gy, imu_s.gz
		mx, my, mz = imu_s.mx*1.0, imu_s.my*1.0, imu_s.mz*1.0
		vx, vy, angular = mot_s.vx, mot_s.vy, mot_s.vz
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
		# KNOWN-UNRESOLVED double negation: Rosmaster_Lib already negates gz for
		# the MPU9250 (see _GYRO_SIGNS there), so this second negation cancels it
		# and the published value carries the RAW sensor sign. The two negations
		# are kept as a matched pair because removing either one flips /odom.
		# To settle it: rotate the chassis by hand and check that
		# /imu/data_raw.angular_velocity.z is positive for CCW-from-above
		# (REP-103), then delete exactly one negation.
		#
		# Bias-correct once and reuse for both /imu/data_raw and /vel_raw. The EKF
		# fuses yaw rate from BOTH (imu0_config vyaw and odom0_config vyaw), so
		# correcting only one would leave the other still injecting drift and make
		# the two sources disagree about whether the robot is turning.
		gz_corrected = self._gyro_z_compensated(-gz*1.0, vx, vy)
		imu.angular_velocity.z = gz_corrected

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
		# Same bias-corrected value published on /imu/data_raw above. This is the
		# one that base_node_X3 integrates into heading, so the stop-motion gate
		# here is what actually stops /odom rotating while the robot is parked.
		twist.angular.z = gz_corrected   # Use IMU gyro instead of firmware's wrong vz
		self.velPublisher.publish(twist)
		# print("ax: %.5f, ay: %.5f, az: %.5f" % (ax, ay, az))
		# print("gx: %.5f, gy: %.5f, gz: %.5f" % (gx, gy, gz))
		# print("mx: %.5f, my: %.5f, mz: %.5f" % (mx, my, mz))
		# rospy.loginfo("battery: {}".format(battery))
		# rospy.loginfo("vx: {}, vy: {}, angular: {}".format(twist.linear.x, twist.linear.y, twist.angular.z))
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

		
		
