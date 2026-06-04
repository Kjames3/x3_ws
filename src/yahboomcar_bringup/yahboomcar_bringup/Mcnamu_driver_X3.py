#!/usr/bin/env python
# encoding: utf-8

#public lib
import sys
import math
import random
import threading
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
		self.car.create_receive_threading()
		self.last_cmd_time = self.get_clock().now()
		self.watchdog_timeout = 0.5
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

	#pub data
	def pub_data(self):
		# Safety watchdog: if no command received for > 0.5s, stop motors
		dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
		if dt > self.watchdog_timeout:
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
		imu.angular_velocity.z = gz*1.0

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
		twist.angular.z = gz   # Use IMU gyro instead of firmware's wrong vz
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

		
		
