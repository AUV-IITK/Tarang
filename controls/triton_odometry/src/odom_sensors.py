#! /usr/bin/env python
"""Node to generate Odometry from DVL sensor"""

import rospy
from nav_msgs.msg import Odometry
from uuv_sensor_ros_plugins_msgs.msg import DVL
from sensor_msgs.msg import Imu
import sys, time
from math import sin, cos, pi
from tf.transformations import euler_from_quaternion, quaternion_from_euler

VERBOSE = True
class odometry_node:
	"""Odometry Node class"""
	# Initialising the variables for positon and velocity
	x = 0.0
	y = 0.0
	z = 0.0
	vel_x = 0.0
	vel_x = 0.0
	vel_x = 0.0
	roll = 0.0
	pitch = 0.0
	yaw = 0.0
	vel_roll = 0.0
	vel_pitch = 0.0
	vel_yaw = 0.0

	# Variable used to calculate the integral
	last_time  = 0.0
	current_time = 0.0

	def __init__(self):
		"""	Initialize the subscriber to subscribe from DVL topic and
			Publisher to publish on the desired topic
		"""
		self.odom_publisher = rospy.Publisher('/triton/odom', Odometry, queue_size=10)
		# Initialise time
		self.last_time = rospy.Time.now()
		self.imu_subscriber = rospy.Subscriber('/triton/imu', Imu, self.callback2, queue_size=10)
		if VERBOSE:
			print("subscribed to /triton/imu")
		self.dvl_subscriber = rospy.Subscriber('/triton/dvl', DVL, self.callback, queue_size=10)
		if VERBOSE:
			print("subscribed to /triton/dvl")
		
	
	def callback2(self, data):
		"""callback function of the subscribed topic publishing IMU Data"""
		orientation_in_quaternion = data.orientation
		quaternion_list = [orientation_in_quaternion.x, orientation_in_quaternion.y, orientation_in_quaternion.z, orientation_in_quaternion.w]
		(self.roll, self.pitch, self.yaw) = euler_from_quaternion(quaternion_list)

		self.vel_roll = data.angular_velocity.x
		self.vel_pitch = data.angular_velocity.y
		self.vel_yaw = data.angular_velocity.z
				

	def callback(self, data):
		"""callback function of the subscribed topic publishing DVL Data"""
		self.current_time = rospy.Time.now()
		if VERBOSE:
			rospy.loginfo(data.velocity)
		vel_x = data.velocity.x
		vel_y = data.velocity.y
		vel_z = data.velocity.z

		# Integrating velocity to find the position
		dt = (self.current_time - self.last_time).to_sec()
		self.x = self.x + ((vel_x*cos(self.yaw)*cos(self.pitch)) - (vel_y*sin(self.yaw)*cos(self.pitch)) + vel_z*sin(self.pitch))*dt
		self.y = self.y + ((vel_x*(cos(self.roll)*sin(self.yaw)+ sin(self.roll)*sin(self.pitch)*cos(self.yaw))) + vel_y*((cos(self.roll)*cos(self.yaw))- sin(self.roll)*sin(self.pitch)*sin(self.yaw)) - vel_z*(sin(self.roll)*cos(self.pitch)))*dt
		self.z = self.z + ((vel_x*(sin(self.roll)*sin(self.yaw)- cos(self.roll)*sin(self.pitch)*cos(self.yaw))) + vel_y*(sin(self.roll)*cos(self.yaw) + cos(self.roll)*sin(self.pitch)*sin(self.yaw)) + vel_z*(cos(self.roll)*cos(self.pitch)))*dt

		# Building the odometry message

		odom = Odometry()
		odom.header = data.header
		odom.child_frame_id = '/triton/base_link'
		odom.pose.pose.position.x = self.x
		odom.pose.pose.position.y = self.y
		odom.pose.pose.position.z = self.z

		q = quaternion_from_euler(self.roll, self.pitch, self.yaw)
		odom.pose.pose.orientation.x = q[0]
		odom.pose.pose.orientation.y = q[1]
		odom.pose.pose.orientation.z = q[2]
		odom.pose.pose.orientation.w = q[3]


		odom.twist.twist.linear.x = vel_x
		odom.twist.twist.linear.y = vel_y
		odom.twist.twist.linear.z = vel_z

		odom.twist.twist.angular.x = self.vel_roll
		odom.twist.twist.angular.y = self.vel_pitch
		odom.twist.twist.angular.z = self.vel_yaw


		if VERBOSE:
			rospy.loginfo(odom)
		self.odom_publisher.publish(odom)



def main(args):
	"""Intialising and Cleaning Up of ROS Nodes"""
	rospy.init_node('odometry_node')
	ic = odometry_node()
	
	try: 
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down the Odometry Node")

if __name__ == '__main__':
	main(sys.argv)

