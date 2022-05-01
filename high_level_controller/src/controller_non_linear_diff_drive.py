#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt, sin, cos, atan, copysign, pi
import time
import tf
import numpy as np
import NonLinearDifferentialController


class Rover:

	def __init__(self):
		# Creates a unique node
		rospy.init_node('controller_non_linear_diff_drive', anonymous=True)

		# Publisher which will publish to the command topic
		self.velocity_publisher = rospy.Publisher('/cmd_vel',Twist, queue_size=10)

		# A subscriber to Odometry which is called when a message is received
		self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.update_odom)

		# A subscriber to PoseStamped which is called when a message is received
		self.goal_subscriber = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.callback_pos)

		self.odom = Odometry()
		self.goal_pose = PoseStamped()

		self.dt = 0.01
		self.initialized_controller = False

		self.L = 0.287
		self.r = 0.033
		self.kp_pos = 20
		self.kp_theta = 50
		self.distance_tolerance = 0.01
		self.yaw_tolerance = 0.02
		self.turning_speed = 50
		self.position_reached_print = False

	def update_odom(self, data):
		"""Callback function which is called when a new message of type Odometry
		is received by the subscriber."""
		dt = (data.header.stamp - self.odom.header.stamp).to_sec() # first will be a big number
		if (dt <= 0.1 and not self.initialized_controller):
			self.dt = dt
			self.controller = NonLinearDifferentialController.NonLinearDifferentialController(
			                  self.dt
							, self.L
							, self.r
							, self.kp_pos
							, self.kp_theta
							, self.distance_tolerance
							, self.yaw_tolerance
							, self.turning_speed)
			self.initialized_controller = True

		self.odom = data
		if self.initialized_controller:
			self.positionController()

	def callback_pos(self, data):
		"""Callback function which is called when a new message of type PoseStamped
		is received by the subscriber."""
		self.position_reached_print = True
		self.goal_pose = data
		rospy.loginfo("Position command received x: %0.2f y: %0.2f", self.goal_pose.pose.position.x,
															self.goal_pose.pose.position.y)

	def euclidean_distance(self, goal_pose):
		"""Euclidean distance between current pose and the goal."""
		return sqrt(pow((goal_pose.pose.position.x - self.odom.pose.pose.position.x), 2) +
				pow((goal_pose.pose.position.y - self.odom.pose.pose.position.y), 2))

	def positionController(self):
		"""Moves the rover to the goal."""

		vel_msg = Twist()

		if self.euclidean_distance(self.goal_pose) >= self.distance_tolerance:

			qx = self.odom.pose.pose.orientation.x
			qy = self.odom.pose.pose.orientation.y
			qz = self.odom.pose.pose.orientation.z
			qw = self.odom.pose.pose.orientation.w

			roll, pitch, yaw = tf.transformations.euler_from_quaternion((qx, qy, qz, qw))

			x = self.odom.pose.pose.position.x
			y = self.odom.pose.pose.position.y

			state = [x , y, yaw]

			x_t = self.goal_pose.pose.position.x
			y_t = self.goal_pose.pose.position.y

			yaw_t = atan2((y_t-y),(x_t-x+1e-9))

			state_t = [x_t , y_t, yaw_t]
			action = self.controller.get_action(state, state_t) # give us in rad/s to convert to PWM, 0 is left, 1 is right
			ul = -action[0]
			ur = -action[1]
			#print("x: ",x," y: ",y, " x_t: ",x_t," y_t: ",y_t)
			#print("ul: ",ul," ur: ",ur)
			# Linear velocity in the x-axis.
			vel_msg.linear.x = self.r*(ul + ur)/2 #from rad/s into m/s

			# Angular velocity in the z-axis.
			vel_msg.angular.z = self.r*(ur - ul)/self.L #rad/s

		else:
			# Stopping robot
			vel_msg.linear.x = 0
			vel_msg.angular.z = 0
			if self.position_reached_print:
				rospy.loginfo("Position reached!")
				self.position_reached_print = False
		# Publishing our vel_msg
		self.velocity_publisher.publish(vel_msg)


if __name__ == '__main__':
	try:
		roverObject = Rover()
		# If we press control + C, the node will stop.
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
