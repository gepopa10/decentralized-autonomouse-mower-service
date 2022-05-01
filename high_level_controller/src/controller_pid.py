#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt, sin, cos, atan, copysign, pi
import time
import tf
import numpy as np


class Rover:

	def __init__(self):
		# Creates a unique node
		rospy.init_node('controller_pid', anonymous=True)

		# Publisher which will publish to the command topic
		self.velocity_publisher = rospy.Publisher('/cmd_vel',Twist, queue_size=10)

		# A subscriber to Odometry which is called when a message is received
		self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.update_odom)

		# A subscriber to PoseStamped which is called when a message is received
		self.goal_subscriber = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.callback_pos)

		self.odom = Odometry()
		self.goal_pose = PoseStamped()
		self.yaw = 0
		self.yaw_des = 0
		self.direction = 1

		self.distance_tolerance = 0.01
		self.yaw_tolerance = 0.02


		# Gains
		self.kp_yaw = 3
		self.kd_yaw = 0.002
		self.ki_yaw = 0.001
		self.kp_pos = 0.5
		self.kd_pos = 0.002
		self.ki_pos = 0.001

		# PID errors
		self.sum_error_yaw = 0
		self.previous_error_yaw = 0
		self.sum_error_pos = 0
		self.previous_error_pos = 0
		self.dt = 0

	def update_odom(self, data):
		"""Callback function which is called when a new message of type Odometry
		is received by the subscriber."""
		self.dt = (data.header.stamp - self.odom.header.stamp).to_sec() # first will be a big number
		if self.dt > 100:
			self.dt = 0

		self.odom = data
		self.positionController()

	def callback_pos(self, data):
		"""Callback function which is called when a new message of type PoseStamped
		is received by the subscriber."""

		# self.reset_pid()

		self.goal_pose = data
		rospy.loginfo("Position command received x: %0.2f y: %0.2f", self.goal_pose.pose.position.x,
															self.goal_pose.pose.position.y)

		vector_1 = [cos(self.yaw), sin(self.yaw)]
		vector_2 = [self.goal_pose.pose.position.x - self.odom.pose.pose.position.x, self.goal_pose.pose.position.y - self.odom.pose.pose.position.y]
		unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
		unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
		dot_product = np.dot(unit_vector_1, unit_vector_2)
		angle = np.arccos(dot_product)
		if angle > pi/2:
			self.direction = -1
		else:
			self.direction = 1

		# cmd_steering_angle = atan2(self.goal_pose.pose.position.y - self.odom.pose.pose.position.y, \
		# 		self.goal_pose.pose.position.x - self.odom.pose.pose.position.x)
		# if cmd_steering_angle > pi/2:
		# 	cmd_steering_angle = cmd_steering_angle - pi
		# 	self.direction = -1
		# elif cmd_steering_angle < -pi/2:
		# 	cmd_steering_angle = cmd_steering_angle + pi
		# 	self.direction = -1

	def reset_pid(self):
		self.sum_error_yaw = 0
		self.previous_error_yaw = 0
		self.sum_error_pos = 0
		self.previous_error_pos = 0

	def euclidean_distance(self, goal_pose):
		"""Euclidean distance between current pose and the goal."""
		return sqrt(pow((goal_pose.pose.position.x - self.odom.pose.pose.position.x), 2) +
				pow((goal_pose.pose.position.y - self.odom.pose.pose.position.y), 2))

	def linear_vel(self, goal_pose):

		# sign = copysign(1, self.steering_angle(goal_pose))
		angle, direction = self.steering_angle(goal_pose)
		# -1 for backwards
		# 1 for forward

		error_pos = self.direction*self.euclidean_distance(goal_pose)
		# error_pos = direction*self.euclidean_distance(goal_pose)

		pos_pid = self.kp_pos*error_pos + \
					self.kd_pos*(error_pos-self.previous_error_pos) + \
					self.ki_pos*self.sum_error_pos*self.dt

		self.previous_error_pos = error_pos
		self.sum_error_pos  += error_pos

		return pos_pid

	def steering_angle(self, goal_pose):
		#this gives always something between [-pi,pi] so we cannot go backwards
		cmd_steering_angle = atan2(goal_pose.pose.position.y - self.odom.pose.pose.position.y, \
				goal_pose.pose.position.x - self.odom.pose.pose.position.x)

		# cmd_steering_angle_shorthest = atan((goal_pose.pose.position.y - self.odom.pose.pose.position.y)/ \
		# 		(goal_pose.pose.position.x - self.odom.pose.pose.position.x))
		#
		# vector_1 = [cos(self.yaw), sin(self.yaw)]
		# vector_2 = [goal_pose.pose.position.x - self.odom.pose.pose.position.x, goal_pose.pose.position.y - self.odom.pose.pose.position.y]
		# unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
		# unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
		# dot_product = np.dot(unit_vector_1, unit_vector_2)
		# angle = np.arccos(dot_product)
		# # print(angle)
		# if abs(angle) > pi/2:
		# 	direction = -1
		# else:
		# 	direction = 1


		# if cmd_steering_angle > pi/2:
		# 	cmd_steering_angle = cmd_steering_angle - pi
		# 	direction = -1
		# elif cmd_steering_angle < -pi/2:
		# 	cmd_steering_angle = cmd_steering_angle + pi
		# 	direction = -1

		if self.direction == -1:
			if cmd_steering_angle >= 0:
				cmd_steering_angle = cmd_steering_angle - pi
			elif cmd_steering_angle < 0:
				cmd_steering_angle = cmd_steering_angle + pi

		# if self.direction == 1:
		# 	cmd_steering_angle_shorthest = cmd_steering_angle

		return cmd_steering_angle, self.direction

	def angular_vel(self, goal_pose, control_orientation = False):

		qx = self.odom.pose.pose.orientation.x
		qy = self.odom.pose.pose.orientation.y
		qz = self.odom.pose.pose.orientation.z
		qw = self.odom.pose.pose.orientation.w

		roll, pitch, self.yaw = tf.transformations.euler_from_quaternion((qx, qy, qz, qw))

		if not control_orientation:
			self.yaw_des, direction = self.steering_angle(goal_pose)
		else:
			qx_goal = goal_pose.pose.orientation.x
			qy_goal = goal_pose.pose.orientation.y
			qz_goal = goal_pose.pose.orientation.z
			qw_goal = goal_pose.pose.orientation.w
			roll_des, pitch_des, self.yaw_des = tf.transformations.euler_from_quaternion((qx_goal, qy_goal, qz_goal, qw_goal))

		# print(yaw_des)
		error_yaw = self.yaw_des - self.yaw

		# error_yaw = atan2(sin(error_yaw),cos(error_yaw)) # to keep error between -pi,pi

		yaw_pid = self.kp_yaw*error_yaw + \
					self.kd_yaw*(error_yaw-self.previous_error_yaw) + \
					self.ki_yaw*self.sum_error_yaw*self.dt

		self.previous_error_yaw = error_yaw
		self.sum_error_yaw  += error_yaw

		return yaw_pid

	def positionController(self):
		"""Moves the rover to the goal."""

		vel_msg = Twist()

		if self.euclidean_distance(self.goal_pose) >= self.distance_tolerance:

			# Linear velocity in the x-axis.
			vel_msg.linear.x = self.linear_vel(self.goal_pose)

			# Angular velocity in the z-axis.
			vel_msg.angular.z = self.angular_vel(self.goal_pose)

		else:
			# Stopping our robot after we are distance_tolerance close to the goal
			# vel_msg.linear.x = 0
			# if (self.yaw_des - self.yaw) >= self.yaw_tolerance:
			# 	# We control desired orientation, by default it will be NULL quaternion so yaw_des = 0
			# 	vel_msg.angular.z = self.angular_vel(self.goal_pose, control_orientation = True)
			# else:
			# 	vel_msg.angular.z = 0

			vel_msg.linear.x = 0
			vel_msg.angular.z = self.angular_vel(self.goal_pose, control_orientation = True)

		# Publishing our vel_msg
		self.velocity_publisher.publish(vel_msg)


if __name__ == '__main__':
	try:
		roverObject = Rover()
		# If we press control + C, the node will stop.
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
