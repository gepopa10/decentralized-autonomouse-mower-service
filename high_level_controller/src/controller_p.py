#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt, sin, cos
import time
import tf


class Rover:

	def __init__(self):
		# Creates a unique node
		rospy.init_node('controller_p', anonymous=True)

		# Publisher which will publish to the command topic
		self.velocity_publisher = rospy.Publisher('/cmd_vel',Twist, queue_size=10)

		# A subscriber to Odometry which is called when a message is received
		self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.update_odom)

		# A subscriber to PoseStamped which is called when a message is received
		self.goal_subscriber = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.callback_pos)

		self.odom = Odometry()
		self.goal_pose = PoseStamped()

		self.rate = rospy.Rate(30)

		self.distance_tolerance = 0.01

		self.vel_gain = 0.5
		self.yawrate_gain = 1


	def update_odom(self, data):
		"""Callback function which is called when a new message of type Odometry
		is received by the subscriber."""
		self.odom = data

	def callback_pos(self, data):
		"""Callback function which is called when a new message of type PoseStamped
		is received by the subscriber."""
		self.goal_pose = data
		duration = self.move2goal(self.goal_pose, self.distance_tolerance)

	def euclidean_distance(self, goal_pose):
		"""Euclidean distance between current pose and the goal."""
		return sqrt(pow((goal_pose.pose.position.x - self.odom.pose.pose.position.x), 2) +
				pow((goal_pose.pose.position.y - self.odom.pose.pose.position.y), 2))

	def linear_vel(self, goal_pose, constant=1.5):
		"""See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
		return constant * self.euclidean_distance(goal_pose)

	def steering_angle(self, goal_pose):
		"""See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
		return atan2(goal_pose.pose.position.y - self.odom.pose.pose.position.y,
				goal_pose.pose.position.x - self.odom.pose.pose.position.x)

	def angular_vel(self, goal_pose, constant=6):
		"""See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
		qx = self.odom.pose.pose.orientation.x
		qy = self.odom.pose.pose.orientation.y
		qz = self.odom.pose.pose.orientation.z
		qw = self.odom.pose.pose.orientation.w

		roll, pitch, yaw = tf.transformations.euler_from_quaternion((qx, qy, qz, qw))

		error_yaw = self.steering_angle(goal_pose) - yaw

		error_yaw = atan2(sin(error_yaw),cos(error_yaw)) # to keep error between -pi,pi

		return constant * (error_yaw)

	def move2goal(self, goal_pose, distance_tolerance):
		"""Moves the rover to the goal."""

		vel_msg = Twist()

		start_time = time.time()

		while self.euclidean_distance(goal_pose) >= distance_tolerance:

			# Porportional controller.
			# https://en.wikipedia.org/wiki/Proportional_control

			# Linear velocity in the x-axis.
			vel_msg.linear.x = self.linear_vel(goal_pose, self.vel_gain)
			vel_msg.linear.y = 0
			vel_msg.linear.z = 0

			# Angular velocity in the z-axis.
			vel_msg.angular.x = 0
			vel_msg.angular.y = 0
			vel_msg.angular.z = self.angular_vel(goal_pose, self.yawrate_gain)

			# Publishing our vel_msg
			self.velocity_publisher.publish(vel_msg)

			# Publish at the desired rate.
			self.rate.sleep()

		# Stopping our robot after the movement is over.
		vel_msg.linear.x = 0
		vel_msg.angular.z = 0
		self.velocity_publisher.publish(vel_msg)

		elapsed = time.time() - start_time

		return elapsed


if __name__ == '__main__':
	try:
		roverObject = Rover()
		# If we press control + C, the node will stop.
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
