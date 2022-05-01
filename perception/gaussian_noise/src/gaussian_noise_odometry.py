#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseArray, TransformStamped
from tf2_ros import TransformBroadcaster
import time
import tf
import numpy as np
from math import pi, sqrt, atan2, cos, sin

'''
Class that will take as input an odometry topic and apply gaussian noise to it.
https://blog.lxsang.me/post/id/16
'''

class gaussian_noise_odometry:
	def __init__(self):
		# Creates a unique node
		rospy.init_node('gaussian_noise_odometry', anonymous=True)

		self.subscriber = rospy.Subscriber("/odom", Odometry, self.callback)
		self.publisher = rospy.Publisher('/noisy_odom', Odometry, queue_size=10)

		# Transform broadcaster for odom tf
		self.odom_tf_broadcaster = TransformBroadcaster()

		self.child_frame_id = rospy.get_param('/mobile_base_controller/base_frame_id', 'base_footprint')

		# Gaussian noise parameters
		self.sigma_pos = rospy.get_param('/position_noise', 0.0)
		self.sigma_theta = rospy.get_param('/theta_noise', 0.0)

	def callback(self, data):
		q = [ data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w ]
		(r, p, theta) = tf.transformations.euler_from_quaternion(q)

		noisy_pose = [ np.random.normal(data.pose.pose.position.x, self.sigma_pos),
						np.random.normal(data.pose.pose.position.y, self.sigma_pos),
		 				np.random.normal(theta, self.sigma_theta)]

		odom = self.pose_to_odom(noisy_pose)
		self.publisher.publish(odom)

	def pose_to_odom(self, pose):

		odom = Odometry()
		odom.header.stamp = rospy.Time.now()
		odom.header.frame_id = "odom"
		odom.child_frame_id = self.child_frame_id

		odom.pose.pose.position.x = pose[0]
		odom.pose.pose.position.y = pose[1]

		qx, qy, qz, qw = tf.transformations.quaternion_from_euler(0 , 0, pose[2])
		odom.pose.pose.orientation.x = qx
		odom.pose.pose.orientation.y = qy
		odom.pose.pose.orientation.z = qz
		odom.pose.pose.orientation.w = qw

		return odom

if __name__ == '__main__':
	try:
		gaussian_noise_odometry = gaussian_noise_odometry()
		# If we press control + C, the node will stop.
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
