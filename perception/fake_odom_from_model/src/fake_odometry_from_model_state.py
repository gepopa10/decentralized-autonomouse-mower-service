#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseArray, TransformStamped
from tf2_ros import TransformBroadcaster
from gazebo_msgs.msg import LinkStates
import time
import tf
import numpy as np

'''
Class that will take as input the robot model and output an odometry message.
'''

class fake_odometry_from_model:

	def __init__(self):
		# Creates a unique node
		rospy.init_node('fake_odometry_from_model', anonymous=True)

		# A subscriber to an array of PoseStamped which is called when a message is received
		self.model_state_subscriber = rospy.Subscriber("/gazebo/link_states", LinkStates, self.callback_model_state)

		# Publisher which will publish the processed path
		self.odom_publisher = rospy.Publisher('/move_base_simple/fake_odom', Odometry, queue_size=10)

		# Transform broadcaster for odom tf
		self.odom_tf_broadcaster = TransformBroadcaster()

		# Publisher for uwb
		self.uwb_publisher = rospy.Publisher('/move_base_simple/uwbs', PoseArray, queue_size=10)

		self.dt = 0.01
		self.nb_callback = 0

		self.child_frame_id = rospy.get_param('/mobile_base_controller/base_frame_id', 'base_footprint')
		self.robot_model_name = rospy.get_param('/robot_model_name', 'waffle')
		self.robot_name = rospy.get_param('/robot_name', 'turtlebot3')
		self.robot_full_name = self.robot_name + "_" + self.robot_model_name
		self.uwb_link_name = "uwb" # with a number like uwb1 and also ::link after
		rospy.loginfo("Publishing odometry from link: %s", self.robot_full_name)

	def callback_model_state(self, state):
		"""Callback function which is called when a new message of type PoseStamped
		is received by the subscriber."""

		odom = self.model_state_to_odom(state)
		# Publishing
		# this is at 1000hz so we publish just 1/10
		if self.nb_callback == 10:
			self.nb_callback = 0
			self.odom_publisher.publish(odom)
			self.publish_uwb(state)

			odom_tf = TransformStamped()
			odom_tf.header = odom.header;
			odom_tf.child_frame_id = odom.child_frame_id;
			odom_tf.transform.translation.x = odom.pose.pose.position.x;
			odom_tf.transform.translation.y = odom.pose.pose.position.y;
			odom_tf.transform.translation.z = odom.pose.pose.position.z;
			odom_tf.transform.rotation = odom.pose.pose.orientation;

			# self.odom_tf_broadcaster.sendTransform(odom_tf) # we need to publish all so we use robot state publisher instead

		self.nb_callback += 1

	def model_state_to_odom(self, state):

		odom = Odometry()
		odom.header.stamp = rospy.Time.now()
		odom.header.frame_id = 'odom'
		odom.child_frame_id = self.child_frame_id

		name_array = state.name
		link_name = self.robot_full_name + "::" + self.child_frame_id

		# Might take some time until the model is there
		try:
			name_index = name_array.index(link_name) # we need to find it automatically turtlebot3_waffle::base_footprint
			pose = state.pose[name_index]
			twist = state.twist[name_index]

			odom.pose.pose = pose
			odom.twist.twist = twist

		except:
			rospy.logwarn("Did not find link: %s in list!", link_name)

		return odom

	def publish_uwb(self,state):

		# Might take some time until the model is there
		name_array = state.name

		try:
			name_indexes = [name_array.index(l) for l in name_array if l.startswith(self.uwb_link_name)]
			posearray = PoseArray();
			posearray.header.stamp = rospy.Time.now()
			posearray.header.frame_id = 'odom'

			for idx in name_indexes:
				pose = state.pose[idx]
				posearray.poses.append(pose)

			self.uwb_publisher.publish(posearray)

		except:
			rospy.logwarn("Did not find link: %s in list!", self.uwb_link_name)


if __name__ == '__main__':
	try:
		fake_odometry_from_model = fake_odometry_from_model()
		# If we press control + C, the node will stop.
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
