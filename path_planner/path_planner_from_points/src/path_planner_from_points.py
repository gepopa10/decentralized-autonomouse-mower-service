#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
import time
import tf
import numpy as np

from visualization_msgs.msg import Marker, MarkerArray

'''
Class that will take as input a path, process it and output a processed path
with more inter waypoints points.
'''

class path_planner_from_points:

	def __init__(self):
		# Creates a unique node
		rospy.init_node('path_planner_from_points', anonymous=True)

		# A subscriber to an array of PoseStamped which is called when a message is received
		self.path_subscriber = rospy.Subscriber("/move_base_simple/path_raw", Path, self.callback_path)

		# Publisher which will publish the processed path
		self.path_publisher = rospy.Publisher('/move_base_simple/path', Path, queue_size=10)

		# Publisher marker
		self.raw_path_publisher_marker_array = rospy.Publisher('/move_base_simple/marker_array/path_raw', MarkerArray, queue_size=10)
		self.path_publisher_marker = rospy.Publisher('/move_base_simple/marker/path', Marker, queue_size=10)
		self.path_publisher_marker_array = rospy.Publisher('/move_base_simple/marker_array/path', MarkerArray, queue_size=10)

		self.dt = 0.01

	def callback_path(self, input_path):
		"""Callback function which is called when a new message of type PoseStamped
		is received by the subscriber."""

		nb_pts_path = len(input_path.poses)
		first_pose = input_path.poses[0]
		rospy.loginfo("Path command received with %i points, first position x: %0.2f y: %0.2f", nb_pts_path,
															first_pose.pose.position.x,
															first_pose.pose.position.y)

		self.create_path(input_path)

	def create_path(self, input_path):
		path = Path()
		path.header = input_path.header
		xspeed = 0.2#3
		yspeed = 0.2#3

		trajx = np.empty(0)
		trajy = np.empty(0)

		#converting input_path to a numpy array
		path_array = []
		for path_point in input_path.poses:
			path_array.append([path_point.pose.position.x, path_point.pose.position.y])

		input_path_array = np.array(path_array, dtype=np.float32)

		for i, wp in enumerate(input_path_array):
		    x1, y1 = wp
		    x2, y2 = input_path_array[i+1]

		    # for x
		    t2x = abs(x2-x1) / xspeed
		    t2y = abs(y2-y1) / yspeed

		    # with user
		    # generate values if dont move
		    if (abs(t2x) < 2*self.dt) & (abs(t2y) < 2*self.dt):
		        t2x = self.dt
		        t2y = self.dt
		    else:
		        # generate values even if position is constant in that axis
		        if abs(t2x) < 2*self.dt:
		            t2x = t2y
		        if abs(t2y) < 2*self.dt:
		            t2y = t2x

			x_list = np.linspace(x1, x2, int(t2x/self.dt))
			y_list = np.linspace(y1, y2, int(t2y/self.dt))
			# print(len(x_list),len(y_list))

			trajx = np.append(trajx, x_list)
			trajy = np.append(trajy, y_list)

		    # stop one step before the last
		    if i + 2 == np.shape(input_path_array)[0]:
		        break

		# print('Modified traj shape: ',np.shape(trajx), np.shape(trajy))
		traj = np.vstack((trajx, trajy)).T
		#traj = np.vstack((trajx[0::2], trajy[0::2])).T
		#print('Modified 2 traj shape: ',np.shape(traj))
		#traj = traj[0::2]


		#converting traj to an array to put in path
		for traj_point in traj:
			pose = PoseStamped()
			pose.header = input_path.header
			pose.pose.position.x = traj_point[0]
			pose.pose.position.y = traj_point[1]
			path.poses.append(pose)

		rospy.loginfo("Number of points in path after planning:  %i points.", len(path.poses))
		# Publishing
		self.path_publisher.publish(path)

		self.vizualize_paths(input_path, path)

	def vizualize_paths(self, input_path, path):
		count = 0

		#Publishing raw path
		points_raw_path = MarkerArray()

		for path_point in input_path.poses:
			point_raw_path = Marker()
			point_raw_path.header = input_path.header
			point_raw_path.lifetime = rospy.Duration()
			point_raw_path.id = count
			count +=1
			point_raw_path.type = point_raw_path.SPHERE
			point_raw_path.action = point_raw_path.ADD
			point_raw_path.pose.orientation.w = 1.0

			point_raw_path.scale.x = 0.05
			point_raw_path.scale.y = 0.05
			point_raw_path.scale.z = 0.05

			point_raw_path.color.a = 1.0
			point_raw_path.color.g = 1.0

			point_raw_path.pose.position.x = path_point.pose.position.x
			point_raw_path.pose.position.y = path_point.pose.position.y
			point_raw_path.pose.position.z = 0

			points_raw_path.markers.append(point_raw_path)

		self.raw_path_publisher_marker_array.publish(points_raw_path)

		#Publishing planned path
		points_path = MarkerArray()

		line_strip_path = Marker()
		line_strip_path.header = path.header
		line_strip_path.lifetime = rospy.Duration()
		line_strip_path.id = count;
		count +=1
		line_strip_path.type = line_strip_path.LINE_STRIP
		line_strip_path.action = line_strip_path.ADD
		line_strip_path.pose.orientation.w = 1.0

		line_strip_path.scale.x = 0.02

		line_strip_path.color.a = 1.0
		line_strip_path.color.b = 1.0

		for path_point in path.poses:
			point_path = Marker()
			point_path.header = path.header
			point_path.lifetime = rospy.Duration()
			point_path.id = count
			count +=1
			point_path.type = point_path.SPHERE
			point_path.action = point_path.ADD
			point_path.pose.orientation.w = 1.0

			point_path.scale.x = 0.01
			point_path.scale.y = 0.01
			point_path.scale.z = 0.01

			point_path.color.a = 1.0
			point_path.color.r = 1.0

			point_path.pose.position.x = path_point.pose.position.x
			point_path.pose.position.y = path_point.pose.position.y
			point_path.pose.position.z = 0

			points_path.markers.append(point_path)

			p = Point()
			p.x = path_point.pose.position.x
			p.y = path_point.pose.position.y
			p.z = 0

			line_strip_path.points.append(p)

		self.path_publisher_marker_array.publish(points_path)
		self.path_publisher_marker.publish(line_strip_path)

if __name__ == '__main__':
	try:
		path_planner_from_points = path_planner_from_points()
		# If we press control + C, the node will stop.
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
