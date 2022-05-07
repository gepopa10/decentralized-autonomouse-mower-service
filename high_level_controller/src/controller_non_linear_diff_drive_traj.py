#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseStamped, Point, Pose, PoseArray
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64
from math import pow, atan2, sqrt, sin, cos, atan, copysign, pi
import time
import tf
import numpy as np
import NonLinearDifferentialController
from visualization_msgs.msg import Marker, MarkerArray
from copy import deepcopy

from dynamic_reconfigure.server import Server
from high_level_controller.cfg import controller_diff_driveConfig

class Rover:

	def __init__(self):
		# Creates a unique node
		rospy.init_node('controller_non_linear_diff_drive', anonymous=True)

		# Publisher which will publish to the command topic
		self.velocity_publisher = rospy.Publisher('/cmd_vel',Twist, queue_size=10)

		# Markers publishers
		self.goals_publisher_marker = rospy.Publisher('/move_base_simple/marker/next_goals', Marker, queue_size=10)
		self.goals_publisher_marker_array = rospy.Publisher('/move_base_simple/marker/next_goals_array', MarkerArray, queue_size=10)
		self.footprints_publisher_marker = rospy.Publisher('/move_base_simple/marker/footprints', Marker, queue_size=10)
		self.uwb_publisher_marker = rospy.Publisher('/move_base_simple/marker/uwbs', Marker, queue_size=10)
		self.mesh_publisher_marker = rospy.Publisher('/move_base_simple/marker/mesh', Marker, queue_size=10)

		# A subscriber to Odometry which is called when a message is received
		self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.callback_odom)

		# A subscriber to Odometry which is called when a message is received
		self.noisy_odom_subscriber = rospy.Subscriber("/noisy_odom", Odometry, self.callback_noisy_odom)

		# A subscriber to an array of PoseStamped which is called when a message is received
		self.path_subscriber = rospy.Subscriber("/move_base_simple/path", Path, self.callback_path)

		# A subscriber to Odometry which is called when a message is received
		self.uwb_subscriber = rospy.Subscriber("/move_base_simple/uwbs", PoseArray, self.callback_uwb)

		self.initialized_controller = False

		# Desired goal
		self.goal_publisher = rospy.Publisher('/move_base_simple/current_goal', PoseStamped, queue_size=10)

		# Mission finished
		self.mission_finished_publisher = rospy.Publisher('/move_base_simple/mission_finished', Float64, queue_size=10)

		# Config server
		self.dynamic_reconfig_srv = Server(controller_diff_driveConfig, self.callback_dynamic_reconfig)

		self.start_path_time = time.time()
		self.odom = Odometry()
		self.noisy_odom = Odometry()
		self.path = Path()
		self.goal_pose = PoseStamped()
		self.goal_pose.header.stamp = rospy.Time.now()
		self.yaw = 0
		self.yaw_des = 0

		self.dt = 0.01

		self.L = 0.574
		self.r = 0.066
		self.kp_pos = 5
		self.kp_theta = 100
		self.distance_tolerance = 0.02
		self.yaw_tolerance = 0.02
		self.turning_speed = 50

		self.nb_pts_path = 0
		self.reached_start_traj = False
		self.traj_incr = 0
		self.end_of_path = False
		self.received_path = False

		self.nb_forward_goals_to_show = 5 #redifined later, currently we use self.forward_dist_to_show # in meters
		self.forward_dist_to_show = 2 # in meters

		self.count_footprint = 0
		self.footprint_marker = Marker()
		self.footprint_markers = []
		self.lifetime_footprints = rospy.Duration(1/self.dt) # to be sure to have enough fps when we have a lot of markers
		self.high_performance_computer = False
		self.repub_all_footprints_end = False

		self.uwb_marker = Marker()
		self.uwb_marker_box = Marker()
		self.publish_mesh_markers()

	def callback_odom(self, data):
		self.odom = data

	def callback_noisy_odom(self, data):
		"""Callback function which is called when a new message of type Odometry
		is received by the subscriber."""
		while True:
			try:
				dt = (data.header.stamp - self.noisy_odom.header.stamp).to_sec() # first will be a big number
				break
			except:
				time.sleep(0.05) # give time for self.noisy_odom object to be created

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

		self.noisy_odom = data
		if self.initialized_controller:
			self.path_controller()

	def callback_path(self, data):
		"""Callback function which is called when a new message of type PoseStamped
		is received by the subscriber."""
		self.start_path_time = time.time()
		self.path = data
		self.nb_pts_path = len(self.path.poses)
		self.nb_forward_goals_to_show = int(0.01*self.nb_pts_path) # 1% of total path
		first_pose = self.path.poses[0]
		self.received_path = True
		self.reached_start_traj = False
		self.end_of_path = False
		self.traj_incr = 0
		rospy.loginfo("Path command received with %i points, first position x: %0.2f y: %0.2f", self.nb_pts_path,
		                                                    first_pose.pose.position.x,
															first_pose.pose.position.y)
		self.publish_mesh_markers()

	def callback_uwb(self, data):
		"""Callback function which is called when a new message of type PoseArray
		is received by the subscriber."""

		self.uwb_marker.header = data.header
		self.uwb_marker.id = 0
		self.uwb_marker.ns = "uwbs_markers" # same id and same namespace overwrite the other marker so no timelife required
		self.uwb_marker.type = self.uwb_marker.LINE_LIST
		self.uwb_marker.action = self.uwb_marker.ADD
		self.uwb_marker.pose.orientation.w = 1.0
		self.uwb_marker.frame_locked = True

		self.uwb_marker.scale.x = 0.02

		self.uwb_marker.color.a = 1.0
		self.uwb_marker.color.r = 0.96
		self.uwb_marker.color.g = 0.74
		self.uwb_marker.color.b = 0.0

		for uwb_pose in data.poses:
			p1 = Point()
			p1.x = self.noisy_odom.pose.pose.position.x
			p1.y = self.noisy_odom.pose.pose.position.y
			p1.z = self.noisy_odom.pose.pose.position.z + 0.216 + 0.063 # this is the top of the scanner height

			self.uwb_marker.points.append(p1)

			p2 = Point()
			p2.x = uwb_pose.position.x
			p2.y = uwb_pose.position.y
			p2.z = uwb_pose.position.z

			self.uwb_marker.points.append(p2)

		self.uwb_publisher_marker.publish(self.uwb_marker)
		self.uwb_marker.points = [] # so we dont keep adding to list and view all lines

		for i in range(len(data.poses)):
			self.uwb_marker_box.header = data.header
			self.uwb_marker_box.id = i
			self.uwb_marker_box.ns = "uwbs_boxes_markers"
			self.uwb_marker_box.type = self.uwb_marker.CUBE
			self.uwb_marker_box.action = self.uwb_marker.ADD
			self.uwb_marker_box.pose.orientation.w = 1.0
			self.uwb_marker_box.frame_locked = True

			scale = 5 # so we see something because they are too small
			self.uwb_marker_box.scale.x = scale*0.04 # size of uwbs
			self.uwb_marker_box.scale.y = scale*0.01
			self.uwb_marker_box.scale.z = scale*0.06

			self.uwb_marker_box.pose.position.x = data.poses[i].position.x
			self.uwb_marker_box.pose.position.y = data.poses[i].position.y
			self.uwb_marker_box.pose.position.z = data.poses[i].position.z

			self.uwb_marker_box.color.a = 1.0
			self.uwb_marker_box.color.r = 1.0
			self.uwb_publisher_marker.publish(self.uwb_marker_box)

	def euclidean_distance(self, goal_pose):
		"""Euclidean distance between current pose and the goal."""
		return sqrt(pow((goal_pose.pose.position.x - self.noisy_odom.pose.pose.position.x), 2) +
				pow((goal_pose.pose.position.y - self.noisy_odom.pose.pose.position.y), 2))

	def path_controller(self):
		"""Moves the rover to the goal."""

		vel_msg = Twist()

		# if self.reached_start_traj:
		# 	self.get_next_goal_from_path()

		# if self.euclidean_distance(self.goal_pose) > self.distance_tolerance:
		# 	self.controller.update_params(20, 50) #puts gains that are good for position control until reach first point of traj

		if self.received_path and not self.end_of_path:

			qx = self.noisy_odom.pose.pose.orientation.x
			qy = self.noisy_odom.pose.pose.orientation.y
			qz = self.noisy_odom.pose.pose.orientation.z
			qw = self.noisy_odom.pose.pose.orientation.w

			roll, pitch, yaw = tf.transformations.euler_from_quaternion((qx, qy, qz, qw))

			x = self.noisy_odom.pose.pose.position.x
			y = self.noisy_odom.pose.pose.position.y

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
			self.publish_next_goals()
			if self.high_performance_computer:
				self.publish_footprint_cube()
			else:
				self.publish_footprint_circle() # self.publish_footprint_line_strip() is a bit easier on cpu than cube

			# if self.euclidean_distance(self.goal_pose) < self.distance_tolerance: #get next pts of traj only when close enough
			self.get_next_goal_from_path()

		elif self.end_of_path:
			if self.repub_all_footprints_end:
				self.publish_complete_path_footprints()
			# Stopping robot
			vel_msg.linear.x = 0
			vel_msg.angular.z = 0
			if self.received_path:
				duration_min = (time.time()- self.start_path_time)/60;
				rospy.loginfo("Path finished!!! in %i min.",round(duration_min,2))
				duration_msg = Float64()
				duration_msg.data = duration_min
				self.mission_finished_publisher.publish(duration_msg)
			self.received_path = False

		# Publishing our vel_msg
		self.velocity_publisher.publish(vel_msg)

		#Publishing current goal to reach
		self.goal_publisher.publish(self.goal_pose)

	def get_next_goal_from_path(self):
		self.goal_pose = self.path.poses[self.traj_incr]
		self.goal_pose.header.stamp = rospy.Time.now()
		self.traj_incr += 1
		if self.traj_incr == self.nb_pts_path:
			self.end_of_path = True

	def publish_next_goals(self):

		line_strip_path = Marker()
		line_strip_path.header = self.path.header
		line_strip_path.lifetime = rospy.Duration()
		line_strip_path.id = 100
		line_strip_path.type = line_strip_path.LINE_STRIP
		line_strip_path.action = line_strip_path.ADD
		line_strip_path.pose.orientation.w = 1.0

		line_strip_path.scale.x = 0.05

		line_strip_path.color.a = 1.0
		line_strip_path.color.b = 1.0

		points_path = MarkerArray()

		nb_forward_goals_to_show = self.nb_forward_goals_to_show
		# at the end of the traj we reduce the number of points we display
		if nb_forward_goals_to_show > self.nb_pts_path - self.traj_incr:
			nb_forward_goals_to_show = self.nb_pts_path - self.traj_incr

		# Other method to find nb of forward pts based on distance
		forward_dist_to_show = self.forward_dist_to_show # in meters
		dist = 0
		j = 0
		nb_forward_goals_to_show = 0
		while dist < forward_dist_to_show:
			try:
				dist += sqrt((self.path.poses[self.traj_incr + j + 1].pose.position.x - self.path.poses[self.traj_incr + j].pose.position.x)**2 + (self.path.poses[self.traj_incr + j + 1].pose.position.y - self.path.poses[self.traj_incr + j].pose.position.y)**2)
				j += 1
				nb_forward_goals_to_show +=1
			except:
				break
				pass

		count = 0
		for i in range(nb_forward_goals_to_show):
			path_point = self.path.poses[self.traj_incr + i]

			# Draw points
			# point_path = Marker()
			# point_path.header = self.path.header
			# # point_path.lifetime = rospy.Duration(0.001)
			# point_path.id = count
			# count +=1
			# point_path.type = point_path.SPHERE
			# point_path.action = point_path.ADD
			# point_path.pose.orientation.w = 1.0
			#
			# point_path.scale.x = 0.1
			# point_path.scale.y = 0.1
			# point_path.scale.z = 0.1
			#
			# point_path.color.a = 1.0
			# point_path.color.b = 1.0
			#
			# point_path.pose.position.x = path_point.pose.position.x
			# point_path.pose.position.y = path_point.pose.position.y
			# point_path.pose.position.z = 0
			#
			# points_path.markers.append(point_path)

			# Draw continous line
			p = Point()
			p.x = path_point.pose.position.x
			p.y = path_point.pose.position.y
			p.z = 0

			line_strip_path.points.append(p)

		# Show only some pts
		nb_elemts_to_show = 5
		idx = np.round(np.linspace(0, len(line_strip_path.points) - 1, nb_elemts_to_show)).astype(int)
		small_arr = []
		small_arr_markers = []
		if line_strip_path.points: #check if we have some points
			for i in idx:
				small_arr.append(line_strip_path.points[i])
				# small_arr_markers.append(points_path.markers[i])

		line_strip_path.points = small_arr
		# points_path.markers = small_arr_markers

		self.goals_publisher_marker.publish(line_strip_path)
		# self.goals_publisher_marker_array.publish(points_path)

	def publish_footprint_cube(self):

		self.footprint_marker.header = self.odom.header
		self.footprint_marker.id = self.count_footprint
		#self.footprint_marker.lifetime = self.lifetime_footprints*3 # *3 because add pub spheres in corners and less frequent, so its easier on cpu
		self.count_footprint +=1
		self.footprint_marker.type = self.footprint_marker.CUBE
		self.footprint_marker.action = self.footprint_marker.ADD
		self.footprint_marker.pose.orientation.w = 1.0

		self.footprint_marker.scale.x = self.L
		self.footprint_marker.scale.y = self.L
		self.footprint_marker.scale.z = 0.01

		self.footprint_marker.color.a = 0.5
		self.footprint_marker.color.g = 1.0

		self.footprint_marker.pose.position.x = self.odom.pose.pose.position.x
		self.footprint_marker.pose.position.y = self.odom.pose.pose.position.y
		self.footprint_marker.pose.position.z = 0.01

		# we append only when we have to a complete footprint size
		if len(self.footprint_markers) == 0:
			self.footprint_markers.append(deepcopy(self.footprint_marker))
			self.footprints_publisher_marker.publish(self.footprint_marker)
		else:
			if (sqrt(pow((self.footprint_marker.pose.position.x - self.footprint_markers[-1].pose.position.x), 2) + \
			    pow((self.footprint_marker.pose.position.y - self.footprint_markers[-1].pose.position.y), 2)) > self.L) \
				or (abs(self.odom.twist.twist.angular.z) > 0.1 and \
				   (sqrt(pow((self.footprint_marker.pose.position.x - self.footprint_markers[-1].pose.position.x), 2) + \
				   pow((self.footprint_marker.pose.position.y - self.footprint_markers[-1].pose.position.y), 2)) > self.L/3)): # when we turn we need more pts

				if abs(self.odom.twist.twist.angular.z) > 0.1:
					self.footprint_marker.color.a = 0.5 # change transparency when we publish a lot of markers in corner
					self.footprint_marker.type = self.footprint_marker.SPHERE # change type also
					self.footprints_publisher_marker.publish(self.footprint_marker)
					self.last_marker_published = "SPHERE"
				else:
					if self.last_marker_published != "CUBE": # we change from sphere to cube
						# we need to change the scale to have a square from the beggining of the center of the last sphere published
						self.footprint_marker.pose.position.x = (self.footprint_markers[-1].pose.position.x + \
						                                        self.footprint_marker.pose.position.x)/2 # recenter the square
						self.footprint_marker.scale.x = 2*self.L # longer to cover sphere intersection and also where it was supposed to be
						self.footprint_marker.scale.y = self.L

					self.footprints_publisher_marker.publish(self.footprint_marker)
					self.last_marker_published = "CUBE"

				self.footprint_markers.append(deepcopy(self.footprint_marker))

	def publish_footprint_line_strip(self):

		self.footprint_marker.header = self.odom.header
		self.footprint_marker.id = self.count_footprint
		self.count_footprint +=1
		self.footprint_marker.lifetime = self.lifetime_footprints*2 # *3 because compared to cube line_strip are less performance heavy and can stay longer in rviz scene without lowering fps
		self.footprint_marker.type = self.footprint_marker.LINE_STRIP
		self.footprint_marker.action = self.footprint_marker.ADD
		self.footprint_marker.pose.orientation.x = 0.0
		self.footprint_marker.pose.orientation.y = 0.0
		self.footprint_marker.pose.orientation.z = 0.0
		self.footprint_marker.pose.orientation.w = 1.0
		self.footprint_marker.frame_locked = True

		self.footprint_marker.scale.x = self.L

		self.footprint_marker.color.a = 1.0
		self.footprint_marker.color.g = 1.0

		p = Point()
		p.x = self.odom.pose.pose.position.x
		p.y = self.odom.pose.pose.position.y
		p.z = self.odom.pose.pose.position.z

		# we append only when we have to a complete footprint size
		if len(self.footprint_markers) == 0:
			self.footprint_markers.append(p)
			self.footprint_marker.points.append(p)
		else:
			if (sqrt(pow((p.x - self.footprint_markers[-1].x), 2) + pow((p.y - self.footprint_markers[-1].y), 2)) > self.L) \
				or (abs(self.odom.twist.twist.angular.z) > 0.1 and \
				   (sqrt(pow((p.x - self.footprint_markers[-1].x), 2) + \
				   pow((p.y - self.footprint_markers[-1].y), 2)) > self.L/10)): # when we turn we need more pts

				self.footprint_markers.append(p)
				self.footprint_marker.points.append(p)

				self.footprints_publisher_marker.publish(self.footprint_marker)
				self.footprint_marker.points.pop(0) # to always keep 2 points only

	def publish_footprint_circle(self):

		self.footprint_marker.header = self.odom.header
		self.footprint_marker.id = self.count_footprint
		self.count_footprint +=1
		self.footprint_marker.type = self.footprint_marker.SPHERE
		self.footprint_marker.action = self.footprint_marker.ADD
		self.footprint_marker.pose.orientation.w = 1.0

		self.footprint_marker.scale.x = self.L
		self.footprint_marker.scale.y = self.L
		self.footprint_marker.scale.z = 0.01

		self.footprint_marker.color.a = 0.5
		self.footprint_marker.color.g = 1.0

		self.footprint_marker.pose.position.x = self.odom.pose.pose.position.x
		self.footprint_marker.pose.position.y = self.odom.pose.pose.position.y
		self.footprint_marker.pose.position.z = 0.01

		# we append only when we have to a complete footprint size
		if len(self.footprint_markers) == 0:
			self.footprint_markers.append(deepcopy(self.footprint_marker))
			self.footprints_publisher_marker.publish(self.footprint_marker)
		else:
			if (sqrt(pow((self.footprint_marker.pose.position.x - self.footprint_markers[-1].pose.position.x), 2) + \
			    pow((self.footprint_marker.pose.position.y - self.footprint_markers[-1].pose.position.y), 2)) > self.L/2) \
				or (abs(self.odom.twist.twist.angular.z) > 0.1 and \
				   (sqrt(pow((self.footprint_marker.pose.position.x - self.footprint_markers[-1].pose.position.x), 2) + \
				   pow((self.footprint_marker.pose.position.y - self.footprint_markers[-1].pose.position.y), 2)) > self.L/3)): # when we turn we need more pts

				self.footprint_markers.append(deepcopy(self.footprint_marker))
				self.footprints_publisher_marker.publish(self.footprint_marker)

	def publish_complete_path_footprints(self):
		for marker in self.footprint_markers:
			self.footprints_publisher_marker.publish(marker)
			time.sleep(0.02)

	def callback_dynamic_reconfig(self, config, level):
		rospy.loginfo("""Reconfigure Request: {kp_pos}, {kp_theta}""".format(**config))
		self.kp_pos = config.kp_pos
		self.kp_theta = config.kp_theta
		if self.initialized_controller:
			self.controller.update_params(self.kp_pos, self.kp_theta)
		return config

	def publish_mesh_markers(self):

		mesh_marker = Marker()
		mesh_marker.header.stamp = rospy.Time.now()
		mesh_marker.header.frame_id = "odom"
		mesh_marker.type = mesh_marker.MESH_RESOURCE
		mesh_marker.action = mesh_marker.ADD

		mesh_marker.pose.orientation.w = 1.0

		mesh_marker.scale.x = 1
		mesh_marker.scale.y = 1
		mesh_marker.scale.z = 1

		mesh_marker.color.a = 0.5
		mesh_marker.color.r = 1.0
		mesh_marker.color.g = 1.0
		mesh_marker.color.b = 1.0

		mesh_marker.pose.position.x = 12
		mesh_marker.pose.position.y = -7.2
		mesh_marker.pose.position.z = 0
		mesh_marker.mesh_resource = "package://turtlebot3_gazebo/models/oak_tree/meshes/oak_tree.dae";
		mesh_marker.ns = "tree_marker"
		self.mesh_publisher_marker.publish(mesh_marker)

		mesh_marker.pose.position.x = 0.532021
		mesh_marker.pose.position.y = -3.01861
		quaternion = tf.transformations.quaternion_from_euler(0, 0, 1.55261)
		mesh_marker.pose.orientation.x = quaternion[0]
		mesh_marker.pose.orientation.y = quaternion[1]
		mesh_marker.pose.orientation.z = quaternion[2]
		mesh_marker.pose.orientation.w = quaternion[3]

		mesh_marker.scale.x = 1.5
		mesh_marker.scale.y = 1.5
		mesh_marker.scale.z = 1.5
		mesh_marker.mesh_resource = "package://turtlebot3_gazebo/models/house_2/meshes/house_2.dae";
		mesh_marker.ns = "house_marker"
		self.mesh_publisher_marker.publish(mesh_marker)

		mesh_marker.pose.position.x = 12
		mesh_marker.pose.position.y = 0.9
		mesh_marker.scale.x = 1.8
		mesh_marker.scale.y = 1.8
		mesh_marker.scale.z = 1.5
		mesh_marker.mesh_resource = "package://turtlebot3_gazebo/models/gazebo_scalable/meshes/gazebo.dae";
		mesh_marker.ns = "gazebo_marker"
		self.mesh_publisher_marker.publish(mesh_marker)

		# common for all walkways
		mesh_marker.scale.x = 0.001
		mesh_marker.scale.y = 0.001
		mesh_marker.scale.z = 0.001

		mesh_marker.pose.position.x = 9.4322
		mesh_marker.pose.position.y = -1.58504
		quaternion = tf.transformations.quaternion_from_euler(0, 0, 2.31451)
		mesh_marker.pose.orientation.x = quaternion[0]
		mesh_marker.pose.orientation.y = quaternion[1]
		mesh_marker.pose.orientation.z = quaternion[2]
		mesh_marker.pose.orientation.w = quaternion[3]

		mesh_marker.mesh_resource = "package://turtlebot3_gazebo/models/walkway_metal_45/meshes/mesh.obj";
		mesh_marker.ns = "walkway1_marker"
		self.mesh_publisher_marker.publish(mesh_marker)

		mesh_marker.pose.position.x = 3.58029
		mesh_marker.pose.position.y = -1.9811
		quaternion = tf.transformations.quaternion_from_euler(0, 0, -1.58962)
		mesh_marker.pose.orientation.x = quaternion[0]
		mesh_marker.pose.orientation.y = quaternion[1]
		mesh_marker.pose.orientation.z = quaternion[2]
		mesh_marker.pose.orientation.w = quaternion[3]

		mesh_marker.mesh_resource = "package://turtlebot3_gazebo/models/walkway_metal_straight/meshes/mesh.obj";
		mesh_marker.ns = "walkway2_marker"
		self.mesh_publisher_marker.publish(mesh_marker)

		mesh_marker.pose.position.x = 5.10698
		mesh_marker.pose.position.y = -2.01247

		mesh_marker.ns = "walkway3_marker"
		self.mesh_publisher_marker.publish(mesh_marker)

		mesh_marker.pose.position.x = 6.62403
		mesh_marker.pose.position.y = -2.04188

		mesh_marker.ns = "walkway4_marker"
		self.mesh_publisher_marker.publish(mesh_marker)

		mesh_marker.pose.position.x = 9.40829
		mesh_marker.pose.position.y = -1.59261
		quaternion = tf.transformations.quaternion_from_euler(0, 0, -0.824284)
		mesh_marker.pose.orientation.x = quaternion[0]
		mesh_marker.pose.orientation.y = quaternion[1]
		mesh_marker.pose.orientation.z = quaternion[2]
		mesh_marker.pose.orientation.w = quaternion[3]

		mesh_marker.ns = "walkway5_marker"
		self.mesh_publisher_marker.publish(mesh_marker)

if __name__ == '__main__':
	try:
		roverObject = Rover()
		# If we press control + C, the node will stop.
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
