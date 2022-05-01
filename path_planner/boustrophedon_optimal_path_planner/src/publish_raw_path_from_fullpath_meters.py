#!/usr/bin/env python

from __future__ import division # to have 2 ints divided and get a float as result
from boustrophedon_optimal_path_planner.srv import publish_raw_path_from_fullpath_meters
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
import pyclipper
import rospy
import os
import time

def handle_publish_raw_path_from_fullpath_meters(req):
    rospy.loginfo("Processing file: <%s> to publish %i waypoints.", req.input_file_name, req.nb_waypoints_path)
    nb_waypoints_path_published = publish_path_from_file(req.input_file_name, req.nb_waypoints_path)
    return str(nb_waypoints_path_published) + " waypoints have been published to /move_base_simple/path."

def publish_raw_path_from_fullpath_meters_server():
    rospy.init_node('publish_raw_path_from_fullpath_meters_server')
    s = rospy.Service('publish_raw_path_from_fullpath_meters', publish_raw_path_from_fullpath_meters, handle_publish_raw_path_from_fullpath_meters)
    rospy.loginfo("Ready to publish_raw_path_from_fullpath_meters.")
    rospy.spin()

def publish_path_from_file(filename, nb_waypoints_path):
    # write to text file
    this_folder = os.path.dirname(os.path.abspath(__file__))
    fname = os.path.join(this_folder + "/data_files", filename)
    file = open(fname,'r')

    firstline = file.readline()
    L = (firstline.strip()).split(' ')

    fullpath = list()
    for i in range(0, len(L) - 1, 2) :
        x = float(L[i])
        y = float(L[i + 1])
        point = (x, y)
        fullpath.append(point)

    # nth_element = len(fullpath)/nb_waypoints_path
    # fullpath_shorten = fullpath[::nth_element] # decrease the size of the path

    nb_increase = 4 # increase to cut better in corners (so robot has time to finish corner before continue), to be adjusted to have a cmd_vel speed between 0.2-0.5 in straight line
    fullpath_shorten =  [ele for ele in fullpath for i in range(nb_increase)] # increase the size of the path

    # Publisher which will publish the path from file
    path_publisher = rospy.Publisher('/move_base_simple/path', Path, queue_size=10)

    # Publisher for markers on permimeter and obstacles
    perimeter_publisher_marker = rospy.Publisher('/move_base_simple/marker/perimeter', Marker, queue_size=10)

    time.sleep(2) # so the pub is fully created before using it

    path = Path()
    path.header.stamp = rospy.Time.now()
    path.header.frame_id = "odom"

    vizualize_perimeter_obstacles(this_folder, path, perimeter_publisher_marker)
    time.sleep(5)

    for x,y in fullpath_shorten :
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = x
        pose.pose.position.y = y
        path.poses.append(pose)

    while path_publisher.get_num_connections() < 1: #need [two (rviz) or 1 (gazebo)] here because we have listeners
        # wait for a connection to publisher
        time.sleep(0.01)
        pass

    rospy.loginfo("Publishing a path of %i points from a path of %i waypoints...", len(path.poses), len(fullpath))
    # Publishing
    path_publisher.publish(path)

    file.close()

    return len(path.poses)

def vizualize_perimeter_obstacles(this_folder, path, perimeter_publisher_marker):

	fname = os.path.join(this_folder + "/data_files", "input_raw.txt")
	input = open(fname,'r')
	# Publish perimeter lines
	perimeter_in = list()
	firstline = input.readline()
	L = (firstline.strip()).split(' ')
	for i in range(0, len(L) - 1, 2) :
		# we get the centroid at the beginning
		if i == 0:
			x0 = int(L[i])
			y0 = int(L[i + 1])
		else:
			vertex = (int(L[i]), int(L[i + 1]))
			perimeter_in.append(vertex)

	count = 0
	line_strip = Marker()
	line_strip.header = path.header
	line_strip.id = count
	count +=1
	line_strip.type = line_strip.LINE_STRIP
	line_strip.action = line_strip.ADD
	line_strip.pose.orientation.w = 1.0

	line_strip.scale.x = 0.05

	line_strip.color.a = 1.0
	line_strip.color.r = 0.0
	line_strip.color.g = 0.0
	line_strip.color.b = 0.0

	margin = 12 # should be the same as robot_radius in optimal_path_planner.py
	perimeter = dpi_to_cartesian(perimeter_in, margin)

	line_strip = fill_line_strip(perimeter, line_strip)

	perimeter_publisher_marker.publish(line_strip)

	# Publish obstacles lines
	polygons = list()
	for line in input :
		polygon = line_to_vertex(line)
		polygons.append(polygon)

	for polygon in polygons:
		polygon = dpi_to_cartesian(polygon, -margin)

		line_strip = Marker()
		line_strip.header = path.header
		line_strip.id = count
		count +=1
		line_strip.type = line_strip.LINE_STRIP
		line_strip.action = line_strip.ADD
		line_strip.pose.orientation.w = 1.0

		line_strip.scale.x = 0.05

		line_strip.color.a = 1.0
		line_strip.color.r = 0.0
		line_strip.color.g = 0.0
		line_strip.color.b = 0.0

		line_strip = fill_line_strip(polygon, line_strip)

		perimeter_publisher_marker.publish(line_strip)

	input.close()
	rospy.loginfo("Publishing perimeter of %i points and also %i obstacles points...", len(perimeter), len(polygons))

def dpi_to_cartesian(perimeter, margin): # margin should be negative for obstacle
    pco = pyclipper.PyclipperOffset()
    pco.AddPath(perimeter, pyclipper.JT_ROUND, pyclipper.ET_CLOSEDPOLYGON) # JT_MITER, JT_ROUND, JT_SQUARE
    solutions = pco.Execute(margin) # dont return the last point, we need to add the first at the end
    # These params should be the same as in perimeter.py
    translation = {'x':0, 'y':11.1}
    dpi = 2000
    range_meters = 20
    min_fitted = 0
    max_fitted = range_meters

    # Transform from dpi based coordinates to cartesian
    perimeter_cartesian = []
    for i in range(len(solutions[0])):
        x_norm = int(solutions[0][i][0])/dpi
        y_norm = int(solutions[0][i][1])/dpi
        # denormalize
        x = (x_norm - 1)*(max_fitted - min_fitted) + max_fitted - translation['x'] # we remove the translation we added before
        y = (y_norm - 1)*(max_fitted - min_fitted) + max_fitted - translation['y']
        point = (x, y)
        perimeter_cartesian.append(point)

    perimeter_cartesian.append(perimeter_cartesian[0])
    return perimeter_cartesian

def line_to_vertex(line):
    L = (line.strip()).split(' ')
    polygon = list()
    for i in range(0, len(L) - 1, 2) :
        vertex = (int(L[i]), int(L[i + 1]))
        polygon.append(vertex)

    return polygon

def fill_line_strip(data, line_strip):
	for i in range(len(data)-1):
		p = Point()
		p.x = data[i][0]
		p.y = data[i][1]
		p.z = 0

		line_strip.points.append(p)

	# we put at the end the first point to loop the contour
	p = Point()
	p.x = data[0][0]
	p.y = data[0][1]
	p.z = 0
	line_strip.points.append(p)

	return line_strip

if __name__ == "__main__":
    publish_raw_path_from_fullpath_meters_server()
