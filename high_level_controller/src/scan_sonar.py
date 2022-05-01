#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan, Range
from math import pow, atan2, sqrt, sin, cos, pi
import math
import time
import tf


class Sonar:

	def __init__(self):
		# Creates a unique node
		rospy.init_node('sonar', anonymous=True)


		self.sonar_publisher = rospy.Publisher('/scan_sonar', LaserScan, queue_size=10)
		self.sonar_subscriber = rospy.Subscriber("/scan", LaserScan, self.CB_scan)

		self.sonar_0_publisher = rospy.Publisher('/sonar0', Range, queue_size=10)
		self.sonar_1_publisher = rospy.Publisher('/sonar1', Range, queue_size=10)
		self.sonar_2_publisher = rospy.Publisher('/sonar2', Range, queue_size=10)
		self.sonar_3_publisher = rospy.Publisher('/sonar3', Range, queue_size=10)

	def CB_scan(self, data):
		"""Callback function which is called when a new message of type PoseStamped
		is received by the subscriber."""
		scan_sonar = LaserScan()
		scan_sonar.header = data.header
		scan_sonar.angle_min = data.angle_min
		scan_sonar.angle_max = data.angle_max

		scan_sonar.angle_increment = pi/2 # in data it is 0.0175rad = 1deg so ranges[N] N from [0,359]

		scan_sonar.time_increment = data.time_increment
		scan_sonar.scan_time = data.scan_time
		scan_sonar.range_min = data.range_min
		scan_sonar.range_max = data.range_max

		scan_sonar.ranges = [float("inf"), float("inf"), float("inf"), float("inf")]
		scan_sonar.ranges[0] = data.ranges[0]
		scan_sonar.ranges[1] = data.ranges[90]
		scan_sonar.ranges[2] = data.ranges[190]
		scan_sonar.ranges[3] = data.ranges[270]


		self.sonar_publisher.publish(scan_sonar)

		sonar = Range()
		sonar.header = data.header
		sonar.radiation_type = 0 # uint8 ULTRASOUND=0
		sonar.field_of_view = 0.01 # in rad, magni robot = 0.5
		sonar.min_range = 0.02 # in m, same as magni robot
		sonar.max_range = 4.0 # in m, same as magni robot

		sonar_0 = sonar
		sonar_0.range = scan_sonar.ranges[0]
		self.sonar_0_publisher.publish(sonar_0)

		sonar_1 = sonar
		sonar_1.range = scan_sonar.ranges[1]
		self.sonar_1_publisher.publish(sonar_1)

		sonar_2 = sonar
		sonar_2.range = scan_sonar.ranges[2]
		self.sonar_2_publisher.publish(sonar_2)

		sonar_3 = sonar
		sonar_3.range = scan_sonar.ranges[3]
		self.sonar_3_publisher.publish(sonar_3)

		print('published at:',time.time())


if __name__ == '__main__':
	try:
		roverObject = Sonar()
		# If we press control + C, the node will stop.
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
