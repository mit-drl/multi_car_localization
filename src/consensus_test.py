#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped
from multi_car_localization.msg import CarMeasurement
from multi_car_localization.msg import CarState
import math
import random

"""
State:
    [x0, y0
     x1, y1
     x2, y2]

Control:
    [dx0, dy0
     dx1, dy1
     dx2, dy2]

Measurements:
    [gps_x0, gps_y0, uwb_00, uwb_01, uwb_02;
     gps_x1, gps_y1, uwb_10, uwb_11, uwb_12;
     gps_x2, gps_y2, uwb_20, uwb_21, uwb_22]

"""

class Consensus(object):

	def __init__(self):
		self.rate = rospy.Rate(rospy.get_param("frequency", 50))

		self.epsilon = rospy.get_param("epsilon", 0.01)
		self.K = rospy.get_param("consensus_iterations", 20)
		self.N = rospy.get_param("number_of_nodes", 3)
		self.Ncars = rospy.get_param("number_of_cars", 3)


		self.xi_prior = None
		self.Ji_prior = None

		self.x_sub = {}
		for i in range(self.Ncars):
			self.x_sub.append("/car" + str(i) + "/state", CarState, self.x_cb)

	def x_cb(self, cs):
		frame_id = cm.header.frame_id
		u = cm.u
		state = cm.state
		ps = cm.ps



	def pose_sub_cb(self, state):
		frame_id = state.header.frame_id

	def publish_pose(self):
		ps = PoseStamped()
		ps.header.frame_id = self.frame_id
		ps.pose.position.x = self.x
		ps.pose.position.y = self.y
		self.pose_pub.publish(ps)


	def range_sub_cb(self, meas):
		for rng in meas.ranges:
			self.ranges[rng.header.frame_id] = rng.range

	# need to make this real NavSat
	def gps_sub_cb(self, gps):
		self.gps = gps

	def run(self):
		while not rospy.is_shutdown():
			self.publish_pose()
			self.rate.sleep()

if __name__ == "__main__":
	uwb_id = rospy.get_param("~uwb_id", "uwb0")
	frame_id = rospy.get_param("~frame_id", "car0")

	rospy.init_node("car", anonymous=False)
	car = Car()
	car.run()