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


Combined State:
	[x00, y00, t00, x01, y01, t01, x02, y02, t02;
	 x10, y10, t10, x11, y11, t11, x12, y12, t12;
	 x20, y20, t20, x21, y21, t21, x22, y22, t22]
row: frame_id
column: car_id

"""

class Consensus(object):

	def __init__(self):
		self.rate = rospy.Rate(rospy.get_param("frequency", 1))

		self.epsilon = rospy.get_param("epsilon", 0.01)
		self.K = rospy.get_param("consensus_iterations", 20)
		self.N = rospy.get_param("number_of_nodes", 3)
		self.Ncars = rospy.get_param("number_of_cars", 3)
		self.Ndim = rospy.get_param("~num_state_dim", 3)

		self.xi_prior = None
		self.Ji_prior = None

		self.full_state = np.zeros((self.Ncars, self.Ncars*self.Ndim))
		self.B = np.zeros((self.Ncars, self.Ncars, self.Ndim, self.Ndim))

		self.x_sub = []
		for i in range(self.Ncars):
			self.x_sub.append(
				rospy.Subscriber("/car" + str(i) + "/states", CarState, self.x_cb))

	def x_cb(self, cs):
		frame_id = cs.header.frame_id
		u = cs.u
		state = cs.state
		ID = cs.car_id
		inf = np.array(cs.inf).reshape((self.Ndim,self.Ndim))

		self.full_state[int(frame_id[-1]), self.Ndim*ID : self.Ndim*(ID+1)] = np.array(state)
		self.B[int(frame_id[-1]), ID] = inf

		# self.full_state[]

	def run(self):
		while not rospy.is_shutdown():
			print self.full_state
			print self.B
			self.rate.sleep()

if __name__ == "__main__":
	rospy.init_node("consensus", anonymous=False)
	consensus = Consensus()
	consensus.run()