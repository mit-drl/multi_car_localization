#!/usr/bin/env python

import roshelper
import rospy
from sensor_msgs.msg import Range

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

z0 = [gps_x0; gps_y0; uwb_01; uwb_02]
x0 = [x0, y0]
x = [x0, y0, x1, y1, x2, y2]
z0 = H0*x
H0 = [1 0 0 0 0 0
	  0 1 0 0 0 0
	  (x0 - x1)^2 + (y0 - y1)^2
	  (x0 - x2)^2 + (y0 - y2)^2]
# ^ assuming that the uwb ranges are squared
lineared H0 = [1  0  0  0  0  0
			   0  1  0  0  0  0
			   2  2 -2 -2  0  0
			   2  2  0  0 -2 -2]

"""

NODE_NAME = "car"
n = roshelper.Node(NODE_NAME, anonymous=False)

@n.entry_point()
class Car(object):

	def __init__(self):
		self.gps_cov = rospy.get_param("~gps_cov", 0.1)
		self.uwb_cov = rospy.get_param("~uwb_cov", 0.01)
		self.ID = rospy.get_param("~ID", 0)
		self.ranges = {}
		self.xi_prior = 0
		self.Ji_prior = 0



	@n.subscriber(UWB_TOPIC + str(self.ID), Range)
	def range_sub(self, rng):
		self.ranges[rng.header.frame_id] = rng.range

	# need to make this real NavSat
	@n.subscriber(GPS_TOPIC + str(self.ID), GPS)
	def gps_sub(self, gps):
		self.gps = gps

	@n.main_loop(frequency=50)
	def run(self):
		pass