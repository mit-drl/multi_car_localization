#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped
from multi_car_localization.msg import CarMeasurement
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

class FakeCar(object):

	def __init__(self):
		self.rate = rospy.Rate(rospy.get_param("frequency", 50))
		self.gps_cov = rospy.get_param("~gps_cov", 0.1)
		self.uwb_cov = rospy.get_param("~uwb_cov", 0.01)
		self.frame_id = rospy.get_param("~frame_id", "car0")
		self.uwb_id = rospy.get_param("~uwb_id", "uwb0")

		self.x = 10*random.random()
		self.y = 10*random.random()
		self.gps = None

		self.pose_pub = rospy.Publisher("/range_position", PoseStamped, queue_size=1)
		#self.range_sub = rospy.Subscriber("/" + self.uwb_id + "/measurements", CarMeasurement, self.range_sub_cb)
		#self.gps_sub = rospy.Subscriber("gps", PoseStamped, self.gps_sub_cb)

		self.ranges = {}

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
	frame_id = rospy.get_param("~frame_id", "car0")

	rospy.init_node("fakecar", anonymous=False)
	car = FakeCar()
	car.run()