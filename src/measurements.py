#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped
from multi_car_msgs.msg import CarMeasurement
from multi_car_msgs.msg import UWBRange
from multi_car_msgs.msg import CarControl
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

class Measurements(object):

	def __init__(self):

		self.rate = rospy.Rate(rospy.get_param("~frequency", 60))
		self.num_uwbs = rospy.get_param("~num_uwbs", 3)
		self.Ncars = rospy.get_param("~num_cars", 3)
		self.frame_id = rospy.get_param("~car_frame_id", "car0")

		self.meas = CarMeasurement()
		self.meas.header = Header()
		self.meas.header.frame_id = self.frame_id

		self.uwb_ranges = self.init_uwb()
		self.gps = [None]*self.Ncars
		self.control = [None] * self.Ncars
		#self.gps_data = {}

		self.uwb_sub = rospy.Subscriber("ranges", UWBRange, self.range_cb, queue_size=1)
		#self.gps_sub = rospy.Subscriber("gps", NavSatFix, self.gps_cb, queue_size=1)
		self.gps_sub = []
		for i in range(self.Ncars):
			self.gps_sub.append(
				rospy.Subscriber(
				"odom" + str(i), Odometry, self.gps_cb, (i,), queue_size=1))
		#self.initial_gps = None

		self.control_sub = rospy.Subscriber("/control", CarControl, self.control_cb, queue_size=1)

		self.meas_pub = rospy.Publisher(
			"measurements", CarMeasurement, queue_size=1)

		self.id_dict = {0 : 0, 26677 : 0, 26626 : 0,
						1 : 1, 26727 : 1, 26630 : 1, 
						2 : 2, 26715 : 2, 26663 : 2}

		#self.br = tf.TransformBroadcaster()

	def init_uwb(self):
		uwbs = {}
		for j in range(self.Ncars):
			for k in range(self.Ncars):
				if k > j:
					uwbs[(j, k)] = -1
					uwbs[(k, j)] = -1
		return uwbs

	def control_cb(self, control):
		car_id = self.id_dict[control.car_id]
		control.car_id = car_id
		self.control[car_id] = control

	def range_cb(self, uwb):
		uwb.to_id = self.id_dict{uwb.to_id}
		uwb.from_id = self.id_dict{uwb.from_id}
		self.uwb_ranges[(uwb.to_id, uwb.from_id)] = uwb
		# if self.frame_id == "car0":	
		# 	print self.uwb_ranges

	def gps_cb(self, gps, args):
		car_id = args[0]
		gps.car_id = self.id_dict{gps.car_id}
		self.gps[car_id] = gps

	def publish_measurements(self):
		gps_good = None not in self.gps
		control_good = None not in self.control

		uwb_good = True
		for uwb in self.uwb_ranges:
			if self.uwb_ranges[uwb] == -1:
				uwb_good = False

		if gps_good and uwb_good and control_good:

			self.meas.header.stamp = rospy.Time.now()
			#if len(self.uwb_ranges) == self.Ncars - 1:
				#if len(self.gps_data) == (self.Ncars - 1)*self.num_uwbs:
			for ID in self.uwb_ranges:
				self.meas.range.append(self.uwb_ranges[ID])
			self.meas.gps = self.gps
			self.meas.control = self.control
			# for ID in self.gps_data:
			# 	self.meas.gps.append(self.gps_data[ID])
			self.meas_pub.publish(self.meas)
			# for pub in self.outside_pub:
			# 	pub.publish(self.meas)
			self.gps = [None]*self.Ncars
			self.uwb_ranges = self.init_uwb()
			self.control = [None]*self.Ncars

	def run(self):
		while not rospy.is_shutdown():
			self.publish_measurements()
			self.rate.sleep()

if __name__ == "__main__":
	rospy.init_node("measurements", anonymous=False)
	measurements = Measurements()
	measurements.run()