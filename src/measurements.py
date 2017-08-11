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

		self.rate = rospy.Rate(rospy.get_param("~frequency", 15))
		self.num_uwbs = rospy.get_param("~num_uwbs", 3)
		self.Ncars = rospy.get_param("~num_cars", 3)
		self.frame_id = rospy.get_param("~car_frame_id", "car0")

		self.meas = CarMeasurement()
		self.meas.header = Header()
		self.meas.header.frame_id = self.frame_id

		self.uwb_ranges = {}
		self.gps = [None]*self.Ncars
		self.control = CarControl()
		#self.gps_data = {}

		self.uwb_sub = rospy.Subscriber("uwb", UWBRange, self.range_cb, queue_size=1)
		#self.gps_sub = rospy.Subscriber("gps", NavSatFix, self.gps_cb, queue_size=1)
		self.gps_sub = []
		for i in range(self.Ncars):
			self.gps_sub.append(
				rospy.Subscriber(
				"odom" + str(i), Odometry, self.gps_cb, (i,), queue_size=1))

		self.control_sub = rospy.Subscriber("control", CarControl, self.control_cb, queue_size=1)
		# self.meas_sub = rospy.Subscriber(
		# 	"outside_measurements", CarMeasurement, self.meas_cb, queue_size=1)

		self.meas_pub = rospy.Publisher(
			"measurements", CarMeasurement, queue_size=1)

		self.outside_pub = list()
		for i in range(self.Ncars):
			if i != int(self.frame_id[-1]):
				self.outside_pub.append(
					rospy.Publisher("/car" + str(i) + "/outside_measurements",
						CarMeasurement, queue_size=1))

	def control_cb(self, control):
		self.control = control

	def range_cb(self, uwb):
		receiver = uwb.to_id
		transmitter = uwb.from_id
		self.uwb_ranges[(transmitter, receiver)] = uwb

	def gps_cb(self, gps, args):
		car_id = args[0]
		self.gps[car_id] = gps
		#self.gps_data[gps.header.frame_id] = gps

	# def meas_cb(self, meas):
	# 	for uwb in meas.range:
	# 		receiver = uwb.to_id
	# 		transmitter = uwb.from_id
	# 		self.uwb_ranges[(transmitter, receiver)] = uwb
	# 	self.gps_data[meas.header.frame_id] = meas.gps
		#for gps in meas.gps:
		#	ID = gps.header.frame_id
		#	self.gps_data[ID] = gps


	def publish_measurements(self):
		wait_for_gps = False
		for gps in self.gps:
			if gps == None:
				wait_for_gps = True

		if len(self.uwb_ranges) > 1 and not wait_for_gps:
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

	def run(self):
		while not rospy.is_shutdown():
			self.publish_measurements()
			self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("measurements", anonymous=False)
    measurements = Measurements()
    measurements.run()