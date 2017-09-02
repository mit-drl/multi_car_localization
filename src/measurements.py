#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped
from multi_car_msgs.msg import CarMeasurement
from multi_car_msgs.msg import UWBRange
from multi_car_msgs.msg import CarControl
from multi_car_msgs.msg import LidarPose
from multi_car_msgs.msg import MeasurementDebug
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import tf

import dict_to_graph
import networkx as nx

class Measurements(object):

	def __init__(self):

		self.rate = rospy.Rate(rospy.get_param("~frequency", 20))
		self.Ncars = rospy.get_param("~num_cars", 3)
		self.frame_id = rospy.get_param("~car_frame_id", "car0")
		self.id_dict = rospy.get_param("/id_dict", None)
		self.connections = rospy.get_param("/connections", None)
		self.own_connections = self.connections[self.frame_id[-1]]
		self.Nconn = len(self.own_connections)

		self.full_graph = dict_to_graph.convert(self.connections)
		self.graph = dict_to_graph.prune(self.full_graph, int(self.frame_id[-1]))

		self.meas = CarMeasurement()
		self.meas.header = Header()
		self.meas.header.frame_id = self.frame_id
		self.meas.car_id = int(self.frame_id[-1])

		self.uwb_ranges = self.init_uwb()
		self.gps = [None]*self.Nconn
		self.control = [None] * self.Nconn
		self.lidar = [None] * self.Nconn
		self.first_time = True

		self.debug = MeasurementDebug()
		self.debug.header.frame_id = self.frame_id
		self.debug_pub = rospy.Publisher("meas_debug", MeasurementDebug, queue_size=1)

		#self.gps_sub = rospy.Subscriber("gps", NavSatFix, self.gps_cb, queue_size=1)
		self.gps_sub = []
		self.control_sub = []
		self.control_sub2 = []
		self.lidar_sub = []
		self.uwb_sub = rospy.Subscriber("/ranges", UWBRange, self.range_cb, queue_size=1)
		for i, ID in enumerate(self.own_connections):
			self.gps_sub.append(
				rospy.Subscriber(
				"odom" + str(ID), Odometry, self.gps_cb, (i,), queue_size=1))

			if int(self.frame_id[-1]) == ID:
				self.control_sub.append(
					rospy.Subscriber("/control", CarControl, self.control_cb, queue_size=1))
				self.lidar_sub.append(
					rospy.Subscriber("/lidar_pose", LidarPose, self.lidar_cb, queue_size=1))
			else:
				self.control_sub.append(
					rospy.Subscriber("/car" + str(ID) + "/control", CarControl, self.control_cb, queue_size=1))
				self.lidar_sub.append(
					rospy.Subscriber("/car" + str(ID) + "/lidar_pose", LidarPose, self.lidar_cb, queue_size=1))

		self.meas_pub = rospy.Publisher(
			"measurements", CarMeasurement, queue_size=1)

	def init_uwb(self):
		uwbs = {}
		for j in self.own_connections:
			for k in self.own_connections:
				if (j, k) in self.graph.edges():
					null_uwb = UWBRange()
					null_uwb.distance = -1
					null_uwb.to_id = j
					null_uwb.from_id = k
					uwbs[(j, k)] = null_uwb
		return uwbs

	def lidar_cb(self, lp):
		car_id = self.id_dict[str(lp.car_id)]
		if car_id in self.own_connections:
			lp.car_id = car_id
			self.lidar[self.own_connections.index(car_id)] = lp

	def control_cb(self, control):
		car_id = self.id_dict[str(control.car_id)]
		if car_id in self.own_connections:
			control.car_id = car_id
			self.control[self.own_connections.index(car_id)] = control

	def range_cb(self, uwb):
		uwb.to_id = self.id_dict[str(uwb.to_id)]
		uwb.from_id = self.id_dict[str(uwb.from_id)]
		if (uwb.to_id, uwb.from_id) in self.graph.edges():
			self.uwb_ranges[(uwb.to_id, uwb.from_id)] = uwb

	def gps_cb(self, gps, args):
		num = args[0]
		self.gps[num] = gps

	def publish_measurements(self):
		control_good = None not in self.control
		# gps_good = None not in self.gps
		# lidar_good = None not in self.lidar

		uwb_good = True
		for i in self.own_connections:
			for j in self.own_connections:
				if i < j and (i, j) in self.graph.edges():
					if self.uwb_ranges[(i, j)].distance == -1 and self.uwb_ranges[(j, i)].distance == -1:
						uwb_good = False

		# to initialize particle you need gps readings
		# from every car
		gps_good = False
		for gps in self.gps:
			if gps is not None:
				gps_good = True
 
 		lidar_good = False
		if self.first_time:
			lidar_good = None not in self.lidar
		else:
			for lidar in self.lidar:
				if lidar is not None:
					lidar_good = True

		num_gps = 0
		for gps in self.gps:
			if gps is not None:
				num_gps += 1
		num_control = 0
		for cont in self.control:
			if cont is not None:
				num_control += 1
		num_lidar = 0
		for lidar in self.lidar:
			if lidar is not None:
				num_lidar += 1
		num_uwb = 0
		for uwb in self.uwb_ranges:
			if self.uwb_ranges[uwb].distance != -1:
				num_uwb += 1
		self.debug.num_uwb = num_uwb
		self.debug.num_lidar = num_lidar
		self.debug.num_gps = num_gps
		self.debug.num_control = num_control
		self.debug.success = False

		if gps_good and uwb_good and control_good and lidar_good:
			if self.first_time:
				self.first_time = False
			self.debug.success = True

			self.meas.header.stamp = rospy.Time.now()

			for ID in self.uwb_ranges:
				self.meas.range.append(self.uwb_ranges[ID])

			self.meas.gps = []
			self.meas.lidar = []

			for gps in self.gps:
				if gps is None:
					blank_gps = Odometry()
					blank_gps.header.frame_id = "None"
					self.meas.gps.append(blank_gps)
				else:
					self.meas.gps.append(gps)
			for lidar in self.lidar:
				if lidar is None:
					blank_lidar = LidarPose()
					blank_lidar.header.frame_id = "None"
					self.meas.lidar.append(blank_lidar)
				else:
					self.meas.lidar.append(lidar)

			self.meas.control = self.control

			self.meas_pub.publish(self.meas)

			self.meas.range = []
			self.gps = [None]*self.Nconn
			self.uwb_ranges = self.init_uwb()
			self.control = [None]*self.Nconn
			self.lidar = [None]*self.Nconn

		self.debug.header.stamp = rospy.Time.now()
		self.debug_pub.publish(self.debug)

	def run(self):
		while not rospy.is_shutdown():
			self.publish_measurements()
			self.rate.sleep()

if __name__ == "__main__":
	rospy.init_node("measurements", anonymous=False)
	measurements = Measurements()
	measurements.run()
