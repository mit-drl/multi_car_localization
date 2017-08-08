#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
import numpy as np
import tf
import copy

class ViconToGPS(object):

	def __init__(self):
		self.rate = rospy.Rate(rospy.get_param("~frequency", 10))
		self.frame_id = rospy.get_param("~frame_id", "car0")

		self.vicon_sub = rospy.Subscriber(
			"/vicon/" + self.frame_id + "/" + self.frame_id, 
			TransformStamped, self.vicon_cb, queue_size=1)
		self.vicon_pub = rospy.Publisher("vicon_path", Path, queue_size=1)
		self.fake_gps_pub = rospy.Publisher("spoofed_gps", PoseStamped, queue_size=1)
		self.var = 0.8
		self.path = Path()
		self.path.header = Header()
		self.spoof = PoseStamped()
		self.tru = PoseStamped()


	def vicon_cb(self, tr):
		self.tru.header = tr.header
		self.tru.pose.position.x = tr.transform.translation.x
		self.tru.pose.position.y = tr.transform.translation.y
		self.path.header = tr.header

		self.spoof.header = tr.header
		self.spoof.pose.position.x = tr.transform.translation.x \
								+ np.random.normal(0, self.var)
		self.spoof.pose.position.y = tr.transform.translation.y \
								+ np.random.normal(0, self.var)

	def run(self):
		while not rospy.is_shutdown():
			if self.spoof is not None:
				self.path.header.stamp = rospy.Time.now()
				self.tru.header.stamp = rospy.Time.now()
				self.spoof.header.stamp = rospy.Time.now()
				self.fake_gps_pub.publish(self.spoof)
				self.path.poses.append(copy.deepcopy(self.tru))

				if len(self.path.poses) > 60:
					self.path.poses.pop(0)

				self.vicon_pub.publish(self.path)

				self.rate.sleep()

if __name__ == "__main__":
	rospy.init_node("spoof_vicon", anonymous=False)
	spoofed = ViconToGPS()
	spoofed.run()