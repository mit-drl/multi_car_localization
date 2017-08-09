#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from gps_common.msg import GPSFix
import numpy as np
import tf
import copy
import math

class ViconToGPS(object):

    def __init__(self):
        self.rate = rospy.Rate(rospy.get_param("~frequency", 3))
        self.frame_id = rospy.get_param("~frame_id", "car0")

        self.vicon_sub = rospy.Subscriber(
            "/vicon/" + self.frame_id + "/" + self.frame_id, 
            TransformStamped, self.vicon_cb, queue_size=1)
        self.vicon_pub = rospy.Publisher("vicon_path", Path, queue_size=1)
        self.fake_gps_pub = rospy.Publisher("spoofed_gps", PoseStamped, queue_size=1)
        self.fake_gps_pub_coords = rospy.Publisher("spoofed_gps_coords", NavSatFix, queue_size=1)
        self.spoof_gps_fix = rospy.Publisher("spoofed_gps_fix", GPSFix, queue_size=1)
        
        self.var = 0.8
        self.path = Path()
        self.path.header = Header()
        
        self.spoof = PoseStamped()
        self.spoof_fix = GPSFix()
        self.tru = PoseStamped()
        self.spoof_coords = NavSatFix()
        self.csail_coords = (42.361826, -71.090607)
      

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

        # 111,111 meters in y direction is ~1 degree latitude
        # 111,111 * cos(latitude) meters in x direction is ~1 degree longitude
        self.spoof_coords.latitude = self.csail_coords[0] \
                                + self.spoof.pose.position.x / 111111
        self.spoof_coords.longitude = self.csail_coords[1] \
                                + self.spoof.pose.position.y / (111111 * math.cos(self.spoof_coords.latitude))
        self.spoof_fix.latitude = self.spoof_coords.latitude
        self.spoof_fix.longitude = self.spoof_coords.longitude
         
    def run(self):
        while not rospy.is_shutdown():
            if self.spoof is not None:
                self.path.header.stamp = rospy.Time.now()
                self.tru.header.stamp = rospy.Time.now()
                self.spoof.header.stamp = rospy.Time.now()
                self.spoof_coords.header.stamp = rospy.Time.now()
                self.spoof_fix.header.stamp = rospy.Time.now()
                self.fake_gps_pub.publish(self.spoof)
                self.fake_gps_pub_coords.publish(self.spoof_coords)
                self.path.poses.append(copy.deepcopy(self.tru))

                if len(self.path.poses) > 60:
                    self.path.poses.pop(0)

                self.vicon_pub.publish(self.path)

                self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("spoof_vicon", anonymous=False)
    spoofed = ViconToGPS()
    spoofed.run()