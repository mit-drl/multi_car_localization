#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from multi_car_msgs.msg import CarState
from multi_car_msgs.msg import GPS
import random
import tf
import numpy as np

# Corey's particle filter publishes a transform
# from /laser to /map

class FakeGPS(object):

    def __init__(self):
        self.rate = rospy.Rate(rospy.get_param("~frequency", 15))
        self.frame_id = rospy.get_param("~car_frame_id", "car0")
        self.ID = int(self.frame_id[-1])

        self.sigma = 0.6

        self.csail_coords = np.array([42.362068, -71.09061995732195])
        self.state = self.convert_to_latlong(1.0, 1.2)

        self.latlong = self.csail_coords + self.state

        self.gps = GPS()
        self.gps.fix = NavSatFix()
        self.gps.header = Header()
        self.gps.header.frame_id = "earth"
        self.gps.car_id = self.ID

        self.pose_pub = rospy.Publisher('fix', NavSatFix, queue_size=1)

    def convert_to_latlong(self, x, y):
        return [x/111111.0, y/(111111.0*np.cos(self.csail_coords[0]))]

    def publish_range(self):
        self.state = self.convert_to_latlong(1.0 + random.gauss(0, self.sigma), 
                        1.2 + random.gauss(0, self.sigma))
        self.latlong = self.csail_coords + self.state
        

        self.gps.header.stamp = rospy.Time.now()
        self.gps.fix.latitude = self.latlong[0]
        self.gps.fix.longitude = self.latlong[1]
        self.pose_pub.publish(self.gps)


    def run(self):
        while not rospy.is_shutdown():
            self.publish_range()
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("fake_gps", anonymous=False)
    gps = FakeGPS()
    gps.run()