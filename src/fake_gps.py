#!/usr/bin/env python

import math
import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from multi_car_msgs.msg import CarMeasurement
from multi_car_msgs.msg import CarState
from sensor_msgs.msg import NavSatFix
import random
import copy

class FakeGPS(object):

    def __init__(self):
        self.rate = rospy.Rate(rospy.get_param("~frequency", 10))
        self.frame_id = rospy.get_param("~frame_id", "car0")
        self.ID = int(self.frame_id[-1])

        self.sigma = 0.6

        self.ps = NavSatFix()
        self.ps.header = Header()
        self.ps.header.frame_id = self.frame_id

        self.gps_pub = rospy.Publisher('gps', NavSatFix, queue_size=1)
        self.range_sub = rospy.Subscriber('/range_position', CarState, self.range_sub_cb)

    def range_sub_cb(self, cs):
        frame_id = cs.header.frame_id
        ID = int(frame_id[-1])

        if ID == self.ID:
            self.ps.header.stamp = rospy.Time.now()
            self.ps.latitude = cs.state[0] + random.gauss(0.0, self.sigma)
            self.ps.longitude = cs.state[1] + random.gauss(0.0, self.sigma)

    def publish_range(self):
        if self.ps.latitude == 0:
            return
        self.gps_pub.publish(self.ps)

    def run(self):
        while not rospy.is_shutdown():
            self.publish_range()
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("gps", anonymous=False)
    gps = FakeGPS()
    gps.run()