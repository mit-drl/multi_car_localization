#!/usr/bin/env python

import math
import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from multi_car_msgs.msg import CarMeasurement
from multi_car_msgs.msg import CarState
from multi_car_msgs.msg import UWBRange
import random
import copy

class FixesToGPS(object):

    def __init__(self):
        self.rate = rospy.Rate(rospy.get_param("~frequency", 20))
        self.frame_id = rospy.get_param("~frame_id", "car0")
        self.ID = int(self.frame_id[-1])

        self.range_pub = rospy.Publisher('uwb', UWBRange, queue_size=1)
        self.range_sub = rospy.Subscriber('fix', GPS, self.gps_cb)

    def range_sub_cb(self, cs):
        frame_id = cs.header.frame_id
        ID = int(frame_id[-1])

        if self.position == None and ID == self.ID:
            self.position = (cs.state[0], cs.state[1])
        elif self.position is not None and ID != self.ID:
            x = cs.state[0]
            y = cs.state[1]
            dist = math.sqrt((self.position[0] - x)**2 + (self.position[1] - y)**2)
            self.ranges[ID] = copy.deepcopy(self.rng)
            self.ranges[ID].distance = dist + random.gauss(0.0, self.sigma)
            self.ranges[ID].from_id = int(frame_id[-1])        

    def publish_range(self):
        if len(self.ranges) == 0:
            return
        for ID in self.ranges:
            self.ranges[ID].header.stamp = rospy.Time.now()
            self.range_pub.publish(self.ranges[ID])

    def run(self):
        while not rospy.is_shutdown():
            self.publish_range()
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("uwb", anonymous=False)
    uwb = FakeUWB()
    uwb.run()