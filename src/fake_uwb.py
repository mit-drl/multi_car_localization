#!/usr/bin/env python

import math
import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from multi_car_localization.msg import CarMeasurement
from multi_car_localization.msg import UWBMsg
import random
import copy

class FakeUWB(object):

    def __init__(self):
        self.rate = rospy.Rate(rospy.get_param("~frequency", 50))
        self.frame_id = rospy.get_param("~frame_id", "car0")
        self.ID = int(self.frame_id[-1])

        self.ranges = {}
        self.rng = Range()
        self.rng.header = Header()
        self.rng.field_of_view = math.pi * 0.1
        self.rng.min_range = 0
        self.rng.max_range = 300

        self.sigma = 0.01
        self.num_cars = 3
        self.uwbs_per_car = 1

        self.position = None

        self.range_pub = rospy.Publisher('uwb', Range, queue_size=1)
        self.range_sub = rospy.Subscriber('/range_position', PoseStamped, self.range_sub_cb)

    def range_sub_cb(self, ps):
        frame_id = ps.header.frame_id
        ID = int(frame_id[-1])

        if self.position == None and ID == self.ID:
            self.position = (ps.pose.position.x, ps.pose.position.y)
        elif self.position is not None and ID != self.ID:
            x = ps.pose.position.x
            y = ps.pose.position.y
            dist = math.sqrt((self.position[0] - x)**2 + (self.position[1] - y)**2)
            self.ranges[ID] = copy.deepcopy(self.rng)
            self.ranges[ID].range = dist + random.gauss(0.0, self.sigma)
            self.ranges[ID].header.frame_id = frame_id        

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