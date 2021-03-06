#!/usr/bin/env python

import math
import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from multi_car_msgs.msg import CarMeasurement
from multi_car_msgs.msg import CarState
from multi_car_msgs.msg import CarControl
from sensor_msgs.msg import NavSatFix
import random
import copy

class FakeControl(object):

    def __init__(self):
        self.rate = rospy.Rate(rospy.get_param("~frequency", 5))
        self.car_id = rospy.get_param("~car_id", 0)
        self.frame_name = rospy.get_param("/frame_name")
        self.frame_id = self.frame_name[self.car_id]

        self.control = CarControl()
        self.control.header = Header()
        self.control.header.frame_id = self.frame_id
        self.control.car_id = self.car_id
        self.control.steering_angle = 0.0
        self.control.velocity = 1.0

        self.control_pub = rospy.Publisher('control', CarControl, queue_size=1)

    def publish_range(self):
        self.control.header.stamp = rospy.Time.now()
        self.control_pub.publish(self.control)

    def run(self):
        while not rospy.is_shutdown():
            self.publish_range()
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("control", anonymous=False)
    fakecontrol = FakeControl()
    fakecontrol.run()
