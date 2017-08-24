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
        self.rate = rospy.Rate(rospy.get_param("~frequency", 20))
        self.frame_id = rospy.get_param("~car_frame_id", "car0")

        self.control = CarControl()
        self.control.header = Header()
        self.control.steering_angle = 0.0
        self.control.velocity = 0.0

        self.control_pub = rospy.Publisher('/control', CarControl, queue_size=1)
        self.controls_pub = rospy.Publisher('/controls', CarControl, queue_size=1)

    def publish_range(self):
        self.control.header.frame_id = self.frame_id
        self.control.car_id = int(self.frame_id[-1])
        self.control.header.stamp = rospy.Time.now()
        self.control_pub.publish(self.control)
        self.controls_pub.publish(self.control)

    def run(self):
        while not rospy.is_shutdown():
            self.publish_range()
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("control", anonymous=False)
    fakecontrol = FakeControl()
    fakecontrol.run()