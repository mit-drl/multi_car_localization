#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Header
from multi_car_msgs.msg import LidarPose
from multi_car_msgs.msg import CarControl
from vesc_msgs.msg import VescStateStamped
from std_msgs.msg import Float64

import random
import tf
import numpy as np

class Control(object):

    def __init__(self):
        self.rate = rospy.Rate(rospy.get_param("~frequency", 40))
        self.frame_id = rospy.get_param("~car_frame_id", "car0")
        self.Ndim = rospy.get_param("~num_state_dim", 3)
        self.ID = int(self.frame_id[-1])

        self.gain = rospy.get_param("/vesc/steering_angle_to_servo_gain")
        self.offset = rospy.get_param("/vesc/steering_angle_to_servo_offset")

        self.control = CarControl()
        self.control.header = Header()
        self.control.header.frame_id = self.frame_id
        self.control.car_id = int(self.frame_id[-1])

        self.prev_x = None
        self.prev_time = None

        self.vel = False
        self.steering_angle = False

        self.vel_sub = rospy.Subscriber('/vesc/sensors/core', VescStateStamped, self.vel_cb, queue_size=1)
        self.steering_sub = rospy.Subscriber('/vesc/sensors/servo_position_command', Float64, self.steering_cb, queue_size=1)
        self.control_pub = rospy.Publisher('control', CarControl, queue_size=1)

    def steering_cb(self, msg):
        self.control.steering_angle = (msg.data - self.offset)/self.gain
        self.steering_angle = True

    def vel_cb(self, core):
        #if self.prev_x is not None and self.prev_time is not None:
            # x = core.distance_traveled
            # t = core.header.stamp

            # dt = (t - self.prev_time).to_secs()
            # dx = (x - self.prev_x)/100.0
            # vel = dx/dt
        self.control.velocity = core.state.speed
        self.vel = True

        #self.prev_x = x
        #self.prev_time = core.header.stamp

    def publish_range(self):
        if self.vel and self.steering_angle:
            self.control.header.stamp = rospy.Time.now()
            self.control_pub.publish(self.control)
            self.steering_angle = False
            self.vel = False

    def run(self):
        while not rospy.is_shutdown():
            self.publish_range()
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("control_node", anonymous=False)
    control = Control()
    control.run()