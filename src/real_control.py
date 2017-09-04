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
import dynamics

class Control(object):

    def __init__(self):
        self.rate = rospy.Rate(rospy.get_param("~frequency", 40))
        self.car_id = rospy.get_param("~car_id", 0)
        self.frame_name = rospy.get_param("/frame_name")
        self.frame_id = self.frame_name[self.car_id]
        self.dynamics_model = rospy.get_param("~dynamics_model", "dubins")
        self.dynamics = dynamics.model(self.dynamics_model)
        self.Ndim = self.dynamics.Ndim
        self.Ninputs = self.dynamics.Ninputs

        self.gain = rospy.get_param("/vesc/steering_angle_to_servo_gain")
        self.offset = rospy.get_param("/vesc/steering_angle_to_servo_offset")

        # erpm (electrical rpm) = speed_to_erpm_gain * speed (meters / second) + speed_to_erpm_offset
        self.speed_gain = rospy.get_param("/vesc/speed_to_erpm_gain")
        self.speed_offset = rospy.get_param("/vesc/speed_to_erpm_offset")

        self.control = CarControl()
        self.control.header = Header()
        self.control.header.frame_id = self.frame_id
        self.control.car_id = self.car_id

        self.prev_x = None
        self.prev_time = None

        self.vel = False
        self.steering_angle = False

        self.control_pub = rospy.Publisher('control', CarControl, queue_size=1)

        self.vel_sub = rospy.Subscriber('/vesc/sensors/core', VescStateStamped, self.vel_cb, queue_size=1)
        self.steering_sub = rospy.Subscriber('/vesc/sensors/servo_position_command', Float64, self.steering_cb, queue_size=1)

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
        # erpm (electrical rpm) = speed_to_erpm_gain * speed (meters / second) + speed_to_erpm_offset
        self.control.velocity = (core.state.speed - self.speed_offset)/float(self.speed_gain)
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