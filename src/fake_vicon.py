#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped
from multi_car_msgs.msg import CarMeasurement
from multi_car_msgs.msg import CarState
from std_msgs.msg import Header
import math
import numpy as np
import random
import tf
from dynamics import RoombaDynamics
from geometry_msgs.msg import TransformStamped

class FakeVicon(object):

    def __init__(self):
        self.rate = rospy.Rate(rospy.get_param("~frequency", 50))
        self.Ncars = rospy.get_param("~num_cars", 3)

        self.transform = TransformStamped()
        self.transform.header = Header()
        self.transform.header.stamp = rospy.Time.now()

        self.pose_pub = []
        for i in range(self.Ncars):
            self.pose_pub.append(rospy.Publisher("/vicon/car" + str(i) + 
                        "/car" + str(i), TransformStamped, queue_size=1))

        self.pose_sub = rospy.Subscriber("/range_position", CarState,
                        self.pose_cb, queue_size=1)

    def pose_cb(self, cs):
        self.transform.header.stamp = rospy.Time.now()
        self.transform.header.frame_id = cs.header.frame_id
        self.transform.transform.translation.x = cs.state[0]
        self.transform.transform.translation.y = cs.state[1]
        quaternion = tf.transformations.quaternion_from_euler(0, 0, cs.state[2])
        self.transform.transform.rotation.x = quaternion[0]
        self.transform.transform.rotation.y = quaternion[1]
        self.transform.transform.rotation.z = quaternion[2]
        self.transform.transform.rotation.w = quaternion[3]

    def run(self):
        while not rospy.is_shutdown():
            for i in range(self.Ncars):
                self.pose_pub[i].publish(self.transform)
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("fakevicon", anonymous=False)
    vicon = FakeVicon()
    vicon.run()