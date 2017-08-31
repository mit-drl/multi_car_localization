#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from multi_car_msgs.msg import LidarPose
from multi_car_msgs.msg import CarState
import random
import tf
import numpy as np

# Corey's particle filter publishes a transform
# from /laser to /map

class FakeLidar(object):

    def __init__(self):
        self.rate = rospy.Rate(rospy.get_param("~frequency", 5))
        self.frame_id = rospy.get_param("~car_frame_id", "car0")
        self.Ndim = rospy.get_param("~num_state_dim", 3)
        self.ID = int(self.frame_id[-1])

        self.num_particles = 100

        self.sigma = 0.05

        self.state = None
        self.pose = PoseStamped()
        self.pose.header = Header()
        self.pose.header.frame_id = self.frame_id

        self.tf = tf.TransformBroadcaster()

        self.range_sub = rospy.Subscriber('/range_position', CarState, self.range_sub_cb, queue_size=1)
        self.pose_pub = rospy.Publisher('/lidar_pose', LidarPose, queue_size=1)
        self.poses_pub = rospy.Publisher('/lidar_poses', LidarPose, queue_size=1)
        self.viz_pub = rospy.Publisher('lidar_viz', PoseStamped, queue_size=1)

    def range_sub_cb(self, cs):
        if cs.car_id == self.ID:
            self.state = list(cs.state)
            self.state[0] += random.gauss(0.0, self.sigma)
            self.state[1] += random.gauss(0.0, self.sigma)
            self.state[2] += random.gauss(0.0, 0.05)

    def publish_range(self):
        if self.state is not None:
            particles = np.zeros((self.num_particles, self.Ndim))
            for i in range(self.num_particles):
                particles[i] = np.array([self.state[0] + random.gauss(0.0, self.sigma), 
                                         self.state[1] + random.gauss(0.0, self.sigma),
                                         self.state[2] + random.gauss(0.0, 0.05)])
            cov = np.cov(particles.T).flatten().tolist()
            ps = LidarPose()
            ps.x = self.state[0]
            ps.y = self.state[1]
            ps.theta = self.state[2]
            ps.car_id = self.ID
            ps.cov = cov
            self.pose_pub.publish(ps)
            self.poses_pub.publish(ps)

            self.pose.pose.position.x = self.state[0]
            self.pose.pose.position.y = self.state[1]
            quaternion = tf.transformations.quaternion_from_euler(0, 0, self.state[2])
            self.pose.pose.orientation.x = quaternion[0]
            self.pose.pose.orientation.y = quaternion[1]
            self.pose.pose.orientation.z = quaternion[2]
            self.pose.pose.orientation.w = quaternion[3]
            self.viz_pub.publish(self.pose)


    def run(self):
        while not rospy.is_shutdown():
            self.publish_range()
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("fake_lidar", anonymous=False)
    lidar = FakeLidar()
    lidar.run()