#!/usr/bin/env python

import math
import rospy
import tf
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class TFBagPublisher(object):

    def __init__(self):
        self.Ncars = 3

        self.rate = rospy.Rate(100)
        self.listener = tf.TransformListener(True, rospy.Duration(1.0))

        self.start_pose = []
        self.init_yaw = [None]*self.Ncars
        for i in range(3):
            num = str(i)
            now = rospy.Time.now()
            self.listener.waitForTransform("/world", "/vicon/ba_car" + num + "/ba_car" + num, now, rospy.Duration(3.0))
            (trans, rot) = self.listener.lookupTransform("/world", "/vicon/ba_car" + num + "/ba_car" + num, now)
            self.start_pose.append(Pose(position=trans, orientation=rot))

        self.br = tf.TransformBroadcaster()

        self.time = rospy.Time.now()

        self.clock_sub = rospy.Subscriber("/clock", Clock, self.clock_cb)

        self.imu_sub = []
        self.imu_pub = []
        self.odom_sub = []
        self.odom_pub = []
        self.first_odom = []
        self.yaw_pub = []
        for i in range(3):
            num = str(i)
            self.imu_sub.append(
                        rospy.Subscriber("/car" + num + "/imu_old", Imu, self.imu_cb, (i,), queue_size=1))
            self.imu_pub.append(rospy.Publisher("/car" + num + "/imu", Imu, queue_size=1))
            self.yaw_pub.append(rospy.Publisher("/car" + num + "/yaw", PoseStamped, queue_size=1))
            
            self.odom_sub.append(rospy.Subscriber("/car" + num + "/odom_old", Odometry, self.odom_cb, (i,), queue_size=1))
            self.odom_pub.append(rospy.Publisher("/car" + num + "/odom", Odometry, queue_size=1))
            self.first_odom.append(None)

    def odom_cb(self, odom, args):
        num = args[0]
        if self.first_odom[num] is None:
            self.first_odom[num] = odom
        odom.header.stamp = self.time
        odom.pose.pose.position.x = odom.pose.pose.position.x - self.first_odom[num].pose.pose.position.x
        odom.pose.pose.position.y = odom.pose.pose.position.y - self.first_odom[num].pose.pose.position.y
        odom.pose.pose.position.z = odom.pose.pose.position.z - self.first_odom[num].pose.pose.position.z
        self.odom_pub[num].publish(odom)

    def clock_cb(self, clock):
        self.time = clock.clock

    def imu_cb(self, imu, args):
        num = args[0]
        imu.header.stamp = self.time

        old_quat = (imu.orientation.x,
                    imu.orientation.y,
                    imu.orientation.z,
                    imu.orientation.w)
        (r, p, y) = tf.transformations.euler_from_quaternion(old_quat)

        if self.init_yaw[num] is None:
            self.init_yaw[num] = y

        yaw = y - 3.14159/2.0#self.init_yaw[num]

        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose = Pose()
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        ps = PoseStamped(header=imu.header, pose=pose)
        self.yaw_pub[num].publish(ps)

        self.imu_pub[num].publish(imu)

    def run(self):
        while not rospy.is_shutdown():
            for i in range(3):
                num = str(i)
                self.br.sendTransform((0, 0, 0),
                                      (0, 0, 0, 1),
                                      rospy.Time.now(),
                                      "/car"+num+"/odom",
                                      "world")

                # self.br.sendTransform(self.starting_position[i].position,
                #                       self.starting_position[i].orientation,
                #                       rospy.Time.now(),
                #                       "/car"+num+"/odom",
                #                       "world")

            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("tf_bag_publisher", anonymous=False)
    tfbagpub = TFBagPublisher()
    tfbagpub.run()