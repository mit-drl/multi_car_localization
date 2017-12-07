#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from multi_car_msgs.msg import UWBRange
from multi_car_msgs.msg import CanopyCollector
from vesc_msgs.msg import VescStateStamped
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage

class CanopyListener(object):

    def __init__(self):
        self.rate = rospy.Rate(rospy.get_param("~frequency", 40))

        self.canopy_msg = CanopyCollector()

        self.imu_pub       = rospy.Publisher("imu", Imu, queue_size=1)
        self.odom_pub      = rospy.Publisher("odom", Odometry, queue_size=1)
        self.range_pub     = rospy.Publisher("ranges", UWBRange, queue_size=1)
        self.core_pub      = rospy.Publisher("vesc/sensors/core", VescStateStamped, queue_size=1)
        self.servo_pub     = rospy.Publisher("vesc/sensors/servo_position_command", Float64, queue_size=1)
        self.lidar_pub     = rospy.Publisher("slam_out_pose", PoseStamped, queue_size=1)
        self.tf_pub        = rospy.Publisher("tf", TFMessage, queue_size=1)
        self.tf_static_pub = rospy.Publisher("tf_static", TFMessage, queue_size=1)

        self.canopy_sub    = rospy.Subscriber("canopy_msg", CanopyCollector)

    def canopy_sub(self, data):
        self.imu_pub.publish(data.imu)
        self.odom_pub.publish(data.odom)
        self.range_pub.publish(data.ranges)
        self.core_pub.publish(data.core)
        self.servo_pub.publish(data.servo)
        self.lidar_pub.publish(data.slam_out_pose)
        self.tf_pub.publish(data.tf)
        self.tf_static.publish(data.tf_static)

if __name__ == "__main__":
    rospy.init_node("canopy_listener", anonymous=False)
    canopy_listener = CanopyListener()
    rospy.spin()