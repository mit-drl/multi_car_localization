#!/usr/bin/env python

import rospy

from multi_car_msgs.msg import CarControl
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class LidarToControl(object):

    def __init__(self):
        self.prev_lidar = None
        self.car_id = rospy.get_param("~car_id", "car1")
        self.car_id = int(self.car_id[-1])
        # self.control_pub   = rospy.Publisher("control", CarControl, queue_size=1)
        # self.lidar_sub     = rospy.Subscriber("slam_out_pose", PoseStamped, self.lidar_cb)
        self.odom_pub      = rospy.Publisher("control", CarControl, queue_size=1)
        self.odom_sub      = rospy.Subscriber("odom", Odometry, self.odom_cb)


    def odom_cb(self, data):
        control = CarControl()
        control.header = data.header
        control.steering_angle = data.twist.twist.angular.z
        control.velocity = data.twist.twist.linear.x
        self.odom_pub.publish(control)

 #    def lidar_cb(self, data):
 #        if self.prev_lidar is not None:
 #            dt = (data.header.stamp - self.prev_lidar.header.stamp).to_sec()
 #            dx = data.pose.position.x - self.prev_lidar.pose.position.x
 #            dy = data.pose.position.y - self.prev_lidar.pose.position.y
 #            vel = (dx**2 + dy**2)**0.5 / dt
 #            if dx < 0:
 #            	vel = -1 * vel

 #            qp = self.prev_lidar.pose.orientation
 #            qpp = (qp.x, qp.y, qp.z, qp.w)
 #            qd = data.pose.orientation
 #            qdd = (qd.x, qd.y, qd.z, qd.w)
 #            tp = euler_from_quaternion(qpp)[2]
 #            td = euler_from_quaternion(qdd)[2]
 #            dtheta = (td - tp)/dt

 #            control = CarControl()
 #            control.header = data.header
 #            control.car_id = self.car_id
 #            control.steering_angle = dtheta
 #            control.velocity = vel

 #            self.control_pub.publish(control)
	# self.prev_lidar = data

if __name__ == "__main__":
    rospy.init_node("lidar_to_control", anonymous=False)
    lidartocontrol = LidarToControl()
    rospy.spin()
