#!/usr/bin/env python

import rospy
import tf
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from multi_car_msgs.msg import UWBRange
from multi_car_msgs.msg import CanopyCollector
from vesc_msgs.msg import VescStateStamped
from multi_car_msgs.msg import CarControl
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
import math

class MeasViz(object):

    def __init__(self):
        self.rate = rospy.Rate(50)
        self.Ncars = 3
        self.counter = 0
        self.paths = []

        self.marker_pubs = []
        self.path_pubs = []
        self.range_subs = []
        self.control_subs = []
        self.control_pubs = []
        for i in range(self.Ncars):
            path = Path()
            path.header.frame_id = "/world"
            self.paths.append(path)
            self.path_pubs.append(
                rospy.Publisher("car" + str(i+1) + "/path", Path, queue_size=1))
            self.range_subs.append(
                rospy.Subscriber("car" + str(i+1) + "/ranges", UWBRange, self.range_cb, (i+1,)))
            self.marker_pubs.append(
                rospy.Publisher("car" + str(i+1) + "/range_marker", Marker, queue_size=1))
            self.control_subs.append(
                rospy.Subscriber("car" + str(i+1) + "/control", CarControl, self.control_cb))
            self.control_pubs.append(
                rospy.Publisher("car" + str(i+1) + "/control_viz", PoseStamped, queue_size=1))

        self.listener = tf.TransformListener()


    def control_cb(self, data):
        cp = PoseStamped()
        cp.header.stamp = rospy.Time(0)
        cp.header.frame_id = "/vicon/ba_car" + str(data.car_id) + "/ba_car" + str(data.car_id)
        cp.pose.position.x = data.velocity
        self.control_pubs[data.car_id-1].publish(cp)


    def range_cb(self, data, args):
        from_id = data.from_id
        to_id = data.to_id
        # try:
        spheres = Marker()
        spheres.header.stamp = rospy.Time(0)
        spheres.header.frame_id = "/vicon/ba_car" + str(from_id) + "/ba_car" + str(from_id)
        spheres.id = self.counter
        self.counter = self.counter + 1
        spheres.type = 4 #linestrip
        spheres.action = 0 #add
        spheres.pose = Pose()
        spheres.scale.x = 0.01;
        spheres.scale.y = 0.05;
        spheres.scale.z = 0.05;
        spheres.color.a = 1.0;
        spheres.lifetime = rospy.Duration(1)
        if to_id == 1:
            spheres.color.r = 0.0;
            spheres.color.g = 1.0;
            spheres.color.b = 0.0;
        elif to_id == 2:
            spheres.color.r = 1.0;
            spheres.color.g = 0.0;
            spheres.color.b = 0.0;
        elif to_id == 3:
            spheres.color.r = 0.0;
            spheres.color.g = 0.0;
            spheres.color.b = 1.0;

        r = data.distance
        num_spheres = 80
        angle_per = 2*math.pi/num_spheres
        a = range(num_spheres)
        a.append(0)
        for i in a:
            angle = angle_per*i
            dx = math.cos(angle)*r
            dy = math.sin(angle)*r
            sx = dx
            sy = dy
            p = Point()
            p.x = sx
            p.y = sy
            spheres.points.append(p)
        self.marker_pubs[to_id-1].publish(spheres)

    def run(self):
        while not rospy.is_shutdown():
            for i in range(self.Ncars):
                try:
                    pos, quat = self.listener.lookupTransform("/world", "/vicon/ba_car" + str(i+1) + "/ba_car" + str(i+1),rospy.Time(0))
                    self.paths[i].header.stamp = rospy.Time(0)
                    new_pose = PoseStamped()
                    new_pose.header.stamp = rospy.Time(0)
                    new_pose.header.frame_id = "/world"
                    new_pose.pose.position.x = pos[0]
                    new_pose.pose.position.y = pos[1]
                    self.paths[i].poses.append(new_pose)
                    if len(self.paths[i].poses) > 100:
                        self.paths[i].poses.pop(0)
                    self.path_pubs[i].publish(self.paths[i])
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("meas_viz", anonymous=False)
    measviz = MeasViz()
    measviz.run()
    # rospy.spin()