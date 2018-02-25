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
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros
from utils import get_id_to_index, get_index_to_id

class MeasViz(object):

    def __init__(self):
        self.rate = rospy.Rate(30)
        self.Ncars = int(rospy.get_param("/num_cars", 3))
        self.counter = 0
        self.paths = []
        self.estimate_paths = []
        self.car_id = int(rospy.get_param("~car_id", 1))
        self.car_ids = rospy.get_param("/car_ids", [])
        self.id_to_index = get_id_to_index(self.Ncars, self.car_ids, self.car_id)
        self.index_to_id = get_index_to_id(self.Ncars, self.car_ids, self.car_id)

        self.marker_pubs = []
        self.path_pubs = []
        self.range_subs = []
        self.control_subs = []
        self.control_pubs = []
        self.estimate_subs = []
        self.estimate_pubs = []
        for i in range(self.Ncars):
            path = Path()
            path.header.frame_id = "/world"
            estimate_path = Path()
            estimate_path.header.frame_id = "/world"
            self.estimate_paths.append(estimate_path)
            self.paths.append(path)
            ID = str(self.index_to_id[i])
            self.path_pubs.append(
                rospy.Publisher("car" + ID + "/path", Path, queue_size=1))
            self.marker_pubs.append(
                rospy.Publisher("car" + ID + "/range_marker", Marker, queue_size=1))
            self.control_pubs.append(
                rospy.Publisher("car" + ID + "/control_viz", Marker, queue_size=1))
            self.control_subs.append(
                rospy.Subscriber("car" + ID + "/control", CarControl, self.control_cb))
            self.range_subs.append(
                rospy.Subscriber("car" + ID + "/ranges", UWBRange, self.range_cb, (int(ID),)))
            if i != 0:
                self.estimate_pubs.append(
                    rospy.Publisher("car" + str(self.car_id) + "/car" + ID +
                                     "/estimate_path", Path, queue_size=1))

        self.listener = tf.TransformListener()
        self.br = tf2_ros.StaticTransformBroadcaster()

        # need to sleep to give rosbag time to start publishing vicon transforms
        rospy.sleep(1.0)
        for i in range(self.Ncars):
            ID = self.index_to_id[i]
            self.car_transforms(ID)

    def control_cb(self, data):
        index = self.id_to_index[data.car_id]
        
        vel_arr = Marker()
        vel_arr.header.stamp = rospy.Time(0)
        vel_arr.header.frame_id = "/vicon/car" + str(data.car_id) + "/car" + str(data.car_id)
        vel_arr.id = data.car_id
        vel_arr.type = 0
        vel_arr.scale.x = data.velocity
        vel_arr.scale.y = 0.1
        vel_arr.scale.z = 0.1
        vel_arr.color.a = 1.0
        vel_arr.color.r = 1.0
        vel_arr.color.g = 1.0
        vel_arr.color.b = 0.0
        self.control_pubs[index].publish(vel_arr)

        ang_arr = Marker()
        ang_arr.header.stamp = rospy.Time(0)
        ang_arr.header.frame_id = "/vicon/car" + str(data.car_id) + "/car" + str(data.car_id)
        ang_arr.id = data.car_id + 10
        ang_arr.type = 0
        ang_arr.scale.x = data.steering_angle
        quat = quaternion_from_euler(0, 0, math.pi/2)
        ang_arr.pose.orientation.x = quat[0]
        ang_arr.pose.orientation.y = quat[1]
        ang_arr.pose.orientation.z = quat[2]
        ang_arr.pose.orientation.w = quat[3]
        ang_arr.scale.y = 0.1
        ang_arr.scale.z = 0.1
        ang_arr.color.a = 1.0
        ang_arr.color.r = 1.0
        ang_arr.color.g = 0.0
        ang_arr.color.b = 0.0
        self.control_pubs[index].publish(ang_arr)


    def range_cb(self, data, args):
        from_id = data.from_id
        to_id = data.to_id
        # try:
        spheres = Marker()
        spheres.header.stamp = rospy.Time(0)
        spheres.header.frame_id = "/vicon/car" + str(from_id) + "/car" + str(from_id)
        spheres.id = self.counter
        self.counter = self.counter + 1
        spheres.type = 4 #linestrip
        spheres.action = 0 #add
        spheres.pose = Pose()
        spheres.scale.x = 0.01;
        spheres.scale.y = 0.05;
        spheres.scale.z = 0.05;
        spheres.color.a = 1.0;
        spheres.lifetime = rospy.Duration(0.7)
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
        index = self.id_to_index[to_id]
        self.marker_pubs[index].publish(spheres)

    def make_transform(self, parent, child, state):
        x, y, z = state[:3]
        if len(state) == 6:
            euler = state[3:]
            quat = quaternion_from_euler(euler[0], euler[1], euler[2])
        else:
            quat = quaternion_from_euler(0, 0, 0)
        transform = TransformStamped()
        transform.header.stamp = rospy.Time(0)
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]

        return transform

    def car_transforms(self, car_num):
        vicon_frame = "/vicon/car" + str(car_num) + "/car" + str(car_num)
        bl_frame = "/car" + str(car_num) + "/base_link"
        vicon_to_bl = self.make_transform(vicon_frame, bl_frame, [0, 0, 0])
        self.br.sendTransform(vicon_to_bl)

        c_frame = "/car" + str(car_num) + "/chassis"
        bl_to_c = self.make_transform(bl_frame, c_frame, [-0.2, 0, 0])
        self.br.sendTransform(bl_to_c)
        ci_frame = "/car" + str(car_num) + "/chassis_inertia"
        c_to_ci = self.make_transform(c_frame, ci_frame, [0, 0, 0])
        self.br.sendTransform(c_to_ci)
        lfw_frame = "/car" + str(car_num) + "/left_front_wheel"
        c_to_lfw = self.make_transform(c_frame, lfw_frame, [0.3, 0.12, 0, -3.14159/2.0, 0, 0])
        self.br.sendTransform(c_to_lfw)
        rfw_frame = "/car" + str(car_num) + "/right_front_wheel"
        c_to_rfw = self.make_transform(c_frame, rfw_frame, [0.3, -0.12, 0, -3.14159/2.0, 0, 0])
        self.br.sendTransform(c_to_rfw)
        lrw_frame = "/car" + str(car_num) + "/left_rear_wheel"
        c_to_lrw = self.make_transform(c_frame, lrw_frame, [0, 0.12, 0, -3.14159/2.0, 0, 0])
        self.br.sendTransform(c_to_lrw)
        rrw_frame = "/car" + str(car_num) + "/right_rear_wheel"
        c_to_rrw = self.make_transform(c_frame, rrw_frame, [0, -0.12, 0, -3.14159/2.0, 0, 0])
        self.br.sendTransform(c_to_rrw)

        cam_frame = "/car" + str(car_num) + "/camera_link"
        laser_frame = "/car" + str(car_num) + "/laser"
        lsh_frame = "/car" + str(car_num) + "/left_steering_hinge"
        fsh_frame = "/car" + str(car_num) + "/right_steering_hinge"
        zed_frame = "/car" + str(car_num) + "/zed_camera_link"
        zedr_frame = "/car" + str(car_num) + "/zed_camera_right_link"
        junk_frames = [cam_frame, laser_frame, lsh_frame, fsh_frame, zed_frame, zedr_frame]
        for frame in junk_frames:
            world_to_junk = self.make_transform("/world", frame, [100, 0, 0])
            self.br.sendTransform(world_to_junk)

    def run(self):
        while not rospy.is_shutdown():
            for i in range(self.Ncars):
                ID = self.index_to_id[i]
                try:
                    pos, quat = self.listener.lookupTransform("/world", "/vicon/car" + str(ID) + "/car" + str(ID),rospy.Time(0))
                    self.paths[i].header.stamp = rospy.Time(0)
                    new_pose = PoseStamped()
                    new_pose.header.stamp = rospy.Time(0)
                    new_pose.header.frame_id = "/world"
                    new_pose.pose.position.x = pos[0]
                    new_pose.pose.position.y = pos[1]
                    self.paths[i].poses.append(new_pose)
                    # if len(self.paths[i].poses) > 100:
                    #     self.paths[i].poses.pop(0)
                    self.path_pubs[i].publish(self.paths[i])
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
                if ID != self.car_id: # i = 1, 2
                    try:
                        pos, quat = self.listener.lookupTransform("/world", "/pfestimate" + str(self.car_id) + str(ID),rospy.Time(0))
                        self.estimate_paths[i-1].header.stamp = rospy.Time(0)
                        new_pose = PoseStamped()
                        new_pose.header.stamp = rospy.Time(0)
                        new_pose.header.frame_id = "/world"
                        new_pose.pose.position.x = pos[0]
                        new_pose.pose.position.y = pos[1]
                        self.estimate_paths[i-1].poses.append(new_pose)
                        # if len(self.paths[i].poses) > 100:
                        #     self.paths[i].poses.pop(0)
                        self.estimate_pubs[i-1].publish(self.estimate_paths[i-1])
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        continue
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("meas_viz", anonymous=False)
    measviz = MeasViz()
    measviz.run()
    # rospy.spin()
