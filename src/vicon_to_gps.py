#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from multi_car_msgs.msg import GPS
import numpy as np
import tf
import copy
import math

class ViconToGPS(object):

    def __init__(self):
        self.rate = rospy.Rate(rospy.get_param("~frequency", 7))
        self.Ncars = rospy.get_param("~num_cars", 3)

        self.vicon_sub = []
        self.vicon_pub = []
        self.path = []

        self.spoof = []
        self.tru = []
        self.spoof_coords = []
        self.fake_gps_pub_coords = []

        self.var = 0.8           
        
        self.csail_coords = (42.362068, -71.09061995732195)

        for i in range(self.Ncars):
            self.vicon_pub.append(
                rospy.Publisher(
                "vicon_path" + str(i), Path, queue_size=1))
            self.path.append(Path())
            self.path[i].header = Header()

            self.fake_gps_pub_coords.append(
                rospy.Publisher("/car" + str(i) + "/fix", GPS, queue_size=1))

            self.spoof.append(PoseStamped())
            self.tru.append(PoseStamped())
            self.spoof_coords.append(GPS())
            self.spoof_coords[i].fix = NavSatFix()
            self.spoof_coords[i].header = Header()
            self.spoof_coords[i].header.frame_id = "earth"

            self.vicon_sub.append(
                rospy.Subscriber(
                "/vicon/car" + str(i) + "/car" + str(i), 
                TransformStamped, self.vicon_cb, (i,),queue_size=1))

        self.fake_gps_pub_coords.append(
            rospy.Publisher("/fix", GPS, queue_size=1))

        # self.fake_gps_pub = rospy.Publisher("spoofed_gps", PoseStamped, queue_size=1)
        # self.spoof_gps_pub_fix = rospy.Publisher("spoofed_gps_fix", GPSFix, queue_size=1)
        # self.gps_to_map_pub = rospy.Publisher("gps_to_map", PoseStamped, queue_size=1)

    def vicon_cb(self, tr, args):
        car_id = args[0]

        tr.header.frame_id = "car" + str(car_id)

        self.tru[car_id].header = tr.header
        self.tru[car_id].pose.position.x = tr.transform.translation.x
        self.tru[car_id].pose.position.y = tr.transform.translation.y
        self.path[car_id].header = tr.header

        self.spoof[car_id].header = tr.header
        self.spoof[car_id].pose.position.x = tr.transform.translation.x \
                                + np.random.normal(0, self.var)
        self.spoof[car_id].pose.position.y = tr.transform.translation.y \
                                + np.random.normal(0, self.var)


        self.spoof_coords[car_id].car_id = car_id
        # spoof map coords (meters) to lat/long
        # 111,111 meters in y direction is ~1 degree latitude
        # 111,111 * cos(latitude) meters in x direction is ~1 degree longitude
        self.spoof_coords[car_id].fix.latitude = self.csail_coords[0] \
                                + self.spoof[car_id].pose.position.y / 111111.0
        self.spoof_coords[car_id].fix.longitude = self.csail_coords[1] \
                                + self.spoof[car_id].pose.position.x / (111111.0 * math.cos(self.spoof_coords[car_id].fix.latitude*math.pi/180.0))
         
    def run(self):
        while not rospy.is_shutdown():
            for i in range(self.Ncars):
                self.tru[i].header.stamp = rospy.Time.now()
                self.spoof[i].header.stamp = rospy.Time.now()
                self.spoof_coords[i].fix.header = self.spoof_coords[i].header
                self.spoof_coords[i].header.stamp = rospy.Time.now()
                # self.spoof_fix.header.stamp = rospy.Time.now()
                # self.gps_to_map.header.stamp = rospy.Time.now()

                # self.fake_gps_pub.publish(self.spoof)
                self.fake_gps_pub_coords[i].publish(self.spoof_coords[i])
                self.fake_gps_pub_coords[-1].publish(self.spoof_coords[i])
                # self.spoof_gps_pub_fix.publish(self.spoof_fix)
                
                if len(self.path[i].poses) > 500:
                    self.path[i].poses.pop(0)

                self.path[i].header.stamp = rospy.Time.now()
                self.path[i].poses.append(copy.deepcopy(self.tru[i]))

                self.vicon_pub[i].publish(self.path[i])

            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("spoof_vicon", anonymous=False)
    spoofed = ViconToGPS()
    spoofed.run()