#!/usr/bin/env python

import math
import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from multi_car_localization.msg import CarMeasurement
from multi_car_localization.msg import CarState
from nav_msgs.msg import Path
from std_msgs.msg import Header
import tf
from tf.transformations import quaternion_from_euler


import particle_filter as pf
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal
from scipy.stats import rv_discrete

class ParticleFilter(object):

    def __init__(self):
        self.Np = rospy.get_param("~num_particles", 100)
        self.Ncars = rospy.get_param("~num_cars", 3)
        self.Ndim = rospy.get_param("~num_state_dim", 3)
        self.Nmeas = rospy.get_param("~num_measurements", 5)
        self.Ninputs = rospy.get_param("~num_inputs", 2)
        self.frame_id = rospy.get_param("~car_frame_id", "car0")

        self.x0 = None

        self.init_cov = np.diag(self.Ncars * [1.0, 1.0, 0.0])
        self.x_cov = np.diag(self.Ncars * [0.1, 0.1, 0.1])
        self.meas_cov = np.diag(self.Ncars * [0.6, 0.6, 0.15, 0.15, 0.15])
        
        # self.init_cov = np.diag(self.Ncars * [1.0, 1.0, 0.05, 0.00])
        # self.x_cov = np.diag(self.Ncars * [0.05, 0.05, 0.05, 0.01])
        # self.meas_cov = np.diag(self.Ncars * [0.6, 0.6, 0.15, 0.15, 0.15])
        #self.control_cov = np.diag(self.Ncars * self.Ndim * [0.1])
        self.resample_perc = rospy.get_param("~resample_perc", 0.3)

        self.prev_time = rospy.get_time()
        self.current_time = 0

        self.u = np.zeros((self.Ncars, self.Ninputs))
        self.xs = np.zeros((1, self.Ncars, self.Ndim))
        self.xs_pred = np.zeros_like(self.xs)

        self.filter = None

        self.error = np.zeros((1,))

        self.rate = rospy.Rate(rospy.get_param("~frequency", 10))

        self.path_pub0 = rospy.Publisher("path0", Path, queue_size=1)
        self.path_pub00 = rospy.Publisher("path00", Path, queue_size=1)
        self.path_pub1 = rospy.Publisher("path1", Path, queue_size=1)
        self.path_pub11 = rospy.Publisher("path11", Path, queue_size=1)
        self.path_pub2 = rospy.Publisher("path2", Path, queue_size=1)
        self.path_pub22 = rospy.Publisher("path22", Path, queue_size=1)

        self.meas_sub = rospy.Subscriber("measurements", CarMeasurement, self.meas_cb)

        self.pos_sub = rospy.Subscriber("/range_position", CarState, self.pos_cb)
        self.pa_pub = rospy.Publisher("particles", PoseArray, queue_size=1)

        self.gps = {}
        self.uwbs = {}
        self.true_pos = {}

    def pos_cb(self, cs):
        self.true_pos[cs.header.frame_id] = cs.state
        self.u[int(cs.header.frame_id[-1])] = cs.u
        
        if self.x0 == None and len(self.true_pos) == 3:
            self.x0 = np.zeros((self.Ncars, self.Ndim))
            for ID in self.true_pos:
                p = self.true_pos[ID]
                idx = int(ID[-1])
                self.x0[idx] = np.array(p, dtype=np.float64)

            self.xs[0] = self.x0
            self.xs_pred[0] = self.x0

            self.filter = pf.MultiCarParticleFilter(
                num_particles=self.Np,
                num_cars=self.Ncars,
                num_state_dim=self.Ndim,
                num_measurements=self.Nmeas,
                x0=self.x0,
                init_cov=self.init_cov,
                x_cov=self.x_cov,
                measurement_cov=self.meas_cov,
                resample_perc=self.resample_perc)

    def meas_cb(self, meas):
        for gps in meas.gps:
            self.gps[gps.header.frame_id] = gps
        for uwb in meas.ranges:
            receiver = meas.header.frame_id
            transmitter = uwb.header.frame_id
            self.uwbs[(transmitter, receiver)] = uwb

    def run(self):
        path0 = Path()
        path0.header = Header()
        path0.header.stamp = rospy.Time(0)
        path0.header.frame_id = "map"
        path00 = Path()
        path00.header = Header()
        path00.header.stamp = rospy.Time(0)
        path00.header.frame_id = "map"
        path1 = Path()
        path1.header = Header()
        path1.header.stamp = rospy.Time(0)
        path1.header.frame_id = "map"
        path11 = Path()
        path11.header = Header()
        path11.header.stamp = rospy.Time(0)
        path11.header.frame_id = "map"
        path2 = Path()
        path2.header = Header()
        path2.header.stamp = rospy.Time(0)
        path2.header.frame_id = "map"
        path22 = Path()
        path22.header = Header()
        path22.header.stamp = rospy.Time(0)
        path22.header.frame_id = "map"

        while not rospy.is_shutdown():
            if self.x0 == None or self.filter == None:
                start_time = rospy.get_time()
                i = 1
            else:
                #rospy.loginfo(self.frame_id)
                self.current_time = rospy.get_time()
                dt = self.current_time - self.prev_time
                print "%s %f" % (self.frame_id, dt)
                
                us = self.u#self.u_func(self.current_time - start_time)
                self.xs = np.append(self.xs, [np.zeros((self.Ncars,self.Ndim))], axis=0)
                for j in xrange(self.Ncars):
                    self.xs[i, j] = self.true_pos["car" + str(j)]
                    #print "%s %d fakeeeeeeee yaw: %f" % (self.frame_id, j, self.xs[i, j, 2]*180.0/math.pi)
                    #self.filter.state_transition(self.xs[i - 1, j], us[j], dt)


                pose0 = PoseStamped()
                pose0.header = Header()
                pose0.header.stamp = rospy.Time(0)
                pose0.header.frame_id = "car0"
                pose0.pose.position.x = self.xs[i, 0, 0]
                pose0.pose.position.y = self.xs[i, 0, 1]
                pose0.pose.orientation.w = 1
                path0.poses.append(pose0)
                pose1 = PoseStamped()
                pose1.header = Header()
                pose1.header.stamp = rospy.Time(0)
                pose1.header.frame_id = "car1"
                pose1.pose.position.x = self.xs[i, 1, 0]
                pose1.pose.position.y = self.xs[i, 1, 1]
                pose1.pose.orientation.w = 1
                path1.poses.append(pose1)
                pose2 = PoseStamped()
                pose2.header = Header()
                pose2.header.stamp = rospy.Time(0)
                pose2.header.frame_id = "car2"
                pose2.pose.position.x = self.xs[i, 2, 0]
                pose2.pose.position.y = self.xs[i, 2, 1]
                pose2.pose.orientation.w = 1
                path2.poses.append(pose2)

                means = np.zeros((self.Ncars, self.Nmeas))
                for j in xrange(self.Ncars):
                    means[j, :2] = self.xs[i, j, :2]
                    #means[j, 5] = self.xs[i, j, 3]
                    for k in xrange(self.Ncars):
                        if j != k:
                            means[j, k + 2] = np.linalg.norm(self.xs[i, j, :2] - self.xs[i, k, :2])
                meas = np.random.multivariate_normal(
                    means.flatten(), self.meas_cov).reshape(self.Ncars, self.Nmeas)
                particles = self.filter.update_particles(us, dt)
                pa = PoseArray()
                pa.header = Header()
                pa.header.stamp = rospy.Time.now()
                pa.header.frame_id = "map"
                for p in particles:
                    for j in range(self.Ncars):
                        pose = Pose()
                        quat = quaternion_from_euler(0, 0, p[j, 2])
                        pose.position.x = p[j, 0]
                        pose.position.y = p[j, 1]
                        pose.orientation.x = quat[0]
                        pose.orientation.y = quat[1]
                        pose.orientation.z = quat[2]
                        pose.orientation.w = quat[3]
                        pa.poses.append(pose)
                self.pa_pub.publish(pa)
                self.filter.update_weights(meas)
                self.xs_pred = np.append(self.xs_pred, [self.filter.predicted_state()], axis=0)

                pose00 = PoseStamped()
                pose00.header = Header()
                pose00.header.stamp = rospy.Time(0)
                pose00.header.frame_id = "car0"
                pose00.pose.position.x = self.xs_pred[i, 0, 0]
                pose00.pose.position.y = self.xs_pred[i, 0, 1]
                pose00.pose.orientation.w = 1
                path00.poses.append(pose00)
                pose11 = PoseStamped()
                pose11.header = Header()
                pose11.header.stamp = rospy.Time(0)
                pose11.header.frame_id = "car1"
                pose11.pose.position.x = self.xs_pred[i, 1, 0]
                pose11.pose.position.y = self.xs_pred[i, 1, 1]
                pose11.pose.orientation.w = 1
                path11.poses.append(pose11)
                pose22 = PoseStamped()
                pose22.header = Header()
                pose22.header.stamp = rospy.Time(0)
                pose22.header.frame_id = "car2"
                pose22.pose.position.x = self.xs_pred[i, 2, 0]
                pose22.pose.position.y = self.xs_pred[i, 2, 1]
                pose22.pose.orientation.w = 1
                path22.poses.append(pose22)

                if len(path0.poses) > 30:
                    path0.poses.pop(0)
                if len(path00.poses) > 30:
                    path00.poses.pop(0)
                if len(path1.poses) > 30:
                    path1.poses.pop(0)
                if len(path11.poses) > 30:
                    path11.poses.pop(0)
                if len(path2.poses) > 30:
                    path2.poses.pop(0)
                if len(path22.poses) > 30:
                    path22.poses.pop(0)

                self.filter.resample()
                self.error = np.append(self.error, np.zeros((1,)))
                for j in xrange(self.Ncars):
                    self.error[i] += np.linalg.norm(self.xs_pred[i, j, :2] - self.xs[i, j, :2]) / self.Ncars

                self.prev_time = self.current_time
                i = i + 1

                self.path_pub0.publish(path0)
                self.path_pub00.publish(path00)
                self.path_pub1.publish(path1)
                self.path_pub11.publish(path11)
                self.path_pub2.publish(path2)
                self.path_pub22.publish(path22)

                self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("filter", anonymous=False)
    particlefilter = ParticleFilter()
    particlefilter.run()