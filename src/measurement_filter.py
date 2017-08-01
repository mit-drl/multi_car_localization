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
        
        self.resample_perc = rospy.get_param("~resample_perc", 0.3)

        self.prev_time = rospy.get_time()
        self.current_time = 0

        self.u = np.zeros((self.Ncars, self.Ninputs))
        self.xs = np.zeros((self.Ncars, self.Ndim))
        self.xs_pred = np.zeros_like(self.xs)
        self.xs_pred_prev = np.zeros_like(self.xs)

        self.filter = None

        self.error = np.zeros((1,))

        self.rate = rospy.Rate(rospy.get_param("~frequency", 10))

        self.meas_sub = rospy.Subscriber("measurements", CarMeasurement, self.meas_cb)

        self.pos_sub = rospy.Subscriber("/range_position", CarState, self.pos_cb)
        self.pa_pub = rospy.Publisher("particles", PoseArray, queue_size=1)

        self.state_pub = rospy.Publisher("states", CarState, queue_size=1)

        self.gps = {}
        self.uwbs = {}
        self.true_pos = {}

        self.true_path_pub = []
        self.filter_path_pub = []
        for i in range(self.Ncars):
            self.true_path_pub.append(
                rospy.Publisher("path" + str(i), Path, queue_size=1))
            self.filter_path_pub.append(
                rospy.Publisher("path" + str(i) + str(i), Path, queue_size=1))



    def pos_cb(self, cs):
        self.true_pos[cs.header.frame_id] = cs.state
        self.u[int(cs.header.frame_id[-1])] = cs.u
        
        if self.x0 == None and len(self.true_pos) == 3:
            self.x0 = np.zeros((self.Ncars, self.Ndim))
            for ID in self.true_pos:
                p = self.true_pos[ID]
                idx = int(ID[-1])
                self.x0[idx] = np.array(p, dtype=np.float64)

            self.xs = self.x0
            self.xs_pred = self.x0

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
        true_paths = []
        filter_paths = []
        infs = [None] * self.Ncars
        for j in range(self.Ncars):
            true_path = Path()
            true_path.header = Header()
            true_path.header.stamp = rospy.Time(0)
            true_path.header.frame_id = "map"
            true_paths.append(true_path)

            filter_path = Path()
            filter_path.header = Header()
            filter_path.header.stamp = rospy.Time(0)
            filter_path.header.frame_id = "map"
            filter_paths.append(filter_path)

        while not rospy.is_shutdown():
            if self.x0 == None or self.filter == None:
                start_time = rospy.get_time()
            else:
                self.current_time = rospy.get_time()
                dt = self.current_time - self.prev_time
                # print "%s %f" % (self.frame_id, dt)
                
                us = self.u
                
                for j in xrange(self.Ncars):
                    self.xs[j] = self.true_pos["car" + str(j)]

                means = np.zeros((self.Ncars, self.Nmeas))
                for j in xrange(self.Ncars):
                    means[j, :2] = self.xs[j, :2]
                    #means[j, :2] = self.xs[i, j, :2]
                    #means[j, 5] = self.xs[i, j, 3]
                    for k in xrange(self.Ncars):
                        if j != k:
                            means[j, k + 2] = np.linalg.norm(self.xs[j, :2] - self.xs[k, :2])
                meas = np.random.multivariate_normal(
                    means.flatten(), self.meas_cov).reshape(self.Ncars, self.Nmeas)
                particles = self.filter.update_particles(us, dt)

                for j in range(self.Ncars):
                    infs[j] = np.linalg.inv(np.cov(particles[:, j, :].T))

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
                self.xs_pred = self.filter.predicted_state()

                for j in range(self.Ncars):

                    pose = PoseStamped()
                    pose.header = Header()
                    pose.header.stamp = rospy.Time(0)
                    pose.header.frame_id = "car" + str(j)
                    pose.pose.position.x = self.xs[j, 0]
                    pose.pose.position.y = self.xs[j, 1]
                    pose.pose.orientation.w = 1                    
                    true_paths[j].poses.append(pose)
                    if len(true_paths[j].poses) > 30:
                        true_paths[j].poses.pop(0)

                    self.true_path_pub[j].publish(true_paths[j])

                    pose2 = PoseStamped()
                    pose2.header = Header()
                    pose2.header.stamp = rospy.Time(0)
                    pose2.header.frame_id = "car" + str(j)
                    pose2.pose.position.x = self.xs_pred[j, 0]
                    pose2.pose.position.y = self.xs_pred[j, 1]
                    pose2.pose.orientation.w = 1
                    filter_paths[j].poses.append(pose2)
                    if len(filter_paths[j].poses) > 30:
                        filter_paths[j].poses.pop(0)

                    self.filter_path_pub[j].publish(filter_paths[j])

                    state = CarState()
                    state.u = us[j].tolist()
                    state.state = self.xs_pred[j].tolist()
                    state.header = Header()
                    state.header.frame_id = self.frame_id
                    state.header.stamp = rospy.Time.now()
                    state.car_id = j
                    state.inf = infs[j].flatten().tolist()
                    self.state_pub.publish(state)

                self.xs_pred_prev = self.xs_pred

                self.filter.resample()
                #self.error = np.append(self.error, np.zeros((1,)))
                #for j in xrange(self.Ncars):
                #    self.error[i] += np.linalg.norm(self.xs_pred[j, :2] - self.xs[j, :2]) / self.Ncars

                self.prev_time = self.current_time

                #self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("filter", anonymous=False)
    particlefilter = ParticleFilter()
    particlefilter.run()