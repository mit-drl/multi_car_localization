#!/usr/bin/env python

import math
import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from multi_car_msgs.msg import CarMeasurement
from multi_car_msgs.msg import CarState
from multi_car_msgs.msg import CombinedState
from nav_msgs.msg import Path
from std_msgs.msg import Header
import tf
from tf.transformations import quaternion_from_euler


import particle_filter as pf
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal
from scipy.stats import rv_discrete
from scipy.linalg import block_diag

import dict_to_graph
import networkx as nx
import dynamics

class ParticleFilter(object):

    def __init__(self):
        self.Np = rospy.get_param("~num_particles", 150)
        self.Ncars = rospy.get_param("~num_cars", 3)
        self.dynamics_model = rospy.get_param("~dynamics_model", "roomba")
        self.dynamics = dynamics.model(self.dynamics_model)
        self.Ndim = self.dynamics.Ndim
        self.Ninputs = self.dynamics.Ninputs

        # self.Ndim = rospy.get_param("~num_state_dim", 3)
        # self.Ninputs = rospy.get_param("~num_inputs", 2)
        self.frame_id = rospy.get_param("~car_frame_id", "car0")

        self.connections = rospy.get_param("/connections", None)
        self.own_connections = self.connections[self.frame_id[-1]]
        self.Nconn = len(self.own_connections)

        self.full_graph = dict_to_graph.convert(self.connections)
        self.graph = dict_to_graph.prune(self.full_graph, int(self.frame_id[-1]))

        self.Nmeas = 2 + 3 + len(self.graph.neighbors(int(self.frame_id[-1]))) + 1

        self.listener = tf.TransformListener()

        self.x0 = None
        self.gps = [None]*self.Nconn
        self.lidar = [None]*self.Nconn
        self.uwbs = {}
        self.init_angle = [-0.1, 1.0, 2.5, -0.5]

        self.init_cov = np.diag(self.Nconn * [1.0, 1.0, 0.01])
        self.x_cov = np.diag(self.Nconn * [0.1, 0.1, 0.03])
        # self.meas_cov = np.diag(self.Ncars * [0.6, 0.6, 0.1, 0.1, 0.1])
        cov_diags = [0.6, 0.6, 0.05, 0.05, 0.05]
        for i in range(self.Nmeas - 5):
            cov_diags.append(0.05)
        self.meas_cov = 2.5*np.diag(self.Nconn * cov_diags)

        self.resample_perc = rospy.get_param("~resample_perc", 0.3)

        self.prev_meas = None
        self.prev_time = rospy.get_time()
        self.current_time = 0

        self.u = np.zeros((self.Nconn, self.Ninputs))
        #self.xs = np.zeros((self.Ncars, self.Ndim))
        self.xs_pred = np.zeros((self.Nconn, self.Ndim))

        self.filter = None

        self.error = np.zeros((1,))

        self.rate = rospy.Rate(rospy.get_param("~frequency", 10))

        self.meas_sub = rospy.Subscriber("measurements", CarMeasurement, self.meas_cb,
           queue_size=1)

        # self.pos_sub = rospy.Subscriber("/range_position", CarState, self.pos_cb,
        #     queue_size=1)
        self.pa_pub = rospy.Publisher("particles", PoseArray, queue_size=1)

        self.state_pub = rospy.Publisher("states", CarState, queue_size=1)
        self.combined_pub = rospy.Publisher("combined", CombinedState, queue_size=1)

        #self.true_pos = {}

        self.trans = None
        self.new_meas = False

        try:
            self.listener.waitForTransform("/utm", "/map", rospy.Time(), rospy.Duration(4.0))
            (self.trans,rot) = self.listener.lookupTransform('/utm', '/map', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "TRANSFORM FAILEDDDDDDDDDDDDDDDDDDD"
            pass

        self.filter_path_pub = []
        for i in self.own_connections:
            self.filter_path_pub.append(
                rospy.Publisher("path" + str(i) + str(i), Path, queue_size=1))

    def meas_cb(self, meas):
        self.gps = meas.gps

        self.u = np.zeros((self.Nconn, self.Ninputs))
        for i, control in enumerate(meas.control):
            self.u[i, 0] = meas.control[i].steering_angle
            self.u[i, 1] = meas.control[i].velocity

        self.uwbs = {}
        for uwb in meas.range:
            self.uwbs[(uwb.to_id, uwb.from_id)] = uwb

        self.lidar = meas.lidar

        if self.x0 == None:
            self.x0 = np.zeros((self.Nconn, self.Ndim))
            # try using lidar instead of gps?
            for i, j in enumerate(self.own_connections):
                self.x0[i, 0] = self.lidar[i].x
                self.x0[i, 1] = self.lidar[i].y
                self.x0[i, 2] = self.lidar[i].theta

                # self.x0[i, 0] = self.gps[i].pose.pose.position.x - self.trans[0]
                # self.x0[i, 1] = self.gps[i].pose.pose.position.y - self.trans[1]
                # self.x0[i, 2] = self.init_angle[j]

            self.xs_pred = self.x0

            self.filter = pf.MultiCarParticleFilter(
                num_particles=self.Np,
                num_cars=self.Nconn,
                num_measurements=self.Nmeas,
                x0=self.x0,
                init_cov=self.init_cov,
                x_cov=self.x_cov,
                measurement_cov=self.meas_cov,
                resample_perc=self.resample_perc,
                dynamics_model=self.dynamics_model)

        self.new_meas = True

    def run(self):
        filter_paths = []
        infs = np.zeros((self.Nconn, self.Ndim, self.Ndim))
        for j in range(self.Nconn):
            filter_path = Path()
            filter_path.header = Header()
            filter_path.header.stamp = rospy.Time(0)
            filter_path.header.frame_id = "map"
            filter_paths.append(filter_path)

        while not rospy.is_shutdown():

            if self.x0 == None or self.filter == None or not self.new_meas:
                start_time = rospy.get_time()
            else:
                self.new_meas = False
                self.current_time = rospy.get_time()
                dt = self.current_time - self.prev_time
                self.prev_time = self.current_time
                if dt > 1.0:
                   print "%s: PF DT BIG: %f" % (self.frame_id, dt)

                us = self.u

                new_meas_cov = self.meas_cov

                meas = np.zeros((self.Nconn, self.Nmeas))
                for j in xrange(self.Nconn):
                    meas[j, 0] = self.gps[j].pose.pose.position.x - self.trans[0]
                    meas[j, 1] = self.gps[j].pose.pose.position.y - self.trans[1]
                    meas[j, 2:5] = [self.lidar[j].x, self.lidar[j].y, self.lidar[j].theta]

                    if self.gps[j].header.frame_id == "None":
                        new_meas_cov[j*self.Nmeas, j*self.Nmeas] = 2345.0
                        new_meas_cov[j*self.Nmeas + 1, j*self.Nmeas + 1] = 2345.0

                    cov_dim = 3
                    if self.lidar[j].header.frame_id == "None":
                        new_meas_cov[j*self.Nmeas + 2:j*self.Nmeas + 5, j*self.Nmeas + 2:j*self.Nmeas + 5] = \
                                2345.0*np.diag([1.0, 1.0, 1.0])
                    else:
                        new_meas_cov[j*self.Nmeas + 2:j*self.Nmeas + 5, j*self.Nmeas + 2:j*self.Nmeas + 5] = \
                                10.0*np.array(self.lidar[j].cov).reshape((cov_dim, cov_dim))
                    """
                    Measurements:
                        [gps_x0, gps_y0, lid_x0, lid_y0, lid_theta0, uwb_00, uwb_01, uwb_02
                         gps_x1, gps_y1, lid_x1, lid_y1, lid_theta1, uwb_10, uwb_11, uwb_12
                         gps_x2, gps_y2, lid_x2, lid_y2, lid_theta2, uwb_20, uwb_21, uwb_22]
                    """
                    # j k
                    # 0 1
                    # 0 2
                    # 1 0
                    # 1 2
                    # 2 0
                    # 2 1
                    for k in xrange(self.Nconn):
                        to_id = self.own_connections[j]
                        from_id = self.own_connections[k]
                        if (to_id, from_id) in self.graph.edges():
                            meas[j, k + 5] = self.uwbs[(to_id, from_id)].distance
                            if self.uwbs[(to_id, from_id)].distance == -1:
                                self.uwbs[(to_id, from_id)].distance = self.uwbs[(from_id, to_id)].distance
                                new_meas_cov[j*self.Nmeas + k + 5, j*self.Nmeas + k + 5] = 12345.0
                        elif to_id == from_id:
                            new_meas_cov[j*self.Nmeas + k + 5, j*self.Nmeas + k + 5] = 0.001
                        elif to_id != from_id:
                            new_meas_cov[j*self.Nmeas + k + 5, j*self.Nmeas + k + 5] = 12345.0

                # about 0.1-0.2 seconds
                # much shorter now that i reduced the rate of
                # the callbacks - i think they were interrupting
                # this function and causing it to take longer
                st0 = rospy.get_time()
                self.filter.set_meas_cov(new_meas_cov)
                particles = self.filter.update_particles(us, dt)
                tim0 = rospy.get_time() - st0
                # print "FWD SIMULATE:     %f" % (tim0)

                # negligible time
                for j in range(self.Nconn):
                    infs[j] = np.linalg.inv(np.cov(particles[:, j, :].T))

                pa = PoseArray()
                pa.header = Header()
                pa.header.stamp = rospy.Time.now()
                pa.header.frame_id = "map"
                for p in particles:
                    for j in range(self.Nconn):
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

                # these two are usually 0.05 seconds
                # could take up to 0.2 seconds though
                st1 = rospy.get_time()
                self.filter.update_weights(meas)
                self.xs_pred = self.filter.predicted_state()
                tim1 = rospy.get_time() - st1
                # print "update weights:   %f" % (tim1)

                for j in range(self.Nconn):

                    if j == 0:
                        pose2 = PoseStamped()
                        pose2.header = Header()
                        pose2.header.stamp = rospy.Time(0)
                        pose2.header.frame_id = "car" + str(j)
                        pose2.pose.position.x = self.xs_pred[j, 0]
                        pose2.pose.position.y = self.xs_pred[j, 1]
                        pose2.pose.orientation.w = 1
                        filter_paths[j].poses.append(pose2)
                        if len(filter_paths[j].poses) > 60:
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
                    #self.state_pub.publish(state)


                combined = CombinedState()
                combined.u = us.flatten().tolist()
                combined.state = self.xs_pred.flatten().tolist()
                combined.header = state.header
                combined.inf = block_diag(*infs).flatten().tolist()
                self.combined_pub.publish(combined)

                # can take up to 0.1 seconds
                # usually around 0.05 seconds
                st2 = rospy.get_time()
                self.filter.resample()
                tim2 = rospy.get_time() - st2
                # print "RESAMPLE     :    %f" % (tim2)

                #self.error = np.append(self.error, np.zeros((1,)))
                #for j in xrange(self.Ncars):
                #    self.error[i] += np.linalg.norm(self.xs_pred[j, :2] - self.xs[j, :2]) / self.Ncars

                # self.prev_time = self.current_time

                #self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("filter", anonymous=False)
    particlefilter = ParticleFilter()
    particlefilter.run()
