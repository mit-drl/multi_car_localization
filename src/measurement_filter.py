#!/usr/bin/env python

import math
import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TransformStamped
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
import utils

class ParticleFilter(object):

    def __init__(self):
        self.rate = rospy.Rate(rospy.get_param("~frequency", 10))
        self.Np = rospy.get_param("~num_particles", 150)
        self.Ncars = rospy.get_param("/num_cars", 3)
        self.fake_sensors = rospy.get_param("~fake_sensors", True)
        self.dynamics_model = rospy.get_param("~dynamics_model", "dubins")
        self.dynamics = dynamics.model(self.dynamics_model)
        self.Ndim = self.dynamics.Ndim
        self.Ninputs = self.dynamics.Ninputs

        self.car_id = rospy.get_param("~car_id", 0)
        self.frame_name = rospy.get_param("/frame_name")
        self.frame_id = self.frame_name[self.car_id]

        self.connections = rospy.get_param("/connections", None)
        self.own_connections = self.connections[str(self.car_id)]
        self.Nconn = len(self.own_connections)
        self.car_index = self.own_connections.index(self.car_id)

        self.full_graph = dict_to_graph.convert(self.connections)
        self.graph = dict_to_graph.prune(self.full_graph, self.car_id)

        self.listener = tf.TransformListener()

        self.x0 = None
        self.lidar = [None]*self.Nconn
        self.uwbs = {}
        self.init_angle = rospy.get_param("/init_angle", [0.0, 1.0, 2.5, -0.5])

        self.lidar_cov = rospy.get_param("/lidar_cov", [0.15, 0.15, 0.05])
        self.uwb_cov   = rospy.get_param("/uwb_cov", 0.1)
        self.num_uwb_meas = len(self.graph.neighbors(self.car_id)) + 1
        self.Nmeas = len(self.lidar_cov) + self.num_uwb_meas

        self.init_cov = np.diag(self.Nconn * rospy.get_param("/init_cov", [0.1, 0.1, 0.01]))
        self.x_cov = np.diag(rospy.get_param("/x_cov", [0.05, 0.05, 0.03]))
        
        cov_diags = list(self.lidar_cov)
        for i in range(self.Nmeas - len(self.lidar_cov)):
            cov_diags.append(self.uwb_cov)
        self.meas_cov = np.diag(self.Nconn * cov_diags)

        self.resample_perc = rospy.get_param("~resample_perc", 0.3)

        self.prev_meas = None
        self.prev_time = rospy.get_time()
        self.current_time = 0

        self.u = np.zeros((self.Nconn, self.Ninputs))
        self.xs_pred = np.zeros((self.Nconn, self.Ndim))

        self.filter = None

        self.trans = None
        self.new_meas = False

        self.pa_max = 100
        self.pa_pub = rospy.Publisher("particles", PoseArray, queue_size=1)
        self.pos_pub = rospy.Publisher("positions", PoseArray, queue_size=1)

        self.state_pub = rospy.Publisher("states", CarState, queue_size=1)
        self.combined_pub = rospy.Publisher("combined", CombinedState, queue_size=1)

        self.filter_path_pub = []
        for i in self.own_connections:
            self.filter_path_pub.append(
                rospy.Publisher("path" + str(i) + str(i), Path, queue_size=1))

        self.meas_sub = rospy.Subscriber("measurements", CarMeasurement, self.meas_cb,
           queue_size=1)

    def meas_cb(self, meas):
        if not self.new_meas:
            self.u = np.zeros((self.Nconn, self.Ninputs))
            for i, control in enumerate(meas.control):
                self.u[i, 0] = meas.control[i].steering_angle
                self.u[i, 1] = meas.control[i].velocity

            self.uwbs = {}
            for uwb in meas.range:
                self.uwbs[(uwb.to_id, uwb.from_id)] = uwb

            self.lidar = meas.lidar

            if self.x0 is None:
                self.x0 = np.zeros((self.Nconn, self.Ndim))
                for i, j in enumerate(self.own_connections):
                    self.x0[i, 0] = self.lidar[i].x
                    self.x0[i, 1] = self.lidar[i].y
                    self.x0[i, 2] = self.lidar[i].theta

                self.xs_pred = self.x0

                self.filter = pf.MultiCarParticleFilter(
                    num_particles=self.Np,
                    num_cars=self.Nconn,
                    car_id=self.car_id,
                    connections=self.own_connections,
                    x0=self.x0,
                    x_cov=self.x_cov,
                    uwb0=self.uwbs,
                    pose_cov=np.diag(self.lidar_cov),
                    uwb_var=self.uwb_cov,
                )

            self.new_meas = True

    def run(self):
        filter_paths = []
        infs = np.zeros((self.Nconn, self.Ndim, self.Ndim))
        for j in range(self.Nconn):
            filter_path = Path()
            filter_path.header = Header()
            filter_path.header.stamp = rospy.Time(0)
            filter_path.header.frame_id = "%smap" % rospy.get_namespace()
            filter_paths.append(filter_path)

        while not rospy.is_shutdown():

            if self.x0 is None or self.filter is None or not self.new_meas:
                start_time = rospy.get_time()
            else:
                self.current_time = rospy.get_time()
                dt = self.current_time - self.prev_time
                self.prev_time = self.current_time
                if dt > 1.0:
                   print "%s: PF DT BIG: %f" % (self.frame_id, dt)

                us = self.u

                new_meas_cov = self.meas_cov
                pose_meas = []

                meas = np.zeros((self.Nconn, self.Nmeas))
                for j in xrange(self.Nconn):
                    meas[j, :3] = [self.lidar[j].x, self.lidar[j].y, self.lidar[j].theta]
                    if self.lidar[j].header.frame_id == 'None':
                        pose_meas.append(None)
                    else:
                        pose_meas.append(np.array([self.lidar[j].x, self.lidar[j].y, self.lidar[j].theta]))

                    cov_dim = 3
                    if self.lidar[j].header.frame_id == "None":
                        meas[j, :3] = [0.0, 0.0, 0.0]
                        new_meas_cov[j*self.Nmeas:j*self.Nmeas+3, j*self.Nmeas:j*self.Nmeas+3] = \
                                2345.0*np.diag([1.0, 1.0, 1.0])
                    elif self.lidar[j].car_id == 0:
                        new_meas_cov[j*self.Nmeas:j*self.Nmeas+3, j*self.Nmeas:j*self.Nmeas+3] = \
                                5000.0*np.diag([1.0, 1.0, 1.0])
                    else:
                        new_meas_cov[j*self.Nmeas:j*self.Nmeas+3, j*self.Nmeas:j*self.Nmeas+3] = \
                                np.diag([0.2, 0.2, 0.05])
                                # 500*np.array(self.lidar[j].cov).reshape((cov_dim, cov_dim))
                    #     print np.array(self.lidar[j].cov).reshape((cov_dim, cov_dim))
                    """
                    Measurements:
                        [lid_x0, lid_y0, lid_theta0, uwb_00, uwb_01, uwb_02
                         lid_x1, lid_y1, lid_theta1, uwb_10, uwb_11, uwb_12
                         lid_x2, lid_y2, lid_theta2, uwb_20, uwb_21, uwb_22]
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
                            meas[j, k + 3] = self.uwbs[(to_id, from_id)].distance
                            if self.uwbs[(to_id, from_id)].distance == -1:
                                self.uwbs[(to_id, from_id)].distance = self.uwbs[(from_id, to_id)].distance
                                new_meas_cov[j*self.Nmeas + k + 3, j*self.Nmeas + k + 3] = 2345.0
                        elif to_id == from_id:
                            new_meas_cov[j*self.Nmeas + k + 3, j*self.Nmeas + k + 3] = 0.001
                        elif to_id != from_id:
                            new_meas_cov[j*self.Nmeas + k + 3, j*self.Nmeas + k + 3] = 2345.0

                # unset semaphore
                self.new_meas = False

                # about 0.1-0.2 seconds
                # much shorter now that i reduced the rate of
                # the callbacks - i think they were interrupting
                # this function and causing it to take longer
                st0 = rospy.get_time()
                # self.filter.set_meas_cov(new_meas_cov)
                particles = self.filter.update_particles(pose_meas)
                tim0 = rospy.get_time() - st0
                # print "FWD SIMULATE:     %f" % (tim0)

                # negligible time
                for j in range(self.Nconn):
                    infs[j] = np.linalg.inv(np.cov(particles[:, j, :], rowvar=False) + 1e-4*np.eye(3))

                frames = PoseArray()
                frames.header = Header()
                frames.header.stamp = rospy.Time.now()
                frames.header.frame_id = "%smap" % rospy.get_namespace()
                positions = PoseArray()
                positions.header = Header()
                positions.header.stamp = rospy.Time.now()
                positions.header.frame_id = "%smap" % rospy.get_namespace()
                for p in particles[:self.pa_max]:
                    for j in range(self.Nconn):
                        frames.poses.append(utils.make_pose(p[j]))
                        if pose_meas[j] is not None:
                            pose = utils.transform(pose_meas[j], p[j])
                            positions.poses.append(utils.make_pose(pose))
                self.pa_pub.publish(frames)
                self.pos_pub.publish(positions)

                # these two are usually 0.05 seconds
                # could take up to 0.2 seconds though
                st1 = rospy.get_time()
                self.filter.update_weights(pose_meas, self.uwbs)
                self.xs_pred, covs = self.filter.predicted_state()
                # print self.car_id, np.stack((np.linalg.det(covs[...,:2,:2]), covs[...,2,2]), axis=-1)
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

                # self.prev_time = self.current_time

                self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("filter", anonymous=False)
    particlefilter = ParticleFilter()
    particlefilter.run()
