#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TransformStamped
from multi_car_msgs.msg import CarControl
from multi_car_msgs.msg import UWBRange
from std_msgs.msg import Header
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler


import new_particle_filter as particle_filter
from relative_dubins_dynamics import RelativeDubinsDynamics
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal
from scipy.stats import rv_discrete
from scipy.linalg import block_diag

import heapq


class NewParticleFilter(object):

    def __init__(self):
        self.rate = rospy.Rate(10)
        self.Np = rospy.get_param("~num_particles", 3)
        self.Ncars = rospy.get_param("/num_cars", 3)
        self.car_id = 1
        self.car_ids = [1, 2, 3]
        self.u_cov = np.matrix([0.1, 0.05, 0.1, 0.05, 0.1, 0.05]) # rospy.get_param("/u_cov", [0.15, 0.15, 0.05])
        self.uwb_cov = rospy.get_param("/uwb_cov", 0.05)
        self.limits = np.matrix([0.5, 0.5, np.pi / 2, 0.5, 0.5, np.pi / 2]).T
        self.bounds = np.asmatrix(np.zeros((3 * (self.Ncars - 1), 2)))
        self.circ_var = [0, 0, 1, 0, 0, 1]

        self.listener = tf.TransformListener()

        self.filter = particle_filter.ParticleFilter()
        self.initialized = False
        self.rel_model = None

        self.control_sub = []
        self.controls_heap = []
        self.car_controls = [[]] * self.Ncars
        self.update_car_control = [True] * self.Ncars
        self.max_control_times = [0]*3
        self.prev_time = rospy.Time.now().to_sec()

        self.controls = np.matrix([0] * self.Ncars * 2).T
        self.con_prev_time = [None] * self.Ncars # store prev times control received
        self.con_prev_dt = [0.1] * self.Ncars # store dt's for each car's control msgs
        self.range_sub = []
        self.pa_pub = rospy.Publisher("/car" + str(self.car_id) + "/particles",
                                      PoseArray, queue_size=1)
        self.pos_pub = rospy.Publisher("/car" + str(self.car_id) + "/estimate",
                                       PoseArray, queue_size=1)
        for i, ID in enumerate(self.car_ids):
            self.control_sub.append(
                rospy.Subscriber("/car" + str(ID) + "/control", CarControl,
                                 self.control_cb, (i,), queue_size=1))
            self.range_sub.append(
                rospy.Subscriber("/car" + str(ID) + "/ranges", UWBRange,
                                 self.range_cb, (ID, i), queue_size=1))

    def range_cb(self, data, args):
        if self.initialized:
            self.filter.correct(data.distance, data.from_id, data.to_id)

    def control_cb(self, data, args):
        if self.initialized:
            index = args[0]
            data_time = data.header.stamp.to_sec()
            heapq.heappush(self.controls_heap, (data_time, data.car_id))
            self.max_control_times[index] = data_time

            if not self.car_controls[index]:
                self.controls[2*index, 0] = data.velocity
                self.controls[2*index+1, 0] = data.steering_angle
                self.update_car_control[index] = False

            self.car_controls[index].append(data)

            while self.controls_heap[0][0] <= min(self.max_control_times):
                control_info = heapq.heappop(self.controls_heap)
                dt = control_info[0] - self.prev_time
                self.prev_time = control_info[0]
                car_idx = control_info[1] - 1
                old_control = self.car_controls[car_idx].pop(0)

                self.filter.predict(dt, self.controls)

                if self.car_controls[car_idx]:
                    self.controls[2*car_idx, 0] = \
                        self.car_controls[car_idx][0].velocity
                    self.controls[2*car_idx+1, 0] = \
                        self.car_controls[car_idx][0].steering_angle

    def set_bounds(self, transforms, limits):
        for i in range(self.Ncars - 1):
            fi = 3 * i
            self.bounds[fi:fi + 3, 0] = transforms[i, :].T - limits[fi:fi + 3]
            self.bounds[fi:fi + 3, 1] = transforms[i, :].T + limits[fi:fi + 3]

    def run(self):
        while not rospy.is_shutdown():
            if not self.initialized:
                car_order_num = 0
                car_id = str(self.car_id)
                initial_transforms = np.asmatrix(np.zeros(((self.Ncars-1)*3, 1)))
                for i in self.car_ids:
                    if i != self.car_id:
                        num = str(i)
                        now = rospy.Time(0)
                        self.listener.waitForTransform("/vicon/car" + car_id + "/car" + car_id,
                                                       "/vicon/car" + num + "/car" + num, now,
                                                       rospy.Duration(3.0))
                        (trans, rot) = self.listener.lookupTransform("/vicon/car" + car_id + "/car" + car_id,
                                                                     "/vicon/car" + num + "/car" + num, now)
                        (r, p, y) = euler_from_quaternion(rot)
                        fi = 3*car_order_num
                        initial_transforms[fi:fi+2] = [[trans[0]], [trans[1]]]
                        initial_transforms[fi+2] = y

                        car_order_num += 1
                print initial_transforms
                self.rel_model = RelativeDubinsDynamics(
                    self.Ncars, initial_transforms, self.u_cov, self.uwb_cov)
                self.set_bounds(initial_transforms, self.limits)
                self.filter.StateTransitionFcn = self.rel_model.pfStateTransition
                self.filter.MeasurementLikelihoodFcn = self.rel_model.pfMeasurementLikelihood
                self.filter.create_uniform_particles(self.Np, self.bounds, self.circ_var)
                self.initialized = True
            else:
                state = self.filter.get_state()
                states = PoseArray()
                states.header.stamp = rospy.Time.now()
                states.header.frame_id = "/vicon/car1/car1"
                for i in range(self.Ncars-1):
                    fi = 3*i
                    p = Pose()
                    p.position.x = state[fi].item()
                    p.position.y = state[fi+1].item()
                    q = quaternion_from_euler(0, 0, state[fi+2].item())
                    p.orientation.x = q[0]
                    p.orientation.y = q[1]
                    p.orientation.z = q[2]
                    p.orientation.w = q[3]
                    states.poses.append(p)
                self.pos_pub.publish(states)

                particles = self.filter.particles
                pa = PoseArray()
                pa.header.stamp = rospy.Time.now()
                pa.header.frame_id = "/vicon/car1/car1"
                for i in range(np.shape(particles)[0]):
                    particle = particles[i, :]
                    for j in range(self.Ncars-1):
                        fj = 3*j
                        p = Pose()
                        p.position.x = particle[0, fj].item()
                        p.position.y = particle[0, fj+1].item()
                        q = quaternion_from_euler(0, 0, particle[0, fj+2])
                        p.orientation.x = q[0]
                        p.orientation.y = q[1]
                        p.orientation.z = q[2]
                        p.orientation.w = q[3]
                        pa.poses.append(p)
                        print p
                self.pa_pub.publish(pa)


if __name__ == "__main__":
    rospy.init_node("new_particle_filter", anonymous=False)
    npf = NewParticleFilter()
    npf.run()
