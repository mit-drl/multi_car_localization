#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TransformStamped
from multi_car_msgs.msg import CarControl
from multi_car_msgs.msg import UWBRange
from std_msgs.msg import Header
import tf
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler


import new_particle_filter as particle_filter
from relative_dubins_dynamics import RelativeDubinsDynamics
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal
from scipy.stats import rv_discrete
from scipy.linalg import block_diag
from utils import get_id_to_index, get_index_to_id

import heapq


class NewParticleFilter(object):

    def __init__(self):
        self.rate = rospy.Rate(30)
        self.Np = rospy.get_param("~num_particles", 2000)
        self.Ncars = int(rospy.get_param("/num_cars", 3))
        self.lag = float(rospy.get_param("~lag", 0.1))
        self.car_id = int(rospy.get_param("~car_id", 1))
        self.car_ids = rospy.get_param("/car_ids", [])
        self.u_cov = [0.6, 0.6] * self.Ncars  # [1.5, 1.5] * self.Ncars # rospy.get_param("/u_cov", [0.15, 0.15, 0.05])
        self.uwb_cov = float(rospy.get_param("~uwb_cov", 0.3))
        self.limits = np.array([0.1, 0.1, np.pi / 6.0, 0.1, 0.1, np.pi / 6.0]).T
        self.bounds = np.zeros((3 * (self.Ncars - 1), 2))
        self.circ_var = [0, 0, 1] * (self.Ncars - 1)
        self.id_to_index = get_id_to_index(self.Ncars, self.car_ids, self.car_id)
        self.index_to_id = get_index_to_id(self.Ncars, self.car_ids, self.car_id)

        # print self.car_id, self.car_ids, self.id_to_index, self.index_to_id

        self.listener = tf.TransformListener()
        self.br = tf2_ros.TransformBroadcaster()

        self.filter = particle_filter.ParticleFilter()
        self.initialized = False
        self.rel_model = None

        self.control_sub = []
        self.controls_heap = []
        self.car_controls = [[] for i in range(self.Ncars)]
        self.max_control_times = [0.0]*self.Ncars
        self.prev_time = None

        self.controls = [0.0] * self.Ncars * 2
        self.last_time = 0.0
        self.recent_time = 0.0

        self.lag_time = None

        self.range_sub = []
        self.pa_pub = rospy.Publisher("/car" + str(self.car_id) + "/particles",
                                      PoseArray, queue_size=1)
        self.pos_pub = []
        for i in range(self.Ncars):  # i, ID in enumerate(self.car_ids):
            ID = self.index_to_id[i]
            self.control_sub.append(
                rospy.Subscriber("/car" + str(ID) + "/control", CarControl,
                                 self.control_cb, (i,), queue_size=1))
            self.range_sub.append(
                rospy.Subscriber("/car" + str(ID) + "/ranges", UWBRange,
                                 self.range_cb, (ID, i), queue_size=1))
            if ID != self.car_id:
                self.pos_pub.append(rospy.Publisher(
                    "/car" + str(self.car_id) + "/car" + str(ID) + "/estimate",
                    PoseStamped, queue_size=1))

    def range_cb(self, data, args):
        if self.initialized:
            self.filter.correct(data.distance, self.id_to_index[data.from_id],
                                self.id_to_index[data.to_id])  # data.from_id-1, data.to_id-1)

    def control_cb(self, data, args):
        if self.initialized:
            # print rospy.get_time() - data.header.stamp.to_sec()
            index = args[0]
            # make controls into a WEIGHTED AVERAGE
            self.controls[2*index] = data.velocity # + 0.5*self.controls[2*index]
            self.controls[2*index+1] = -0.05 + data.steering_angle  # + 0.5*self.controls[2*index+1]
            if data.header.stamp.to_sec() > self.recent_time: 
                # rospy.loginfo("Update recent time DT = {}, CarId = {}".format(
                #     data.header.stamp.to_sec() - self.recent_time,
                #     index+1))
                self.recent_time = data.header.stamp.to_sec()  # rospy.get_time()

    def control_cb_old(self, data, args):
        if self.initialized:
            index = args[0]
            data_time = data.header.stamp.to_sec()
            heapq.heappush(self.controls_heap, (data_time, data.car_id))

            self.max_control_times[index] = data_time

            if len(self.car_controls[index]) == 0:
                self.controls[2*index] = data.velocity
                self.controls[2*index+1] = data.steering_angle

            self.car_controls[index].append(data)
            self.lag_time = rospy.get_time()

            # print self.controls_heap[0][0], min(self.max_control_times)

            while self.controls_heap[0][0] <= min(self.max_control_times):
                control_info = heapq.heappop(self.controls_heap)
                if self.prev_time is None:
                    dt = 0.1
                else:
                    dt = control_info[0] - self.prev_time
                self.prev_time = control_info[0]
                car_idx = control_info[1] - 1
                old_control = self.car_controls[car_idx].pop(0)

                self.filter.predict(dt, np.asarray(self.controls))
                # print self.controls

                if self.car_controls[car_idx]:
                    self.controls[2*car_idx] = \
                        self.car_controls[car_idx][0].velocity
                    self.controls[2*car_idx+1] = \
                        self.car_controls[car_idx][0].steering_angle

    def set_bounds(self, transforms, limits):
        for i in range(self.Ncars - 1):
            fi = 3 * i
            self.bounds[fi:fi + 3, 0] = transforms[i, :].T - limits[fi:fi + 3]
            self.bounds[fi:fi + 3, 1] = transforms[i, :].T + limits[fi:fi + 3]

    def run(self):
        ticker = 1
        dt = 0
        while not rospy.is_shutdown():
            if not self.initialized:
                rospy.sleep(1.0)
                car_order_num = 0
                car_id = str(self.car_id)
                initial_transforms = np.zeros(((self.Ncars-1), 3))
                for index in range(self.Ncars):
                    i = self.index_to_id[index]
                    if i != self.car_id:
                        num = str(i)
                        now = rospy.Time(0)
                        starting_trans = "/vicon/car" + car_id + "/car" + car_id
                        ending_trans = "/vicon/car" + num + "/car" + num
                        self.listener.waitForTransform(starting_trans,
                                                       ending_trans, now,
                                                       rospy.Duration(3.0))
                        (trans, rot) = self.listener.lookupTransform(starting_trans,
                                                                     ending_trans, now)
                        (r, p, y) = euler_from_quaternion(rot)
                        fi = 3*car_order_num
                        initial_transforms[car_order_num, 0:2] = trans[0:2]
                        initial_transforms[car_order_num, 2] = y

                        car_order_num += 1
                print initial_transforms
                self.rel_model = RelativeDubinsDynamics(
                    self.Ncars, initial_transforms, self.u_cov, self.uwb_cov)
                self.set_bounds(initial_transforms, self.limits)
                self.filter.StateTransitionFcn = self.rel_model.pfStateTransition
                self.filter.MeasurementLikelihoodFcn = self.rel_model.pfMeasurementLikelihood
                self.filter.LagCompensationFcn = self.rel_model.pfLagCompensation
                self.filter.create_uniform_particles(self.Np, self.bounds, self.circ_var)
                self.initialized = True
            else:
                ticker += 1
                if self.lag_time is None:
                    self.lag_time = rospy.get_time()
                if self.last_time <= 0:
                    self.last_time = self.recent_time
                else:
                    if ticker % 2 == 0:
                        dt = self.recent_time - self.last_time
                        if dt > 0.0:
                            # print dt
                            self.last_time = self.recent_time
                            self.lag_time = rospy.get_time()
                            self.filter.predict(dt, np.asarray(self.controls))
                        else:
                            rospy.logwarn("DT < 0: DT = {}".format(dt))

                        particles = self.filter.particles
                        pa = PoseArray()
                        pa.header.stamp = rospy.Time(0)
                        pa.header.frame_id = "/vicon/car" + str(self.car_id) + "/car" + str(self.car_id)
                        for i in range(0, np.shape(particles)[0], 10):
                            particle = particles[i, :]
                            for j in range(self.Ncars-1):
                                fj = 3*j
                                p = Pose()
                                p.position.x = particle[fj]
                                p.position.y = particle[fj+1]
                                q = quaternion_from_euler(0, 0, particle[fj+2])
                                p.orientation.x = q[0]
                                p.orientation.y = q[1]
                                p.orientation.z = q[2]
                                p.orientation.w = q[3]
                                pa.poses.append(p)
                        self.pa_pub.publish(pa)
                    # if True:
                    state = self.filter.get_state()
                    lag = rospy.get_time() - self.lag_time + dt + self.lag
                    print lag
                    state = self.filter.lag_compensate(state[:,None], np.asarray(self.controls), lag)
                    for i in range(self.Ncars-1):
                        fi = 3*i
                        p = PoseStamped()
                        p.header.stamp = rospy.Time.now()
                        p.header.frame_id = "/vicon/car" + str(self.car_id) + "/car" + str(self.car_id)
                        p.pose.position.x = state[fi]
                        p.pose.position.y = state[fi+1]
                        q = quaternion_from_euler(0, 0, state[fi+2])
                        p.pose.orientation.x = q[0]
                        p.pose.orientation.y = q[1]
                        p.pose.orientation.z = q[2]
                        p.pose.orientation.w = q[3]
                        self.pos_pub[i].publish(p)

                        t = TransformStamped()
                        t.header.stamp = rospy.Time.now()
                        t.header.frame_id = "/vicon/car" + str(self.car_id) + "/car" + str(self.car_id)
                        ID = self.index_to_id[i+1]
                        t.child_frame_id = "pfestimate" + str(self.car_id) + str(ID)
                        t.transform.translation.x = p.pose.position.x
                        t.transform.translation.y = p.pose.position.y
                        t.transform.translation.z = p.pose.position.z
                        t.transform.rotation.x = p.pose.orientation.x
                        t.transform.rotation.y = p.pose.orientation.y
                        t.transform.rotation.z = p.pose.orientation.z
                        t.transform.rotation.w = p.pose.orientation.w

                        self.br.sendTransform(t)

            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("new_particle_filter", anonymous=False)
    npf = NewParticleFilter()
    npf.run()
