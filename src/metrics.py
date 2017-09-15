#!/usr/bin/env python

import math
import rospy
from multi_car_msgs.msg import CarState
from multi_car_msgs.msg import CombinedState
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import numpy as np
import dynamics

class Metrics(object):

    def __init__(self):
        self.rate = rospy.Rate(rospy.get_param("~frequency", 100))
        self.car_id = rospy.get_param("~car_id", 0)
        self.frame_name = rospy.get_param("/frame_name")
        self.frame_id = self.frame_name[self.car_id]
        self.ID = self.car_id

        self.Ncars = rospy.get_param("/num_cars", 3)
        self.dynamics_model = rospy.get_param("~dynamics_model", "dubins")
        self.dynamics = dynamics.model(self.dynamics_model)
        self.Ndim = self.dynamics.Ndim
        self.Ninputs = self.dynamics.Ninputs

        self.id_dict = rospy.get_param("/id_dict", None)
        self.connections = rospy.get_param("/connections", None)
        self.own_connections = self.connections[self.frame_id[-1]]
        self.Nconn = len(self.own_connections)

        self.true_state = [None]*self.Ncars
        self.pf_state = [None]*self.Ncars
        self.con_state = [None]*self.Ncars

        self.pf_pub = []
        self.con_pub = []
        for i in range(self.Ncars):
            self.con_pub.append(
                rospy.Publisher("consensus_error" + str(i), PoseStamped, queue_size=1))
        for i in range(self.Nconn):
            self.pf_pub.append(
                rospy.Publisher("pf_error" + str(self.own_connections[i]), PoseStamped, queue_size=1))

        self.truth_sub = rospy.Subscriber('/range_position', CarState, self.truth_cb)
        self.pf_sub = rospy.Subscriber('combined', CombinedState, self.pf_cb)
        self.con_sub = rospy.Subscriber('consensus_state', CombinedState, self.consensus_cb)

    def truth_cb(self, cs):
        ID = self.id_dict[str(cs.car_id)]
        self.true_state[ID] = cs.state

    def pf_cb(self, cs):
        states = np.array(cs.state).reshape((self.Nconn, self.Ndim))
        for i, ID in enumerate(self.own_connections):
            self.pf_state[ID] = states[i]

    def consensus_cb(self, cm):
        states = np.array(cm.state).reshape((self.Ncars, self.Ndim))
        for ID in range(self.Ncars):
            self.con_state[ID] = states[ID]

    def publish_errors(self):
        pf_good = True
        for ID in self.own_connections:
            pf = self.pf_state[ID]
            if pf is None:
                pf_good = False

        if pf_good:
            if None not in self.con_state:
                if None not in self.true_state:
                    # publish
                    self.calculate_and_publish_errors()
                    # reset
                    self.true_state = [None]*self.Ncars
                    self.pf_state = [None]*self.Ncars
                    self.con_state = [None]*self.Ncars

    def calculate_and_publish_errors(self):
        for i, state in enumerate(self.true_state):
            # particle filter error
            if i in self.own_connections:
                pf_state = self.pf_state[i]
                error = PoseStamped()
                error.header = Header()
                error.header.stamp = rospy.Time.now()
                error.header.frame_id = "map"
                x = pf_state[0] - state[0]
                y = pf_state[1] - state[1]
                error.pose.position.x = np.sqrt(x**2 + y**2)
                error.pose.position.y = 0.0
                error.pose.position.z = pf_state[2] - state[2]
                self.pf_pub[self.own_connections.index(i)].publish(error)

            con_state = self.con_state[i]
            error = PoseStamped()
            error.header = Header()
            error.header.stamp = rospy.Time.now()
            error.header.frame_id = "map"
            x = con_state[0] - state[0]
            y = con_state[1] - state[1]
            error.pose.position.x = np.sqrt(x**2 + y**2)
            error.pose.position.y = 0.0
            error.pose.position.z = con_state[2] - state[2]
            self.con_pub[i].publish(error)

    def run(self):
        while not rospy.is_shutdown():
            self.publish_errors()
            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("metrics", anonymous=False)
    metrics = Metrics()
    metrics.run()