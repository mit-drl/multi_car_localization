#!/usr/bin/env python

import math
import rospy
from multi_car_msgs.msg import CarState
from multi_car_msgs.msg import CombinedState
from geometry_msgs.msg import Point
import numpy as np

class Metrics(object):

    def __init__(self):
        self.rate = rospy.Rate(rospy.get_param("~frequency", 100))
        self.frame_id = rospy.get_param("~frame_id", "car0")
        self.ID = int(self.frame_id[-1])

        self.Ncars = rospy.get_param("~num_cars", 3)
        self.Ndim = rospy.get_param("~num_state_dim", 3)

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
                rospy.Publisher("consensus_error", Point, queue_size=1))
        for i in range(self.Nconn):
            self.pf_pub.append(
                rospy.Publisher("pf_error", Point, queue_size=1))

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
        for pf in self.pf_state:
            if pf == None:
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
        for state in self.true_state:
            # particle filter error
            for i, ID in enumerate(self.own_connections):
                pf_state = self.pf_state[ID]
                error = Point()
                error.x = pf_state[0] - state[0]
                error.y = pf_state[1] - state[1]
                error.z = pf_state[2] - state[2]
                self.pf_pub[i].publish(error)

            for con_state in self.con_state:
                error = Point()
                error.x = con_state[0] - state[0]
                error.y = con_state[1] - state[1]
                error.z = con_state[2] - state[2]
                self.con_pub[i].publish(error)

    def run(self):
        while not rospy.is_shutdown():
            self.publish_errors()
            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("metrics", anonymous=False)
    metrics = Metrics()
    metrics.run()