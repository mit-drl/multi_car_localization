#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from multi_car_localization.msg import CarMeasurement
from multi_car_localization.msg import CarState
from multi_car_localization.msg import CombinedState
from multi_car_localization.msg import ConsensusMsg
from nav_msgs.msg import Path
from geometry_msgs.msg import  Pose
import math
import random
from scipy.linalg import block_diag
import pdb

"""
State:
    [x0, y0
     x1, y1
     x2, y2]

Control:
    [dx0, dy0
     dx1, dy1
     dx2, dy2]

Measurements:
    [gps_x0, gps_y0, uwb_00, uwb_01, uwb_02;
     gps_x1, gps_y1, uwb_10, uwb_11, uwb_12;
     gps_x2, gps_y2, uwb_20, uwb_21, uwb_22]


Combined State:
	[x00, y00, t00, x01, y01, t01, x02, y02, t02;
	 x10, y10, t10, x11, y11, t11, x12, y12, t12;
	 x20, y20, t20, x21, y21, t21, x22, y22, t22]
row: frame_id
column: car_id

"""

class Consensus(object):

	def __init__(self):
		self.rate = rospy.Rate(rospy.get_param("frequency", 1))

		self.epsilon = rospy.get_param("epsilon", 0.01)
		self.K = rospy.get_param("consensus_iterations", 20)
		self.N = rospy.get_param("number_of_nodes", 3)
		self.Ncars = rospy.get_param("number_of_cars", 3)
		self.Ndim = rospy.get_param("~num_state_dim", 3)

		self.frame_id = rospy.get_param("~frame_id", "car0")

		self.xi_prior = None
		self.Ji_prior = np.zeros((self.Ncars*self.Ndim, self.Ncars*self.Ndim))

		self.x_post = None
		self.J_post = None
		self.Q = np.diag(self.Ncars * [0.1, 0.1, 0.1])

		self.zi = np.zeros((self.Ncars*self.Ndim,))
		self.Bi = np.zeros((self.Ncars*self.Ndim, self.Ncars*self.Ndim))
		self.u = None

		self.Hi = np.identity(self.Ncars*self.Ndim)

		self.x_sub = rospy.Subscriber("combined", CombinedState, self.x_cb)

		self.vi = None
		self.Vi = None

		self.vj = {}
		self.Vj = {}
		self.v_sub = []
		for i in range(self.Ncars):
			if i != int(self.frame_id[-1]):
				self.v_sub.append(
					rospy.Subscriber("/car" + str(i) + "/v", ConsensusMsg, self.v_cb))

		self.v_pub = rospy.Publisher("v", ConsensusMsg, queue_size=1)
		
		self.paths = []
		self.consensus_pub = []
		for i in range(self.Ncars):
			path = Path()
			path.header = Header()
			path.header.stamp = rospy.Time.now()
			path.header.frame_id = "map"
			self.paths.append(path)
			self.consensus_pub.append(
				rospy.Publisher("consensus" + str(i), Path, queue_size=1))
		self.prev_time = rospy.get_time()

	def x_cb(self, cs):
		frame_id = cs.header.frame_id
		self.u = np.array(cs.u)
		self.zi = np.array(cs.state)
		self.Bi = np.array(cs.inf).reshape((self.Ncars*self.Ndim, self.Ncars*self.Ndim))

		if self.xi_prior == None:
			self.xi_prior = self.zi

	def v_cb(self, v):
		frame_id = v.header.frame_id
		self.vj[frame_id] = np.array(v.vj)
		self.Vj[frame_id] = np.array(v.Vj).reshape((self.Ncars*self.Ndim, self.Ncars*self.Ndim))

	def consensus(self, vi, Vi, vj, Vj):
		new_Vi = Vi
		new_vi = vi
		for key in Vj:
			new_Vi += self.epsilon*(Vj[key] - Vi)
			new_vi += self.epsilon*(vj[key] - vi)
		return new_vi, new_Vi

	def icf(self):
		dt = rospy.get_time() - self.prev_time
		self.prev_time = rospy.get_time()
		if self.x_post is not None:
			for i in range(self.Ncars):
				self.xi_prior[i*self.Ndim:i*self.Ndim+3] = self.state_transition(
					self.x_post[i*self.Ndim:i*self.Ndim+3], self.u[i*2:i*2+2], dt)
			self.Ji_prior += self.Q

		self.Vi = self.Ji_prior/self.Ncars + np.dot(self.Hi.T,np.dot(self.Bi,self.Hi))
		self.vi = np.dot(self.Ji_prior,self.xi_prior)/self.Ncars + np.dot(self.Hi.T,np.dot(self.Bi,self.zi))

		self.starting_counter = -1
		self.current_counter = 0
		self.vj = {}
		self.Vj = {}

		while self.current_counter < self.K:
			self.publish_v()

			if self.starting_counter == self.current_counter:
				pass
			else:
				if len(self.vj) == self.Ncars -1 and len(self.Vj) == self.Ncars-1:
					self.starting_counter = self.current_counter
					self.vi, self.Vi = self.consensus(self.vi, self.Vi, self.vj, self.Vj)
					self.vj = {}
					self.Vj = {}
					self.current_counter += 1

		self.x_post = np.dot(np.linalg.inv(self.Vi),self.vi)
		self.J_post = self.Ncars*self.Vi

		for i in range(self.Ncars):
			pose = PoseStamped()
			pose.header = Header()
			pose.header.frame_id = "map"
			pose.header.stamp = rospy.Time().now()
			pose.pose.position.x = self.x_post[i*self.Ndim]
			pose.pose.position.y = self.x_post[i*self.Ndim + 1]
			self.paths[i].poses.append(pose)
			self.consensus_pub[i].publish(self.paths[i])
		#self.state_pub.publish(self.x_post, self.J_post)

	def publish_v(self):
		v_msg = ConsensusMsg()
		v_msg.header = Header()
		v_msg.header.stamp = rospy.Time.now()
		v_msg.header.frame_id = self.frame_id
		v_msg.vj = self.vi.flatten().tolist()
		v_msg.Vj = self.Vi.flatten().tolist()
		self.v_pub.publish(v_msg)

	def state_transition(self, x, u, dt):
		# u is a tuple (u_d, u_a)
		steps = 1.
		h = dt/steps
		x = [x]
		for i in range(0, int(steps)):
			k1 = self.state_transition_model(x[i], u)
			k2 = self.state_transition_model(x[i] + 0.5*h*k1, u)
			k3 = self.state_transition_model(x[i] + 0.5*h*k2, u)
			k4 = self.state_transition_model(x[i] + k3*h, u)
			new_x = x[i] + (h/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4)
			x.append(new_x)
		return x[-1]

	def state_transition_model(self, state, u):
		u_d, u_v, = u
		x, y, phi = state
		dx = u_v*math.cos(phi)
		dy = u_v*math.sin(phi)
		dphi = (u_v/3.)*math.tan(u_d)
		return np.array([dx, dy, dphi])

	def run(self):
		while not rospy.is_shutdown():
			if self.Bi is not None and self.xi_prior is not None:
				self.icf()

			# self.rate.sleep()

if __name__ == "__main__":
	rospy.init_node("consensus", anonymous=False)
	consensus = Consensus()
	consensus.run()