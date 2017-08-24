#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from multi_car_msgs.msg import CarMeasurement
from multi_car_msgs.msg import CarState
from multi_car_msgs.msg import CombinedState
from multi_car_msgs.msg import ConsensusMsg
from nav_msgs.msg import Path
from geometry_msgs.msg import  Pose
import math
import random
from scipy.linalg import block_diag
import pdb
from dynamics import RoombaDynamics

import dict_to_graph
import networkx as nx

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
		self.rate = rospy.Rate(rospy.get_param("~frequency", 100))

		self.epsilon = rospy.get_param("~epsilon", 0.2)
		self.K = rospy.get_param("~consensus_iterations", 15)
		self.Ncars = rospy.get_param("~num_cars", 3)
		self.Ndim = rospy.get_param("~num_state_dim", 3)
		self.frame_id = rospy.get_param("~frame_id", "car0")

		self.connections = rospy.get_param("/connections", None)
		self.own_connections = self.connections[self.frame_id[-1]]
		self.Nconn = len(self.own_connections)

		self.full_graph = dict_to_graph.convert(self.connections)
		self.graph = dict_to_graph.prune(self.full_graph, int(self.frame_id[-1]))


		self.robot = RoombaDynamics()

		self.xi_prior = None
		self.Ji_prior = np.zeros((self.Ncars*self.Ndim, self.Ncars*self.Ndim))

		self.x_post = None
		self.J_post = None

		self.zi = np.zeros((self.Nconn*self.Ndim,))
		self.Bi = np.zeros((self.Nconn*self.Ndim, self.Nconn*self.Ndim))
		self.u =  np.zeros((self.Ncars, 2))

		# self.Hi = np.identity(self.Nconn*self.Ndim)

		# process noise
		self.Q = np.diag(self.Ncars * [0.4, 0.4, 0.1])
		# measurement matrix
		self.Hi = np.zeros((self.Nconn*self.Ndim, self.Ncars*self.Ndim))
		identity = np.identity(self.Ndim)
		for i, ID in enumerate(self.own_connections):
			self.Hi[i*self.Ndim:(i+1)*self.Ndim, ID*self.Ndim:(ID+1)*self.Ndim] = identity
		for i in range(self.Ncars):
			if i not in self.own_connections:
				self.Q[i*self.Ndim:(i+1)*self.Ndim, i*self.Ndim:(i+1)*self.Ndim] = np.diag([100.0, 100.0, 3.14])


		self.x_sub = rospy.Subscriber("combined", CombinedState, self.x_cb)

		self.vi = None
		self.Vi = None

		self.vj = {}
		self.Vj = {}
		self.v_sub = rospy.Subscriber("/consensus", ConsensusMsg, self.consensus_cb)
		# for i in self.own_connections:
		# 	if i != int(self.frame_id[-1]):
		# 		self.v_sub.append(
		# 			rospy.Subscriber("/car" + str(i) + "/v", ConsensusMsg, self.consensus_cb))

		self.v_pub = rospy.Publisher("/consensus", ConsensusMsg, queue_size=1)
		
		self.paths = []
		self.consensus_pub = []
		self.consensus_state_pub = rospy.Publisher("consensus_state", CombinedState, queue_size=1)
		for i in range(self.Ncars):
			path = Path()
			path.header = Header()
			path.header.stamp = rospy.Time.now()
			path.header.frame_id = "map"
			self.paths.append(path)
			self.consensus_pub.append(
				rospy.Publisher("consensus" + str(i), Path, queue_size=1))
		self.prev_time = rospy.get_time()
		self.new_meas = False

	def x_cb(self, cs):
		frame_id = cs.header.frame_id
		u = np.array(cs.u)
		for i, ID in enumerate(self.own_connections):
			self.u[ID] = u[i*2:(i+1)*2]

		self.zi = np.array(cs.state)
		self.Bi = np.array(cs.inf).reshape((self.Nconn*self.Ndim, self.Nconn*self.Ndim))

		if self.xi_prior == None:
			self.xi_prior = np.zeros((self.Ncars*self.Ndim,))
			for i, ID in enumerate(self.own_connections):
				self.xi_prior[ID*self.Ndim:(ID+1)*self.Ndim] = self.zi[i*self.Ndim:(i+1)*self.Ndim]

		self.new_meas = True

	def consensus_cb(self, v):
		if v.header.frame_id != self.frame_id and int(v.header.frame_id[-1]) in self.own_connections:
			frame_id = v.header.frame_id
			self.vj[frame_id] = np.array(v.states)
			self.Vj[frame_id] = np.array(v.confidences).reshape((self.Ncars*self.Ndim, self.Ncars*self.Ndim))

	def consensus(self, vi, Vi, vj, Vj):
		new_Vi = Vi
		new_vi = vi
		for key in Vj:
			new_Vi += self.epsilon*(Vj[key] - Vi)
			new_vi += self.epsilon*(vj[key] - vi)

		return new_vi, new_Vi

	def icf(self):
		dt = rospy.get_time() - self.prev_time
		if dt > 1.0:
			print "DT BIG: %f" % (dt)
		#print dt
		self.prev_time = rospy.get_time()
		if self.x_post is not None:
			for i in range(self.Ncars):
				self.xi_prior[i*self.Ndim:(i+1)*self.Ndim] = self.robot.state_transition(
					self.x_post[i*self.Ndim:(i+1)*self.Ndim], self.u[i], dt)
			phi = self.robot.phi(self.x_post, self.u, dt, self.Ncars, self.Ndim)
			self.Ji_prior = np.linalg.inv(phi*np.linalg.inv(self.J_post)*phi.T + self.Q)

		self.Vi = self.Ji_prior/self.Ncars + np.dot(self.Hi.T,np.dot(self.Bi,self.Hi))
		self.vi = np.dot(self.Ji_prior,self.xi_prior)/self.Ncars \
			+ np.dot(self.Hi.T,np.dot(self.Bi,self.zi))

		self.current_counter = 0
		self.vj = {}
		self.Vj = {}

		st0 = rospy.get_time()
		while self.current_counter < self.K and not rospy.is_shutdown():
			self.publish_v()
			if len(self.vj) == self.Nconn -1 and len(self.Vj) == self.Nconn-1:
				self.vi, self.Vi = self.consensus(self.vi, self.Vi, self.vj, self.Vj)
				self.vj = {}
				self.Vj = {}
				self.current_counter += 1
			else:
				self.rate.sleep()
			# this is meant to break out of the loop
			# if you kill the node
			# if rospy.get_time() - st0 > 2.0:
			# 	break
		tim3 = rospy.get_time() - st0
		print "consensus loops:  %f" % (tim3)

		self.x_post = np.dot(np.linalg.inv(self.Vi),self.vi)
		self.J_post = self.Ncars*self.Vi

		cs = CombinedState()
		cs.header = Header()
		cs.header.frame_id = "map"
		cs.header.stamp = rospy.Time().now()
		cs.state = self.x_post.flatten().tolist()
		self.consensus_state_pub.publish(cs)

		for i in range(self.Ncars):
			pose = PoseStamped()
			pose.header = Header()
			pose.header.frame_id = "map"
			pose.header.stamp = rospy.Time().now()
			pose.pose.position.x = self.x_post[i*self.Ndim]
			pose.pose.position.y = self.x_post[i*self.Ndim + 1]
			self.paths[i].poses.append(pose)
			if len(self.paths[i].poses) > 300:
				self.paths[i].poses.pop(0)
			self.consensus_pub[i].publish(self.paths[i])
		#self.state_pub.publish(self.x_post, self.J_post)

	def publish_v(self):
		v_msg = ConsensusMsg()
		v_msg.car_id = int(self.frame_id[-1])
		v_msg.header = Header()
		v_msg.header.stamp = rospy.Time.now()
		v_msg.header.frame_id = self.frame_id
		v_msg.states = self.vi.flatten().tolist()
		v_msg.confidences = self.Vi.flatten().tolist()
		self.v_pub.publish(v_msg)

	def run(self):
		while not rospy.is_shutdown():
			st3 = rospy.get_time()
			if self.Bi is not None and self.xi_prior is not None:
				if self.new_meas == True:
					self.new_meas = False
					self.icf()

					tim3 = rospy.get_time() - st3
					print "CONSENSUS:        %f" % (tim3)	

			#self.rate.sleep()

if __name__ == "__main__":
	rospy.init_node("consensus", anonymous=False)
	consensus = Consensus()
	consensus.run()
	#rospy.spin()