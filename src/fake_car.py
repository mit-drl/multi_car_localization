#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped
from multi_car_localization.msg import CarMeasurement
from multi_car_localization.msg import CarState
from std_msgs.msg import Header
import math
import numpy as np
import random
import tf

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

z0 = [gps_x0; gps_y0; uwb_01; uwb_02]
x0 = [x0, y0]
x = [x0, y0, x1, y1, x2, y2]
z0 = H0*x
H0 = [1 0 0 0 0 0
	  0 1 0 0 0 0
	  (x0 - x1)^2 + (y0 - y1)^2
	  (x0 - x2)^2 + (y0 - y2)^2]
# ^ assuming that the uwb ranges are squared
lineared H0 = [1  0  0  0  0  0
			   0  1  0  0  0  0
			   2  2 -2 -2  0  0
			   2  2  0  0 -2 -2]

"""

class FakeCar(object):

	def __init__(self):
		self.rate = rospy.Rate(rospy.get_param("frequency", 50))
		self.frame_id = rospy.get_param("~frame_id", "car0")

		self.x0 = np.array([10*random.random(), 10*random.random(),
							math.pi*(random.random()-0.5)], dtype=np.float64)
		self.x = self.x0
		self.u = (0.1*(random.random()-0.5), 7.0)
		self.current_time = rospy.get_time()
		self.prev_time = self.current_time

		self.state = CarState()
		self.state.header = Header()
		self.state.header.frame_id = self.frame_id
		self.state.u.append(self.u[0])
		self.state.u.append(self.u[1])

		self.pose_pub = rospy.Publisher("/range_position", CarState, queue_size=1)

	def publish_pose(self):
		#self.state.ps.pose.position.x = self.x[0]
		#self.state.ps.pose.position.y = self.x[1]

		self.state.state = self.x.tolist()

		#print "%s real yaw: %f" % (self.frame_id, self.x[2]*180./math.pi)

		self.pose_pub.publish(self.state)


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
			self.current_time = rospy.get_time()
			self.publish_pose()
			dt = self.current_time - self.prev_time
			self.x = self.state_transition(self.x, self.u, dt)
			self.prev_time = self.current_time
			self.rate.sleep()

if __name__ == "__main__":
	frame_id = rospy.get_param("~frame_id", "car0")

	rospy.init_node("fakecar", anonymous=False)
	car = FakeCar()
	car.run()