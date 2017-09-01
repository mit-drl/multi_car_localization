#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped
from multi_car_msgs.msg import CarMeasurement
from multi_car_msgs.msg import CarState
from multi_car_msgs.msg import CarControl
from std_msgs.msg import Header
import math
import numpy as np
import random
import tf
from dynamics import DubinsVelocityDynamics

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
        self.rate = rospy.Rate(rospy.get_param("~frequency", 20))
        self.frame_id = rospy.get_param("~frame_id", "car0")

        self.x0 = np.array([50.*random.random(), 50.*random.random(),
                            0.1*(random.random()-0.5)], dtype=np.float64)
        self.init_angle = [-0.1, 1.0, 2.5, -0.5]
        ID = int(self.frame_id[-1])
        self.x0[2] = self.init_angle[ID]

        # self.x0 = np.array([10*random.random(), 10*random.random(),
        #                     math.pi*(random.random()-0.5)], dtype=np.float64)
        self.x = self.x0
        self.u = [0.3*(random.random()-0.5), 5.0]
        self.current_time = rospy.get_time()
        self.prev_time = self.current_time

        self.initial_state = self.x0.tolist()
        self.br = tf.TransformBroadcaster()

        self.state = CarState()
        self.state.header = Header()
        self.state.header.frame_id = self.frame_id
        self.state.u.append(0.0)
        self.state.u.append(0.0)
        # self.state.u.append(self.u[0])
        # self.state.u.append(self.u[1])
        self.state.car_id = int(self.frame_id[-1])

        self.control = CarControl()
        self.control.header = Header()
        self.control.header.frame_id = self.frame_id
        self.control.car_id = int(self.frame_id[-1])

        self.robot = DubinsVelocityDynamics()

        self.pose_pub = rospy.Publisher("/range_position", CarState, queue_size=1)
        self.control_pub = rospy.Publisher("control", CarControl, queue_size=1)
        self.control_pub2 = rospy.Publisher("/control", CarControl, queue_size=1)

    def publish_pose(self):
        self.state.state = self.x.tolist()
        self.state.header.stamp = rospy.Time.now()

        self.control.header.stamp = rospy.Time.now()
        self.control.steering_angle = self.state.u[0]
        self.control.velocity = self.state.u[1]

        self.pose_pub.publish(self.state)
        self.control_pub.publish(self.control)
        self.control_pub2.publish(self.control)

    def run(self):
        maxcounts = 180
        count = 0
        while not rospy.is_shutdown():
            self.br.sendTransform((self.initial_state[0], self.initial_state[1], 0),
                tf.transformations.quaternion_from_euler(0, 0, self.initial_state[2]),
                rospy.Time.now(),
                "true_" + self.frame_id,
                "map")

            self.current_time = rospy.get_time()
            self.publish_pose()
            if count < maxcounts:
                self.x = self.x0
                count += 1
                if count == maxcounts:
                    self.state.u[0] = self.u[0]
                    self.state.u[1] = self.u[1]
            else:
                dt = self.current_time - self.prev_time
                u1 = self.u[0] + np.random.normal(0, 2.0*dt)
                u2 = self.u[1] + np.random.normal(0, 3.0*dt)
                new_u = (u1, u2)
                self.x = self.robot.state_transition(self.x, new_u, dt)
                # self.x[2] = self.x[2] % (2*math.pi)
            self.prev_time = self.current_time
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("fakecar", anonymous=False)
    car = FakeCar()
    car.run()