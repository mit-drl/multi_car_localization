#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped
from multi_car_msgs.msg import CarMeasurement
from multi_car_msgs.msg import CarState
from std_msgs.msg import Header
import math
import numpy as np
import random
import tf
from dynamics import RoombaDynamics

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

        self.x0 = np.array([10*random.random(), 10*random.random(),
                            math.pi*(random.random()-0.5)], dtype=np.float64)
        self.x = self.x0
        self.u = (1.2*(random.random()-0.5), 7.0)
        self.current_time = rospy.get_time()
        self.prev_time = self.current_time

        self.state = CarState()
        self.state.header = Header()
        self.state.header.frame_id = self.frame_id
        self.state.u.append(self.u[0])
        self.state.u.append(self.u[1])
        self.state.car_id = 0

        self.robot = RoombaDynamics()

        self.pose_pub = rospy.Publisher("/range_position", CarState, queue_size=1)

    def publish_pose(self):
        #self.state.ps.pose.position.x = self.x[0]
        #self.state.ps.pose.position.y = self.x[1]

        self.state.state = self.x.tolist()

        #print "%s real yaw: %f" % (self.frame_id, self.x[2]*180./math.pi)

        self.pose_pub.publish(self.state)

    def run(self):
        while not rospy.is_shutdown():
            self.current_time = rospy.get_time()
            self.publish_pose()
            dt = self.current_time - self.prev_time
            self.x = self.robot.state_transition(self.x, self.u, dt)
            self.prev_time = self.current_time
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("fakecar", anonymous=False)
    car = FakeCar()
    car.run()