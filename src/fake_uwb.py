#!/usr/bin/env python

import math
import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from multi_car_msgs.msg import CarMeasurement
from multi_car_msgs.msg import CarState
from multi_car_msgs.msg import UWBRange
import random
import copy

class FakeUWB(object):

    def __init__(self):
        self.rate = rospy.Rate(rospy.get_param("~frequency", 30))
        self.frame_id = rospy.get_param("~frame_id", "car0")
        self.ID = int(self.frame_id[-1])

        self.ranges = {}
        self.rng = UWBRange()
        self.rng.header = Header()
        self.rng.to_id = self.ID

        self.sigma = 0.15
        self.Ncars = 3
        self.uwbs_per_car = 1

        self.position = None

        self.positions = [None]*self.Ncars

        self.range_pub = rospy.Publisher('uwb', UWBRange, queue_size=1)
        self.range_sub = rospy.Subscriber('/range_position', CarState, self.range_sub_cb)

    def range_sub_cb(self, cs):
        frame_id = cs.header.frame_id
        ID = int(frame_id[-1])

        self.positions[ID] = (cs.state[0], cs.state[1])

    def publish_range(self):
        pos_good = True
        for pos in self.positions:
            if pos == None:
                pos_good = False

        if pos_good:
            for j, pos1 in enumerate(self.positions):
                for k, pos2 in enumerate(self.positions):
                    if k > j:
                        x = pos2[0]
                        y = pos2[1]
                        dist = math.sqrt((pos1[0] - x)**2 + (pos1[1] - y)**2)
                        rng = UWBRange()
                        rng.header = Header()
                        rng.header.stamp = rospy.Time.now()
                        rng.header.frame_id = self.frame_id
                        rng.to_id = j
                        rng.from_id = k
                        rng.distance = max(0.0, dist + random.gauss(0.0, self.sigma))
                        self.range_pub.publish(rng)   

                        rng.to_id = k
                        rng.from_id = j
                        rng.distance = max(0.0, dist + random.gauss(0.0, self.sigma))
                        self.range_pub.publish(rng)

    def run(self):
        while not rospy.is_shutdown():
            self.publish_range()
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("uwb", anonymous=False)
    uwb = FakeUWB()
    uwb.run()
    #rospy.spin()