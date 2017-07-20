#!/usr/bin/env python

import math
import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import PoseStamped

DIST_TOPIC = "range"

NODE_NAME = "fake_uwb"
n = roshelper.Node(NODE_NAME, anonymous=False)

@n.entry_point()
class FakeUWB:

    def __init__(self):
        self.rate = rospy.Rate(rospy.get_param("frequency", 100))
        self.ID = rospy.get_param("frame_id", "car1")
        self.ranges = {}
        self.rng = Range()
        self.rng.field_of_view = math.pi * 0.1
        self.rng.min_range = 0
        self.rng.max_range = 300
        self.pub = None
        self.offset = 0.0
        self.position = (0, 0)

    @n.subscriber('range_position', PoseStamped)
    def range_sub(self, ps):
        ID = ps.header.frame_id
        x = ps.position.x
        y = ps.position.y
        self1.ranges[ID] = copy(self.rng)
        self.ranges[ID].range = math.sqrt(x**2 + y**2)
        self.ranges[ID].header.frame_id = ID
        self.range_pub(self.ranges[ID])

    @n.publisher('range', Range)
    def range_pub(self, rng):
        return rng

    @n.subscriber('car_pose', PoseStamped)
    def car_sub(self, ps):
        if self.ID == ps.header.frame_id:
            self.position = (ps.position.x, ps.position.y)
            self.car_pub(ps)

    @n.publisher('range_position', PoseStamped)
    def car_pub(self, ps):
        return ps

    def run(self):
        pass

if __name__ == "__main__":
    n.start(spin=True)