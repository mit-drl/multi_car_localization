#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose, PointStamped
from multi_car_msgs.msg import CarControl
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import csv

class DataAnalyzer(object):

    def __init__(self):
        self.Ncars = 3

        self.rate = rospy.Rate(100)
        self.listener = tf.TransformListener(True, rospy.Duration(1.0))
        self.car_id = 1
        self.car_ids = [1, 2, 3]

        self.filename = "/home/brandon/projects/multi_car_ws/src/multi_car_localization/src/data.csv"

        self.trans = [None] * (self.Ncars - 1)
        self.estimate_sub = []
        self.trans_pub = []
        self.vicon_control_pub = []
        self.delta_pub = rospy.Publisher("/car" + str(self.car_id) + "/delta",
                                         PointStamped, queue_size=1)
        for i in range(self.Ncars):
            num = str(i+1)
            self.vicon_control_pub.append(rospy.Publisher(
                "/car" + num + "/vicon_controls", CarControl, queue_size=1))
            if i != self.Ncars-1:
                self.trans_pub.append(rospy.Publisher(
                    "transpub" + num, PoseStamped, queue_size=1))
            if i != 0:
                self.estimate_sub.append(rospy.Subscriber(
                    "/car" + str(self.car_id) + "/car" + num + "/estimate", PoseStamped,
                    self.estimate_cb, (self.car_id, i), queue_size=1))

    def estimate_cb(self, data, args):
        index = args[1] - 1
        if self.trans[index] is None:
            return
        est = data.pose
        delta = PointStamped()
        delta.header.stamp = rospy.Time.now()
        delta.header.frame_id = str(self.car_id) + "-" + str(args[1]+1)
        delta.point.x = est.position.x - self.trans[index].position.x
        delta.point.y = est.position.y - self.trans[index].position.y

        eo = est.orientation
        to = self.trans[index].orientation
        (_, _, est_theta) = euler_from_quaternion([eo.x, eo.y, eo.z, eo.w])
        (_, _, trans_theta) = euler_from_quaternion([to.x, to.y, to.z, to.w])

        delta.point.z = est_theta - trans_theta
        self.delta_pub.publish(delta)
        with open(self.filename, 'ab') as csvfile:
            writer = csv.writer(csvfile, delimiter=' ', quotechar='|',
                                quoting=csv.QUOTE_MINIMAL)
            writer.writerow([0] * 4 * (args[1] == 1) +
                [rospy.get_time(), delta.point.x, delta.point.y, delta.point.z] +
                [0] * 4 * (args[1] == 2))

    def run(self):
        while not rospy.is_shutdown():
            car_id = str(self.car_id)
            for i in self.car_ids:
                num = str(i)
                if num != self.car_id:
                    now = rospy.Time(0)
                    starting_trans = "/vicon/car" + car_id + "/car" + car_id
                    ending_trans = "/vicon/car" + num + "/car" + num
                    self.listener.waitForTransform(starting_trans,
                        ending_trans, now, rospy.Duration(0.5))
                    (trans, rot) = self.listener.lookupTransform(starting_trans,
                        ending_trans, now)
                    ps = Pose()
                    ps.position.x = trans[0]
                    ps.position.y = trans[1]
                    ps.position.z = trans[2]
                    ps.orientation.x = rot[0]
                    ps.orientation.y = rot[1]
                    ps.orientation.z = rot[2]
                    ps.orientation.w = rot[3]
                    self.trans[i-2] = ps

            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("tf_bag_publisher", anonymous=False)
    tfpub = DataAnalyzer()
    tfpub.run()
