#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose, PointStamped
from multi_car_msgs.msg import CarControl
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import numpy as np
import csv
from utils import get_id_to_index, get_index_to_id

class DataAnalyzer(object):

    def __init__(self):
        self.Ncars = rospy.get_param("/num_cars", 3)

        self.rate = rospy.Rate(100)
        self.bag_name = rospy.get_param("~bag_name", "2018-02-23-2cars-straight")
        self.listener = tf.TransformListener(True, rospy.Duration(1.0))
        self.car_id = int(rospy.get_param("~car_id", 1))
        self.car_ids = rospy.get_param("/car_ids", [])
        self.id_to_index = get_id_to_index(self.Ncars, self.car_ids, self.car_id)
        self.index_to_id = get_index_to_id(self.Ncars, self.car_ids, self.car_id)

        self.filename = "/home/brandon/projects/multi_car_ws/src/multi_car_localization/src/data" \
                        + self.bag_name + ".csv"

        self.trans = [None] * (self.Ncars - 1)
        self.estimate_sub = []
        self.data = {}
        self.fieldnames = []
        self.trans_pub = []
        self.vicon_control_pub = []
        self.delta_pub = rospy.Publisher("/car" + str(self.car_id) + "/delta",
                                         PointStamped, queue_size=1)

        for i in range(self.Ncars - 1):
            ID = str(self.index_to_id[i+1])
            time = "time" + str(self.car_id) + ID
            x = "x" + str(self.car_id) + ID
            y = "y" + str(self.car_id) + ID
            d = "d" + str(self.car_id) + ID
            theta = "theta" + str(self.car_id) + ID
            self.data[time] = []
            self.data[x] = []
            self.data[y] = []
            self.data[d] = []
            self.data[theta] = []
            self.fieldnames += [time, x, y, d, theta]

        for i in range(self.Ncars):
            num = str(self.index_to_id[i])
            self.vicon_control_pub.append(rospy.Publisher(
                "/car" + num + "/vicon_controls", CarControl, queue_size=1))
            if i != self.Ncars-1:
                self.trans_pub.append(rospy.Publisher(
                    "transpub" + num, PoseStamped, queue_size=1))
            if i != 0:
                self.estimate_sub.append(rospy.Subscriber(
                    "/car" + str(self.car_id) + "/car" + num + "/estimate", PoseStamped,
                    self.estimate_cb, (self.car_id, int(num)), queue_size=1))

        self.first_time = True

        rospy.on_shutdown(self.print_data)

    def print_data(self):
        car_id = str(self.car_id)
        sec_id = str(self.index_to_id[1])
        print "MEAN ERROR:", np.average(self.data["d" + car_id + sec_id])
        d = np.array(self.data["d" + car_id + sec_id])
        d2 = np.sqrt(np.sum(np.square(d))/len(d))
        print "RMSE: ", d2

    def estimate_cb(self, data, args):
        self.print_time = rospy.get_time()
        index = self.id_to_index[args[1]] - 1  # transform index
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

        dist = math.sqrt(delta.point.x**2 + delta.point.y**2)
        self.data["time" + str(self.car_id) + str(args[1])].append(rospy.get_time())
        self.data["x" + str(self.car_id) + str(args[1])].append(delta.point.x)
        self.data["y" + str(self.car_id) + str(args[1])].append(delta.point.y)
        self.data["d" + str(self.car_id) + str(args[1])].append(dist)
        self.data["theta" + str(self.car_id) + str(args[1])].append(delta.point.z)

        self.delta_pub.publish(delta)

        write = 'ab'
        if self.first_time:
            write = 'wb'
            self.first_time = False

        with open(self.filename, write) as csvfile:
            writer = csv.DictWriter(csvfile, delimiter=' ', quotechar='|',
                                quoting=csv.QUOTE_MINIMAL, fieldnames=self.fieldnames)
            row = {}
            time = "time" + str(self.car_id) + str(args[1])
            x = "x" + str(self.car_id) + str(args[1])
            y = "y" + str(self.car_id) + str(args[1])
            d = "d" + str(self.car_id) + str(args[1])
            theta = "theta" + str(self.car_id) + str(args[1])
            row[time] = self.data[theta][-1]
            row[x] = self.data[x][-1]
            row[y] = self.data[y][-1]
            row[d] = self.data[d][-1]
            row[theta] = self.data[theta][-1]
            writer.writerow(row)


    def run(self):
        while not rospy.is_shutdown():
            car_id = str(self.car_id)
            for index in range(self.Ncars):
                i = self.index_to_id[index]
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
