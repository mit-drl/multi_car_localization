#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose
from multi_car_msgs.msg import CarControl
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
from rosgraph_msgs.msg import Clock

class TransformPublisher(object):

    def __init__(self):
        self.Ncars = 3

        self.rate = rospy.Rate(100)
        self.listener = tf.TransformListener(True, rospy.Duration(1.0))
        self.car_id = 1
        self.car_ids = [1, 2, 3]

        self.prev_time = None
        self.prev_ps = [None] * self.Ncars

        self.trans_pub = []
        self.vicon_control_pub = []
        for i in range(self.Ncars):
            num = str(i+1)
            self.vicon_control_pub.append(rospy.Publisher(
                "/car" + num + "/vicon_controls", CarControl, queue_size=1))
            if i != self.Ncars-1:
                self.trans_pub.append(rospy.Publisher(
                    "transpub" + num, PoseStamped, queue_size=1))
            # self.trans_sub.append(
            #     "/vicon/car" + num + "/car" + num, TransformStamped,
            #     (i, ), self.vicon_cb, queue_size=1)

    def run(self):
        ticker = 1
        while not rospy.is_shutdown():
            car_id = str(self.car_id)
            current_time = rospy.Time.now()
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
                    ps = PoseStamped()
                    ps.header.stamp = rospy.Time.now()
                    ps.header.frame_id = "/car1/base_link"
                    ps.pose.position.x = trans[0]
                    ps.pose.position.y = trans[1]
                    ps.pose.position.z = trans[2]
                    ps.pose.orientation.x = rot[0]
                    ps.pose.orientation.y = rot[1]
                    ps.pose.orientation.z = rot[2]
                    ps.pose.orientation.w = rot[3]
                    self.trans_pub[i-2].publish(ps)

                now = rospy.Time(0)
                starting_trans = "/world"
                ending_trans = "/vicon/car" + num + "/car" + num
                self.listener.waitForTransform(starting_trans,
                    ending_trans, now, rospy.Duration(1.0))
                (trans, rot) = self.listener.lookupTransform(starting_trans,
                    ending_trans, now)

                p = Pose()
                p.position.x = trans[0]
                p.position.y = trans[1]
                p.position.z = trans[2]
                p.orientation.x = rot[0]
                p.orientation.y = rot[1]
                p.orientation.z = rot[2]
                p.orientation.w = rot[3]

                if None in self.prev_ps:
                    self.prev_ps[i-1] = p
                if self.prev_time is None:
                    self.prev_time = rospy.Time.now()

                dt = (current_time - self.prev_time).to_sec()
                if ticker % 5 == 0 and dt != 0:
                    print dt
                    dx = p.position.x - self.prev_ps[i-1].position.x
                    dy = p.position.y - self.prev_ps[i-1].position.y
                    oq = []
                    bad_quat = self.prev_ps[i-1].orientation
                    oq.append(bad_quat.x)
                    oq.append(bad_quat.y)
                    oq.append(bad_quat.z)
                    oq.append(bad_quat.w)
                    (_, _, prev_theta) = euler_from_quaternion(oq)
                    (_, _, cur_theta)  = euler_from_quaternion(rot)
                    dtheta = cur_theta - prev_theta
                    if dtheta < -math.pi:
                        dtheta += 2*math.pi
                    elif dtheta > math.pi:
                        dtheta -= 2*math.pi
                    vel = math.sqrt(dx**2 + dy**2)/dt
                    ang = dtheta/dt
                    control = CarControl()
                    control.header.stamp = rospy.Time.now() + rospy.Duration(0.7)
                    control.velocity = vel
                    control.steering_angle = ang
                    self.prev_ps[i-1] = p
                    self.vicon_control_pub[i-1].publish(control)
                    if i == self.Ncars:
                        self.prev_time = current_time

            ticker += 1
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("tf_bag_publisher", anonymous=False)
    tfpub = TransformPublisher()
    tfpub.run()
