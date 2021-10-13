#! /usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from rospy import Publisher, Subscriber
from turtlesim.msg import Pose
import math
import numpy as np


class Follower:
    def __init__(self):
        self.pub_2 = Publisher('/turtle2/cmd_vel', Twist)
        self.sub_1 = Subscriber('/turtle1/pose', Pose, self.follow)
        self.sub_2 = Subscriber('/turtle2/pose', Pose, self.update)
        self.pose = Pose()

    def update(self, my_pose):
        self.pose = my_pose

    def follow(self, target):
        msg = Twist()
        msg.linear.x = np.sqrt((target.y - self.pose.y) ** 2 + (target.x - self.pose.x) ** 2)
        msg.angular.z = (math.atan2(target.y - self.pose.y, target.x - self.pose.x) - self.pose.theta)

        while msg.angular.z > np.pi:
            msg.angular.z -= 2 * np.pi
        while msg.angular.z < -np.pi:
            msg.angular.z += 2 * np.pi

        self.pub_2.publish(msg)


rospy.init_node('main')
Follower()
rospy.spin()