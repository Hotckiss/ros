#! /usr/bin/python3

import numpy as np

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan

rospy.init_node('viz_map')


class LaserCallback:
    def __init__(self):
        self.rate = rospy.Rate(20)
        self.marker_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

    def create_marker(self, points):
        marker = Marker()

        marker.id = 0
        marker.action = 0
        marker.header.frame_id = "base_laser_link"
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.67
        marker.type = marker.POINTS
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.points = points

        return marker

    def __call__(self, msg: LaserScan):
        laser_ranges = np.array(msg.ranges)
        angles = msg.angle_min + msg.angle_increment * np.arange(len(msg.ranges))

        outliers_mask = np.zeros_like(laser_ranges, dtype=bool)
        outliers_mask[1:-1] = np.abs(laser_ranges[1:] - laser_ranges[:-1]) > 0.1

        points = [Point(x, y, 0.0) for x, y in zip((laser_ranges * np.cos(angles))[~outliers_mask], (laser_ranges * np.sin(angles))[~outliers_mask])]
        marker = self.create_marker(points)

        self.rate.sleep()
        self.marker_publisher.publish(marker)


subscriber = rospy.Subscriber('/base_scan', LaserScan, LaserCallback())

rospy.spin()