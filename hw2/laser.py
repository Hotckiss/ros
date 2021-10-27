#! /usr/bin/python3

import numpy as np

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

rospy.init_node('viz_map')


class LaserCallback:
    def __init__(self):
        self.rate = rospy.Rate(20)
        self.marker_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        self.grid_publisher = rospy.Publisher('/grid', OccupancyGrid, queue_size=10)
        self.resolution = 0.1
        self.size = 10

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

    def create_grid(self):
        grid = OccupancyGrid()
        grid.header.frame_id = 'base_laser_link'
        grid.info.resolution = self.resolution
        grid.info.width = self.real_grid_size()
        grid.info.height = self.real_grid_size()

        return grid

    def real_grid_size(self):
        return 2 * int(self.size / self.resolution) + 3

    def locate_point(self, x, y):
        return int((x + self.size) // self.resolution),  int((y + self.size) // self.resolution)

    def __call__(self, msg: LaserScan):
        laser_ranges = np.array(msg.ranges)
        angles = msg.angle_min + msg.angle_increment * np.arange(len(msg.ranges))

        outliers_mask = np.zeros_like(laser_ranges, dtype=bool)
        outliers_mask[1:-1] = np.abs(laser_ranges[1:] - laser_ranges[:-1]) > 0.1

        xs = (laser_ranges * np.cos(angles))[~outliers_mask]
        ys = (laser_ranges * np.sin(angles))[~outliers_mask]

        # markers
        points = [Point(x, y, 0.0) for x, y in zip(xs, ys)]
        marker = self.create_marker(points)
        self.rate.sleep()
        self.marker_publisher.publish(marker)

        # grid
        grid = self.create_grid()

        data = np.zeros((self.real_grid_size(), self.real_grid_size()), dtype=int)
        for x, y in zip(xs, ys):
            if abs(x) < self.size and abs(y) < self.size:
                pt = self.locate_point(x, y)
                data[pt] = min(100, data[pt] + 5)

        grid.info.origin.position.x = -self.size
        grid.info.origin.position.y = -self.size
        grid.data = list(data.transpose(1, 0).reshape(-1))

        self.grid_publisher.publish(grid)


subscriber = rospy.Subscriber('/base_scan', LaserScan, LaserCallback())

rospy.spin()
