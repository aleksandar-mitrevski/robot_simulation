#!/usr/bin/env python
import roslib
roslib.load_manifest('robot')

from math import sqrt, cos, sin
import numpy as np

import rospy
import tf
from sensor_msgs.msg import LaserScan
from map.srv import PositionOfClosestObstacle

class LaserScanNode(object):
    def __init__(self):
        self.number_of_readings = int(rospy.get_param('~number_of_readings', '180'))
        self.scanner_min_angle = float(rospy.get_param('~scanner_min_angle', '-1.57'))
        self.scanner_max_angle = float(rospy.get_param('~scanner_max_angle', '1.57'))
        self.scanner_angle_increment = float(rospy.get_param('~scanner_angle_increment', '0.0174'))
        self.scanner_min_range = float(rospy.get_param('~scanner_min_range', '0.0'))
        self.scanner_max_range = float(rospy.get_param('~scanner_max_range', '5.0'))
        self.scanner_frequency = float(rospy.get_param('~scanner_frequency', '60'))
        self.scanner_time_increment = float(rospy.get_param('~scanner_time_increment', '0.00009259'))
        self.noise_standard_deviation = float(rospy.get_param('~noise_standard_deviation', '1.'))

        self.scan_publisher = rospy.Publisher('laser_scan', LaserScan, queue_size=100)
        self.laser_frame = '/laser_front'

        self.tf_listener = tf.TransformListener()
        self.tf_listener.waitForTransform('/map', self.laser_frame, rospy.Time(0), rospy.Duration(10.))

        while not rospy.is_shutdown():
            laser_position, obstacle_positions = self.read_obstacle_positions()
            self.publish_scans(laser_position, obstacle_positions)

    def read_obstacle_positions(self):
        laser_position = -1. * np.ones(2)
        laser_obstacle_positions = -1. * np.ones((self.number_of_readings, 2))

        try:
            (translation, quat_rotation) = self.tf_listener.lookupTransform('/map', self.laser_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return laser_positions, laser_obstacle_positions

        laser_position[0] = translation[0]
        laser_position[1] = translation[1]
        euler_rotation = tf.transformations.euler_from_quaternion(quat_rotation)

        angle = euler_rotation[2] + self.scanner_min_angle
        for i in xrange(self.number_of_readings):
            direction_x = cos(angle)
            direction_y = sin(angle)

            rospy.wait_for_service('position_of_closest_obstacle')
            try:
                proxy = rospy.ServiceProxy('position_of_closest_obstacle', PositionOfClosestObstacle)
                result = proxy(laser_position[0], laser_position[1], direction_x, direction_y)
                if result.success == 'success':
                    laser_obstacle_positions[i,0] = result.position_x
                    laser_obstacle_positions[i,1] = result.position_y
                else:
                    laser_obstacle_positions[i,0] = self.scanner_max_range
                    laser_obstacle_positions[i,1] = self.scanner_max_range
            except rospy.ServiceException, e:
                rospy.logerr('position_of_closest_obstacle service call failed')

            angle = angle + self.scanner_angle_increment

        return laser_position, laser_obstacle_positions

    def publish_scans(self, laser_position, obstacle_positions):
        ranges = []
        for i in xrange(self.number_of_readings):
            distance = self.distance(laser_position[0], laser_position[1], obstacle_positions[i,0], obstacle_positions[i,1])
            distance_with_noise = self.noise_standard_deviation * np.random.randn() + distance
            if distance_with_noise < self.scanner_max_range:
                ranges.append(distance_with_noise)
            else:
                ranges.append(self.scanner_max_range)

        laser_scan = LaserScan()
        laser_scan.header.stamp = rospy.Time.now()
        laser_scan.header.frame_id = self.laser_frame
        laser_scan.angle_min = self.scanner_min_angle
        laser_scan.angle_max = self.scanner_max_angle
        laser_scan.angle_increment = self.scanner_angle_increment
        laser_scan.time_increment = self.scanner_time_increment
        laser_scan.range_min = self.scanner_min_range
        laser_scan.range_max = self.scanner_max_range
        laser_scan.ranges = ranges

        self.scan_publisher.publish(laser_scan)

    def distance(self, point1_x, point1_y, point2_x, point2_y):
        return sqrt((point1_x - point2_x) * (point1_x - point2_x) + (point1_y - point2_y) * (point1_y - point2_y))

if __name__ == '__main__':
    rospy.init_node('laser_scan_publisher')

    try:
        node = LaserScanNode()
    except rospy.ROSInterruptException: pass
