#!/usr/bin/env python
from math import sqrt, cos, sin
import numpy as np

import rospy
import tf
from sensor_msgs.msg import LaserScan

from scripts.laser_data import LaserData
from map.srv import PositionOfClosestObstacle
from robot.srv import SensorMeasurements, SensorMeasurementsResponse

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
        self.world_frame = rospy.get_param('~world_frame', '/world')
        self.actual_front_laser_frame = rospy.get_param('~front_sensor_frame', '/laser_front')
        self.actual_back_laser_frame = rospy.get_param('~back_sensor_frame', '/laser_back')
        self.front_laser_frame = rospy.get_param('~front_sensor_frame', '/laser_front_copy')
        self.back_laser_frame = rospy.get_param('~back_sensor_frame', '/laser_back_copy')

        self.scan_service = rospy.Service('sensor_measurements', SensorMeasurements, self.return_current_scans)
        self.scan_publisher = rospy.Publisher('laser_scan', LaserScan, queue_size=10)

        self.tf_listener = tf.TransformListener()
        self.tf_listener.waitForTransform(self.world_frame, self.front_laser_frame, rospy.Time(0), rospy.Duration(10))
        self.tf_listener.waitForTransform(self.world_frame, self.back_laser_frame, rospy.Time(0), rospy.Duration(10))

        while not rospy.is_shutdown():
            front_laser_data, back_laser_data = self.read_laser_data()
            self.publish_scans(front_laser_data)
            self.publish_scans(back_laser_data)

    def return_current_scans(self, request=None):
        front_laser_data, back_laser_data = self.read_laser_data()
        scan_msgs = list()
        front_scan_msg = self.generate_laser_msg(front_laser_data)
        scan_msgs.append(front_scan_msg)

        back_scan_msg = self.generate_laser_msg(back_laser_data)
        scan_msgs.append(back_scan_msg)

        response = SensorMeasurementsResponse()
        response.scans = scan_msgs
        response.success = True
        return response

    def read_laser_data(self):
        front_laser_data = LaserData(self.actual_front_laser_frame)
        back_laser_data = LaserData(self.actual_back_laser_frame)
        front_laser_data.obstacle_positions = -1. * np.ones((self.number_of_readings, 2))
        back_laser_data.obstacle_positions = -1. * np.ones((self.number_of_readings, 2))

        try:
            (translation, quat_rotation) = self.tf_listener.lookupTransform(self.world_frame, self.front_laser_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return front_laser_data, back_laser_data

        front_laser_data.laser_position[0] = translation[0]
        front_laser_data.laser_position[1] = translation[1]
        euler_rotation = tf.transformations.euler_from_quaternion(quat_rotation)
        front_laser_data.heading = euler_rotation[2]

        try:
            (translation, quat_rotation) = self.tf_listener.lookupTransform(self.world_frame, self.back_laser_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return front_laser_data, back_laser_data

        back_laser_data.laser_position[0] = translation[0]
        back_laser_data.laser_position[1] = translation[1]
        euler_rotation = tf.transformations.euler_from_quaternion(quat_rotation)
        back_laser_data.heading = euler_rotation[2]

        front_angle = front_laser_data.heading - self.scanner_min_angle
        back_angle = back_laser_data.heading - self.scanner_min_angle
        for i in xrange(self.number_of_readings):
            front_direction_x = cos(front_angle)
            front_direction_y = sin(front_angle)

            back_direction_x = cos(back_angle)
            back_direction_y = sin(back_angle)

            rospy.wait_for_service('position_of_closest_obstacle')
            try:
                proxy = rospy.ServiceProxy('position_of_closest_obstacle', PositionOfClosestObstacle)
                result = proxy(front_laser_data.laser_position[0], front_laser_data.laser_position[1], front_direction_x, front_direction_y)
                if result.success == 'success':
                    front_laser_data.obstacle_positions[i,0] = result.position_x
                    front_laser_data.obstacle_positions[i,1] = result.position_y
                else:
                    front_laser_data.obstacle_positions[i,0] = 2 * front_laser_data.laser_position[0] + self.scanner_max_range
                    front_laser_data.obstacle_positions[i,1] = 2 * front_laser_data.laser_position[1] + self.scanner_max_range
            except rospy.ServiceException, e:
                rospy.logerr('position_of_closest_obstacle service call failed')

            rospy.wait_for_service('position_of_closest_obstacle')
            try:
                proxy = rospy.ServiceProxy('position_of_closest_obstacle', PositionOfClosestObstacle)
                result = proxy(back_laser_data.laser_position[0], back_laser_data.laser_position[1], back_direction_x, back_direction_y)
                if result.success:
                    back_laser_data.obstacle_positions[i,0] = result.position_x
                    back_laser_data.obstacle_positions[i,1] = result.position_y
                else:
                    back_laser_data.obstacle_positions[i,0] = 2 * back_laser_data.laser_position[0] + self.scanner_max_range
                    back_laser_data.obstacle_positions[i,1] = 2 * back_laser_data.laser_position[1] + self.scanner_max_range
            except rospy.ServiceException, e:
                rospy.logerr('position_of_closest_obstacle service call failed')

            front_angle = front_angle - self.scanner_angle_increment
            back_angle = back_angle - self.scanner_angle_increment

        return front_laser_data, back_laser_data

    def publish_scans(self, laser_data):
        laser_scan_msg = self.generate_laser_msg(laser_data)
        self.scan_publisher.publish(laser_scan_msg)

    def generate_laser_msg(self, laser_data):
        ranges = []
        for i in xrange(self.number_of_readings):
            distance = self.distance(laser_data.laser_position[0], laser_data.laser_position[1], laser_data.obstacle_positions[i,0], laser_data.obstacle_positions[i,1])
            distance_with_noise = self.noise_standard_deviation * np.random.randn() + distance
            if distance_with_noise < self.scanner_max_range:
                ranges.append(distance_with_noise)
            else:
                ranges.append(self.scanner_max_range)

        laser_scan_msg = LaserScan()
        laser_scan_msg.header.stamp = rospy.Time.now()
        laser_scan_msg.header.frame_id = laser_data.frame_id
        laser_scan_msg.angle_min = self.scanner_min_angle
        laser_scan_msg.angle_max = self.scanner_max_angle
        laser_scan_msg.angle_increment = self.scanner_angle_increment
        laser_scan_msg.time_increment = self.scanner_time_increment
        laser_scan_msg.range_min = self.scanner_min_range
        laser_scan_msg.range_max = self.scanner_max_range
        laser_scan_msg.ranges = ranges

        return laser_scan_msg

    def distance(self, point1_x, point1_y, point2_x, point2_y):
        return sqrt((point1_x - point2_x) * (point1_x - point2_x) + (point1_y - point2_y) * (point1_y - point2_y))

if __name__ == '__main__':
    rospy.init_node('laser_scan_publisher')

    try:
        node = LaserScanNode()
    except rospy.ROSInterruptException: pass
