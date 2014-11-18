#!/usr/bin/env python
from math import sqrt, cos, sin
import numpy as np

import rospy
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped

from scripts.laser_data import LaserData
from map.srv import PositionsOfClosestObstacle
from fault_injector.msg import InjectFault
from robot.srv import SensorMeasurements, SensorMeasurementsResponse
from robot.msg import ScanAndPose

class LaserScanNode(object):
    '''Simulates laser scans assuming that a robot has two laser scanners.

    Author -- Aleksandar Mitrevski

    '''
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

        #ground truth frame; there should be a path from 'world_frame' to
        #'front/back_sensor_frame' in the tf tree
        self.world_frame = rospy.get_param('~world_frame', '/world')

        #map frame; there should be a path from 'map_frame' to
        #'actual_front/back_sensor_frame' in the tf tree
        self.map_frame = rospy.get_param('~map_frame', '/map')

        #the simulated laser scans are sent in these frames
        self.actual_front_laser_frame = rospy.get_param('~actual_front_sensor_frame', '/laser_front')
        self.actual_back_laser_frame = rospy.get_param('~actual_back_sensor_frame', '/laser_back')

        #frames used in a ground truth representation
        self.front_laser_frame = rospy.get_param('~front_sensor_frame', '/laser_front_copy')
        self.back_laser_frame = rospy.get_param('~back_sensor_frame', '/laser_back_copy')

        self.log_file_directory = rospy.get_param('~log_file_directory', None)

        #indicates whether to inject faults to each of the sensors
        self.inject_front_laser_fault = False
        self.inject_back_laser_fault = False

        self.scan_service = rospy.Service('sensor_measurements', SensorMeasurements, self.return_current_scans)
        self.scan_publisher = rospy.Publisher('laser_scan', LaserScan, queue_size=10)
        self.scan_and_pose_publisher = rospy.Publisher('scan_and_pose', ScanAndPose, queue_size=10)
        rospy.Subscriber('inject_fault', InjectFault, self.inject_fault)

        self.tf_listener = tf.TransformListener()
        self.tf_listener.waitForTransform(self.world_frame, self.front_laser_frame, rospy.Time(0), rospy.Duration(10))
        self.tf_listener.waitForTransform(self.world_frame, self.back_laser_frame, rospy.Time(0), rospy.Duration(10))
        self.tf_listener.waitForTransform(self.map_frame, self.actual_front_laser_frame, rospy.Time(0), rospy.Duration(10))
        self.tf_listener.waitForTransform(self.map_frame, self.actual_back_laser_frame, rospy.Time(0), rospy.Duration(10))

        while not rospy.is_shutdown():
            #as long as the node is running:
            #   - we read laser data given the current pose of the robot and a known map
            #   - we publish the scans
            #   - we log the scans
            front_laser_data, back_laser_data, front_laser_pose, back_laser_pose = self.read_laser_data()
            self.publish_scans(front_laser_data)
            self.publish_scans(back_laser_data)
            self.log_scans(front_laser_data)
            self.log_scans(back_laser_data)
            self.publish_scans_and_pose(front_laser_data, front_laser_pose)
            rospy.sleep(0.1)

    def inject_fault(self, msg):
        '''Sets/resets the values of the flags that indicate whether to inject faults.

        Keyword arguments:
        msg -- An 'InjectFault' message.

        '''
        if msg.frame_id == self.actual_front_laser_frame or '/' + msg.frame_id == self.actual_front_laser_frame:
            if msg.inject_fault:
                self.inject_front_laser_fault = True
            else:
                self.inject_front_laser_fault = False
        elif msg.frame_id == self.actual_back_laser_frame or '/' + msg.frame_id == self.actual_back_laser_frame:
            if msg.inject_fault:
                self.inject_back_laser_fault = True
            else:
                self.inject_back_laser_fault = False

    def return_current_scans(self, request=None):
        '''A service proxy returning the current set of laser measurements.
        '''
        front_laser_data, back_laser_data,_,_ = self.read_laser_data()
        scan_msgs = list()
        front_scan_msg = self.generate_laser_msg(front_laser_data)
        scan_msgs.append(front_scan_msg)

        #back_scan_msg = self.generate_laser_msg(back_laser_data)
        #scan_msgs.append(back_scan_msg)

        response = SensorMeasurementsResponse()
        response.scans = scan_msgs
        response.success = True
        return response

    def read_laser_data(self):
        '''Simulates laser readings given the current pose of a robot.
        '''
        front_laser_data = LaserData(self.actual_front_laser_frame)
        back_laser_data = LaserData(self.actual_back_laser_frame)
        front_laser_data.obstacle_positions = -1. * np.ones((self.number_of_readings, 2))
        back_laser_data.obstacle_positions = -1. * np.ones((self.number_of_readings, 2))

        front_laser_pose = PoseStamped()
        front_laser_pose.header.frame_id = self.actual_front_laser_frame
        back_laser_pose = PoseStamped()
        back_laser_pose.header.frame_id = self.actual_back_laser_frame        

        try:
            (laser_map_translation, laser_map_quat_rotation) = self.tf_listener.lookupTransform(self.map_frame, self.actual_front_laser_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return front_laser_data, back_laser_data, front_laser_pose, back_laser_pose

        front_laser_pose.pose.position.x = laser_map_translation[0]
        front_laser_pose.pose.position.y = laser_map_translation[1]
        front_laser_pose.pose.orientation.x = laser_map_quat_rotation[0]
        front_laser_pose.pose.orientation.y = laser_map_quat_rotation[1]
        front_laser_pose.pose.orientation.z = laser_map_quat_rotation[2]
        front_laser_pose.pose.orientation.w = laser_map_quat_rotation[3]

        try:
            (translation, quat_rotation) = self.tf_listener.lookupTransform(self.world_frame, self.front_laser_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return front_laser_data, back_laser_data, front_laser_pose, back_laser_pose

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

        front_directions_x = list()
        front_directions_y = list()

        back_directions_x = list()
        back_directions_y = list()

        front_angle = front_laser_data.heading - self.scanner_min_angle
        back_angle = back_laser_data.heading - self.scanner_min_angle
        for i in xrange(self.number_of_readings):
            front_directions_x.append(cos(self.normalise_angle(front_angle)))
            front_directions_y.append(sin(self.normalise_angle(front_angle)))

            back_directions_x.append(cos(self.normalise_angle(back_angle)))
            back_directions_y.append(sin(self.normalise_angle(back_angle)))

            front_angle = front_angle - self.scanner_angle_increment
            back_angle = back_angle - self.scanner_angle_increment

        rospy.wait_for_service('positions_of_closest_obstacle')
        try:
            proxy = rospy.ServiceProxy('positions_of_closest_obstacle', PositionsOfClosestObstacle)
            result = proxy(front_laser_data.laser_position[0], front_laser_data.laser_position[1], front_directions_x, front_directions_y)
            if result.success:
                for i in xrange(self.number_of_readings):
                    if not self.inject_front_laser_fault:
                        front_laser_data.obstacle_positions[i,0] = result.position_x[i]
                        front_laser_data.obstacle_positions[i,1] = result.position_y[i]
                    else:
                        front_laser_data.obstacle_positions[i,0] = front_laser_data.laser_position[0]
                        front_laser_data.obstacle_positions[i,1] = front_laser_data.laser_position[1]
            else:
                for i in xrange(self.number_of_readings):
                    front_laser_data.obstacle_positions[i,0] = 2 * front_laser_data.laser_position[0] + self.scanner_max_range
                    front_laser_data.obstacle_positions[i,1] = 2 * front_laser_data.laser_position[1] + self.scanner_max_range
        except rospy.ServiceException, e:
            rospy.logerr('positions_of_closest_obstacle service call failed')

        rospy.wait_for_service('positions_of_closest_obstacle')
        try:
            proxy = rospy.ServiceProxy('positions_of_closest_obstacle', PositionsOfClosestObstacle)
            result = proxy(back_laser_data.laser_position[0], back_laser_data.laser_position[1], back_directions_x, back_directions_y)
            if result.success:
                for i in xrange(self.number_of_readings):
                    if not self.inject_back_laser_fault:
                        back_laser_data.obstacle_positions[i,0] = result.position_x[i]
                        back_laser_data.obstacle_positions[i,1] = result.position_y[i]
                    else:
                        back_laser_data.obstacle_positions[i,0] = back_laser_data.laser_position[0]
                        back_laser_data.obstacle_positions[i,1] = back_laser_data.laser_position[1]
            else:
                for i in xrange(self.number_of_readings):
                    back_laser_data.obstacle_positions[i,0] = 2 * back_laser_data.laser_position[0] + self.scanner_max_range
                    back_laser_data.obstacle_positions[i,1] = 2 * back_laser_data.laser_position[1] + self.scanner_max_range
        except rospy.ServiceException, e:
            rospy.logerr('positions_of_closest_obstacle service call failed')

        return front_laser_data, back_laser_data, front_laser_pose, back_laser_pose

    def publish_scans(self, laser_data):
        '''Publishes the data in 'laser_data'.

        Keyword arguments:
        laser_data -- A 'LaserData' object.

        '''
        laser_scan_msg = self.generate_laser_msg(laser_data)
        self.scan_publisher.publish(laser_scan_msg)

    def log_scans(self, laser_data):
        '''Saves the data in 'laser_data' into a log file
        with the name of the frame associated with the data
        and inside 'self.log_file_directory'.

        Keyword arguments:
        laser_data -- A 'LaserData' object.

        '''
        ranges = self.calculate_ranges(laser_data)
        file_handle = open(self.log_file_directory + '/' + laser_data.frame_id + '.log', 'a')
        for i in xrange(len(ranges)-1):
            file_handle.write(str(ranges[i]) + ' ')
        file_handle.write(str(ranges[len(ranges)-1]) + '\n')

    def publish_scans_and_pose(self, laser_data, pose):
        '''Publishes 'laser_data', making the data synchronised with 'pose'.

        Keyword arguments:
        laser_data -- A 'LaserData' object.
        pose -- A 'PoseStamped' message.

        '''
        msg = self.generate_scan_and_pose_msg(laser_data, pose)
        self.scan_and_pose_publisher.publish(msg)

    def generate_scan_and_pose_msg(self, laser_data, laser_pose):
        '''Generates a 'ScanAndPose' message from the input data.

        Keyword arguments:
        laser_data -- A 'LaserData' object.
        pose -- A 'PoseStamped' message.

        Returns:
        msg -- A 'ScanAndPose' message.

        '''
        msg = ScanAndPose()
        msg.scan = self.generate_laser_msg(laser_data)
        laser_pose.header.stamp = msg.scan.header.stamp
        msg.pose = laser_pose
        return msg

    def generate_laser_msg(self, laser_data):
        '''Generates a 'LaserScan' message from the input data.

        Keyword arguments:
        laser_data -- A 'LaserData' object.

        Returns:
        msg -- A 'LaserScan' message.

        '''
        laser_scan_msg = LaserScan()
        laser_scan_msg.header.stamp = rospy.Time.now()
        laser_scan_msg.header.frame_id = laser_data.frame_id
        laser_scan_msg.angle_min = self.scanner_min_angle
        laser_scan_msg.angle_max = self.scanner_max_angle
        laser_scan_msg.angle_increment = self.scanner_angle_increment
        laser_scan_msg.time_increment = self.scanner_time_increment
        laser_scan_msg.range_min = self.scanner_min_range
        laser_scan_msg.range_max = self.scanner_max_range
        laser_scan_msg.ranges = self.calculate_ranges(laser_data)

        return laser_scan_msg

    def calculate_ranges(self, laser_data):
        '''Calculates the distances from the laser to the obstacles in its view.

        Keyword arguments:
        laser_data -- A 'LaserData' object.

        '''
        ranges = []
        for i in xrange(self.number_of_readings):
            distance = self.distance(laser_data.laser_position[0], laser_data.laser_position[1], laser_data.obstacle_positions[i,0], laser_data.obstacle_positions[i,1])
            distance_with_noise = self.noise_standard_deviation * np.random.randn() + distance
            if distance_with_noise < self.scanner_max_range:
                ranges.append(distance_with_noise)
            else:
                ranges.append(self.scanner_max_range)
        return ranges

    def distance(self, point1_x, point1_y, point2_x, point2_y):
        '''Calculates the distance between a point with coordinates (point1_x, point1_y)
        and a point with coordinates (point2_x, point2_y).
        '''
        return sqrt((point1_x - point2_x) * (point1_x - point2_x) + (point1_y - point2_y) * (point1_y - point2_y))

    def normalise_angle(self, angle):
        '''Converts the input angle to the range (0,2*pi).
        '''
        if angle < 0.:
            angle = angle + 2 * 3.14
        return angle

if __name__ == '__main__':
    rospy.init_node('laser_scan_publisher')

    try:
        node = LaserScanNode()
    except rospy.ROSInterruptException: pass
