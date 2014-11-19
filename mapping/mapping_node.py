#!/usr/bin/env python
import numpy as np
from math import cos, sin

import rospy
import tf
from sensor_msgs.msg import LaserScan

from scripts.mapper import Mapper
from scripts.mapping_parameters import MappingParameters
from scripts.coordinates import Coordinates
from map.msg import CellCoordinateArray, OccupancyGridFloat
from map.srv import GetMap, RayTracedCells
from robot.msg import ScanAndPose

class MappingNode(object):
    '''Defines a node that takes care of updating and publishing a map.

    Author -- Aleksandar Mitrevski

    '''
    def __init__(self):
        self.map_frame = rospy.get_param('map_frame', '/map')

        #an estimate of a usual obstacle thickness (in meters)
        self.obstacle_thickness = float(rospy.get_param('~obstacle_thickness', '0.5'))

        #increments used for updating the occupancy confidence of individual cells
        self.log_occ_prob_increment = float(rospy.get_param('~log_occ_prob_increment', '0.040005'))
        self.log_free_prob_increment = float(rospy.get_param('~log_free_prob_increment', '-0.040005'))

        self.tf_listener = tf.TransformListener()

        rospy.Subscriber('scan_and_pose', ScanAndPose, self.update_map)
        self.map_values_update_publisher = rospy.Publisher('occupancy_grid_update', OccupancyGridFloat, queue_size=10)
        self.mapper = Mapper(MappingParameters(self.obstacle_thickness, self.log_occ_prob_increment, self.log_free_prob_increment))

    def update_map(self, scan_and_pose):
        '''Updates the occupancy values of the map given the scans in 'scan_and_pose'.
        Called whenever a 'ScanAndPose' message is received.

        Keyword arguments:
        scan_and_pose -- A 'ScanAndPose' message.

        '''
        #################################################################
        #commented out because the scans and poses were not synchronised
        #################################################################
        #self.tf_listener.waitForTransform(self.map_frame, scans.header.frame_id, scans.header.stamp, rospy.Duration(10))
        #try:
        #    (translation, quat_rotation) = self.tf_listener.lookupTransform(self.map_frame, scans.header.frame_id, scans.header.stamp)
        #except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #    return

        laser_position = Coordinates(scan_and_pose.pose.pose.position.x, scan_and_pose.pose.pose.position.y)
        laser_quaternion = [scan_and_pose.pose.pose.orientation.x, scan_and_pose.pose.pose.orientation.y, scan_and_pose.pose.pose.orientation.z, scan_and_pose.pose.pose.orientation.w]
        laser_euler_rotation = tf.transformations.euler_from_quaternion(laser_quaternion)

        directions_x = list()
        directions_y = list()
        angle = laser_euler_rotation[2] - scan_and_pose.scan.angle_min
        number_of_readings = int((scan_and_pose.scan.angle_max - scan_and_pose.scan.angle_min) / scan_and_pose.scan.angle_increment) + 1
        for i in xrange(number_of_readings):
            direction_x = cos(self.normalise_angle(angle))
            directions_x.append(direction_x)
            direction_y = sin(self.normalise_angle(angle))
            directions_y.append(direction_y)
            angle = angle - scan_and_pose.scan.angle_increment

        occupancy_grid = None
        rospy.wait_for_service('get_map')
        try:
            proxy = rospy.ServiceProxy('get_map', GetMap)
            result = proxy()
            occupancy_grid = np.array(result.occupancy_grid.data).reshape((result.occupancy_grid.width, result.occupancy_grid.height))
        except rospy.ServiceException:
            rospy.logerr('get_map service call failed')
            return

        rospy.wait_for_service('ray_traced_cells')
        try:
            proxy = rospy.ServiceProxy('ray_traced_cells', RayTracedCells)
            result = proxy(laser_position.x, laser_position.y, directions_x, directions_y, scan_and_pose.scan.ranges)

            world_coordinates = list()
            map_coordinates = list()
            for i in xrange(number_of_readings):
                number_of_cells = len(result.cell_coordinates[i].world_coordinates)
                for j in xrange(number_of_cells):
                    world_coordinates.append(Coordinates(result.cell_coordinates[i].world_coordinates[j].x, result.cell_coordinates[i].world_coordinates[j].y))
                    map_coordinates.append(Coordinates(result.cell_coordinates[i].map_coordinates[j].x, result.cell_coordinates[i].map_coordinates[j].y))
                    occupancy_grid = self.mapper.update_map(occupancy_grid, laser_position, scan_and_pose.scan.range_max, world_coordinates, map_coordinates, scan_and_pose.scan.ranges[i])

            self.publish_updated_map_values(occupancy_grid)
        except rospy.ServiceException:
            rospy.logerr('ray_traced_cells service call failed')
            return

    def publish_updated_map_values(self, occupancy_grid):
        '''Publishes the current version of the occupancy grid.
        '''
        occupancy_grid_msg = OccupancyGridFloat()
        occupancy_grid_msg.data = list(occupancy_grid.flatten())

        self.map_values_update_publisher.publish(occupancy_grid_msg)

    def normalise_angle(self, angle):
        '''Converts 'angle' to the range [0,2*pi).
        '''
        if angle < 0.:
            angle = angle + 2 * 3.14
        return angle

if __name__ == '__main__':
    rospy.init_node('mapping')

    try:
        node = MappingNode()
        rospy.spin()
    except rospy.ROSInterruptException: pass
