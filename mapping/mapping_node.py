#!/usr/bin/env python
import numpy as np

import rospy
import tf
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

from scripts.mapper import Mapper
from scripts.mapping_parameters import MappingParameters
from scripts.coordinates import Coordinates
from map.msg import CellCoordinateArray
from map.srv import GetMap, RayTracedCells

class MappingNode(object):
    def __init__(self):
        self.map_frame = rospy.get_param('map_frame', '/map')
        self.obstacle_thickness = float(rospy.get_param('~obstacle_thickness', '0.5'))
        self.occ_prob_increment = float(rospy.get_param('~occ_prob_increment', '0.001'))
        self.free_prob_increment = float(rospy.get_param('~free_prob_increment', '0.001'))

        self.tf_listener = tf.TransformListener()

        rospy.Subscriber('laser_scan', LaserScan, self.update_map)
        self.map_values_update_publisher = rospy.Publisher('occupancy_grid_update', OccupancyGrid, queue_size=10)
        self.mapper = Mapper(MappingParameters(self.obstacle_thickness, self.occ_prob_increment, self.free_prob_increment))

    def update_map(self, scans):
        occupancy_grid = None
        rospy.wait_for_service('get_map')
        try:
            proxy = rospy.ServiceProxy('get_map', GetMap)
            result = proxy()
            occupancy_grid = np.array(result.occupancy_grid.data).reshape((result.occupancy_grid.width, result.occupancy_grid.height))
        except rospy.ServiceException:
            rospy.logerr('get_map service call failed')
            return

        self.tf_listener.waitForTransform(self.map_frame, scans.header.frame_id, rospy.Time(0), rospy.Duration(10))
        try:
            (translation, quat_rotation) = self.tf_listener.lookupTransform(self.map_frame, scans.header.frame_id, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        laser_position = Coordinates(translation[0], translation[1])

        directions_x = list()
        directions_y = list()
        angle = laser_euler_rotation[2] - scans.angle_min
        number_of_readings = int((scans.angle_max - scans.angle_min) / scans.angle_increment) + 1
        for i in xrange(number_of_readings):
            direction_x = cos(angle)
            directions_x.append(direction_x)
            direction_y = sin(angle)
            directions_y.append(direction_y)
            angle = angle - scans.angle_increment

        rospy.wait_for_service('ray_traced_cells')
        try:
            proxy = rospy.ServiceProxy('ray_traced_cells')
            result = proxy(laser_position.x, laser_position.y, directions_x, directions_y, scans.ranges)

            world_coordinates = list()
            map_coordinates = list()
            number_of_cells = len(result.cell_coordinates.world_coordinates)
            for i in xrange(number_of_cells):
                world_coordinates.append(Coordinates(result.cell_coordinates.world_coordinates[i].x, result.cell_coordinates.world_coordinates[i].y))
                map_coordinates.append(Coordinates(result.cell_coordinates.map_coordinates[i].x, result.cell_coordinates.map_coordinates[i].y))

            occupancy_grid = self.mapper.update_map(occupancy_grid, laser_position, scans.range_max, world_coordinates, map_coordinate, scans.ranges)
        except rospy.ServiceException:
            rospy.logerr('ray_traced_cells service call failed')
            return

    def publish_updated_map_values(self, occupancy_grid):
        occupancy_grid_msg = OccupancyGrid()
        occupancy_grid_msg.header.stamp = rospy.Time.now()
        occupancy_grid_msg.data = list(occupancy_grid.flatten().astype(int))

        self.self.map_values_update_publisher.publish(occupancy_grid_msg)

if __name__ == '__main__':
    rospy.init_node('mapping')

    try:
        node = MappingNode()
        rospy.spin()
    except rospy.ROSInterruptException: pass
