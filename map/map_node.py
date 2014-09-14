#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import OccupancyGrid

from scripts.coordinates import Coordinates
from scripts.occupancy_grid_map import OccupancyGridMap

from map.srv import *
from map.msg import CellCoordinateArray

class MapNode(object):
    def __init__(self):
        self.map_frame = rospy.get_param('map_frame', '/map')
        self.map_width = float(rospy.get_param('map_width', '10.0'))
        self.map_height = float(rospy.get_param('map_height', '10.0'))
        self.map_resolution = float(rospy.get_param('map_resolution', '0.1'))
        self.ground_truth_map_image_file_name = rospy.get_param('ground_truth_map_image_file_name', None)
        self.map_image_file_name = rospy.get_param('map_image_file_name', None)

        self.ground_truth_occupancy_grid = OccupancyGridMap(self.map_width, self.map_height, self.map_resolution, self.ground_truth_map_image_file_name)
        self.occupancy_grid = OccupancyGridMap(self.map_width, self.map_height, self.map_resolution, self.map_image_file_name)

        self.map_publisher = rospy.Publisher('occupancy_grid', OccupancyGrid, queue_size=5)
        self.rviz_map_publisher = rospy.Publisher('nav_msgs/occupancy_grid', OccupancyGrid, queue_size=5)
        self.map_service = rospy.Service('get_map', GetMap, self.get_map)
        self.ray_traced_cells_service = rospy.Service('ray_traced_cells', RayTracedCells, self.get_ray_traced_cells)
        rospy.Subscriber('occupancy_grid_update', OccupancyGrid, self.update_occupancy_values)
        self.closest_obstacle_service = rospy.Service('position_of_closest_obstacle', PositionOfClosestObstacle, self.find_position_of_closest_obstacle)
        self.closest_obstacles_service = rospy.Service('positions_of_closest_obstacle', PositionsOfClosestObstacle, self.find_positions_of_closest_obstacle)

        while not rospy.is_shutdown():
            self.publish_map()
            self.publish_rviz_map()

    def publish_map(self):
        occupancy_grid_msg = OccupancyGrid()
        occupancy_grid_msg.header.stamp = rospy.Time.now()
        occupancy_grid_msg.header.frame_id = self.map_frame

        occupancy_grid_msg.info.width = self.occupancy_grid.columns
        occupancy_grid_msg.info.height = self.occupancy_grid.rows
        occupancy_grid_msg.info.resolution = self.occupancy_grid.resolution

        map_origin = self.occupancy_grid.get_map_origin()
        origin_pose = Pose(Point(map_origin.x, map_origin.y, 0.), Quaternion(0., 0., 0., 1.))
        occupancy_grid_msg.info.origin = origin_pose
        occupancy_grid_msg.data = list(self.occupancy_grid.get_map().flatten().astype(int))

        self.map_publisher.publish(occupancy_grid_msg)

    def publish_rviz_map(self):
        occupancy_grid_msg = OccupancyGrid()
        occupancy_grid_msg.header.stamp = rospy.Time.now()
        occupancy_grid_msg.header.frame_id = self.map_frame

        occupancy_grid_msg.info.width = self.occupancy_grid.columns
        occupancy_grid_msg.info.height = self.occupancy_grid.rows
        occupancy_grid_msg.info.resolution = self.occupancy_grid.resolution

        map_origin = self.occupancy_grid.get_map_origin()
        origin_pose = Pose(Point(map_origin.x, map_origin.y, 0.), Quaternion(0., 0., 0., 1.))
        occupancy_grid_msg.info.origin = origin_pose

        map_values = list(self.occupancy_grid.get_map().flatten().astype(int))
        for i in xrange(len(map_values)):
            if abs(map_values[i] - 50) < 5:
                map_values[i] = -1
        occupancy_grid_msg.data = map_values

        self.rviz_map_publisher.publish(occupancy_grid_msg)

    def get_map(self):
        occupancy_grid_msg = OccupancyGrid()
        occupancy_grid_msg.header.stamp = rospy.Time.now()
        occupancy_grid_msg.header.frame_id = self.map_frame

        occupancy_grid_msg.info.width = self.occupancy_grid.columns
        occupancy_grid_msg.info.height = self.occupancy_grid.rows
        occupancy_grid_msg.info.resolution = self.occupancy_grid.resolution

        map_origin = self.occupancy_grid.get_map_origin()
        origin_pose = Pose(Point(map_origin.x, map_origin.y, 0.), Quaternion(0., 0., 0., 1.))
        occupancy_grid_msg.info.origin = origin_pose

        occupancy_grid_msg.data = list(self.occupancy_grid.get_map().flatten().astype(int))
        return GetMapResponse(occupancy_grid_msg)

    def find_position_of_closest_obstacle(self, request):
        position = Coordinates(request.position_x, request.position_y)
        direction = Coordinates(request.direction_vector_x, request.direction_vector_y)
        obstacle_position, position_inside_map = self.ground_truth_occupancy_grid.find_closest_obstacle(position, direction)
        if position_inside_map:
            return PositionOfClosestObstacleResponse(True, obstacle_position.x, obstacle_position.y)
        else:
            return PositionOfClosestObstacleResponse(False, 0., 0.)

    def find_positions_of_closest_obstacle(self, request):
        number_of_directions = len(request.direction_vector_x)

        success = list()
        obstacle_positions_x = list()
        obstacle_positions_y = list()
        for i in xrange(number_of_directions):
            position = Coordinates(request.position_x, request.position_y)
            direction = Coordinates(request.direction_vector_x[i], request.direction_vector_y[i])
            obstacle_position, position_inside_map = self.occupancy_grid.find_closest_obstacle(position, direction)

            if position_inside_map:
                obstacle_positions_x.append(obstacle_position.x)
                obstacle_positions_y.append(obstacle_position.y)
                success.append(True)
            else:
                obstacle_positions_x.append(0)
                obstacle_positions_y.append(0)
                success.append(False)

        return PositionsOfClosestObstacleResponse(success, obstacle_positions_x, obstacle_positions_y)

    def update_occupancy_values(self, grid_msg):
        occupancy_values = list(grid_msg.data)
        self.occupancy_grid.update_occupancy_values(occupancy_values)

    def get_ray_traced_cells(self, request):
        number_of_directions = len(request.direction_vector_x)

        ray_coordinates = list()
        for i in xrange(number_of_directions):
            position = Coordinates(request.position_x, request.position_y)
            direction = Coordinates(request.direction_vector_x[i], request.direction_vector_y[i])
            included_cells_map_coordinates, included_cells_world_coordinates = self.occupancy_grid.find_ray_traced_cells(position, direction, request.ranges[i])

            coordinate_array = CellCoordinateArray()
            number_of_cells = len(included_cells_map_coordinates)
            for i in xrange(number_of_cells):
                coordinate_array.map_coordinates.append(Point(included_cells_map_coordinates[i].x, included_cells_map_coordinates[i].y, 0))
                coordinate_array.world_coordinates.append(Point(included_cells_world_coordinates[i].x, included_cells_world_coordinates[i].y, 0))
            ray_coordinates.append(coordinate_array)

        return RayTracedCellsResponse(ray_coordinates)

if __name__ == '__main__':
    rospy.init_node('map')
    try:
        MapNode()
    except rospy.ROSInterruptException: pass
