#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import OccupancyGrid

from scripts.coordinates import Coordinates
from scripts.occupancy_grid_map import OccupancyGridMap

class MapNode(object):
    def __init__(self):
        self.map_width = float(rospy.get_param('map_width', '10.0'))
        self.map_height = float(rospy.get_param('map_height', '10.0'))
        self.map_resolution = float(rospy.get_param('map_resolution', '0.1'))
        self.map_image_file_name = rospy.get_param('map_image_file_name', None)

        self.occupancy_grid = OccupancyGridMap(self.map_image_file_name, self.map_width, self.map_height, self.map_resolution)
        self.map_publisher = rospy.Publisher('nav_msgs/occupancy_grid', OccupancyGrid, queue_size=5)

        while not rospy.is_shutdown():
            self.publish_map()

    def publish_map(self):
        occupancy_grid_message = OccupancyGrid()
        occupancy_grid_message.header.stamp = rospy.Time.now()
        occupancy_grid_message.header.frame_id = 'map'

        occupancy_grid_message.info.width = self.occupancy_grid.columns
        occupancy_grid_message.info.height = self.occupancy_grid.rows
        occupancy_grid_message.info.resolution = self.occupancy_grid.resolution

        map_origin = self.occupancy_grid.get_map_origin()
        origin_pose = Pose(Point(map_origin.x, map_origin.y, 0.), Quaternion(0., 0., 0., 1.))
        occupancy_grid_message.info.origin = origin_pose

        occupancy_grid_message.data = list(self.occupancy_grid.get_map().flatten().astype(int))

        self.map_publisher.publish(occupancy_grid_message)

if __name__ == '__main__':
    rospy.init_node('map')
    try:
        MapNode()
    except rospy.ROSInterruptException: pass
