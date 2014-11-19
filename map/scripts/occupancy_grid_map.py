import numpy as np
from coordinates import Coordinates

import Image

class OccupancyGridMap(object):
    '''Defines a structure for storing an occupancy grid map.

    Author -- Aleksandar Mitrevski

    '''
    def __init__(self, width=10., height=10., resolution=0.1, map_image_file_name=None):
        '''Initialises an occupancy grid map with x coordinates in the range
        (-width/2,width/2) and y coordinates in the range (-height/2,height/2).

        Keyword arguments:
        width -- Map width in meters (default 10.).
        height -- Map height in meters (default 10.).
        resolution -- Map resolution in meters (default 0.1).
        map_image_file -- Name of a grayscale image that contains a map (default None, in which case an empty map with unknown occupancy values is initialised).

        '''
        self.width = width
        self.height = height
        self.rows = int(round(height / resolution))
        self.columns = int(round(width / resolution))
        self.resolution = resolution
        self.occupancy_grid = 50. * np.ones((self.rows, self.columns))

        if map_image_file_name != None:
            self._read_map(map_image_file_name)

        self.x_boundaries = (-width/2., width/2.)
        self.y_boundaries = (-height/2., height/2.)

    def get_map(self):
        return np.array(self.occupancy_grid)

    def get_map_origin(self):
        '''Returns a 'Coordinates' object representing the world coordinates of the (0,0) cell.
        '''
        return Coordinates(self.x_boundaries[0] + self.resolution/2., self.y_boundaries[0] + self.resolution / 2.)

    def update_occupancy_values(self, occupancy_values):
        '''Updates the occupancy values of the map.

        Keyword arguments:
        occupancy_values -- A list of size 'self.rows' * 'self.columns' containing the new occupancy values.

        '''
        values = np.array(occupancy_values).reshape((self.rows, self.columns))
        for i in xrange(self.rows):
            for j in xrange(self.columns):
                self.occupancy_grid[i,j] = values[i,j]

    def find_closest_obstacle(self, position, direction):
        '''Finds the coordinates of the obstacle closest to a range scanner at the given position and direction.

        Keyword arguments:
        position -- A 'Coordinates' object representing the position of a range sensor.
        direction -- A 'Coordinates' object representing the direction vector of the sensor's ray.

        Returns:
        obstacle_position -- A 'Coordinates' object representing the map coordinates of the closest obstacle.
                             The coordinates (width,height) are returned if the measurement falls outside the map.
        position_inside_map -- True if the closest obstacle is inside the map and False otherwise.

        '''
        t = 0.
        t_increment = self.resolution
        point_position = Coordinates(position.x, position.y)
        position_inside_map = True
        obstacle_position = Coordinates(self.width, self.height)

        while position_inside_map:
            try:
                map_coordinates = self.world_to_map_coordinates(point_position.x, point_position.y)
                if self.occupancy_grid[map_coordinates.x, map_coordinates.y] > 95:
                    obstacle_position = point_position
                    break
                t = t + t_increment
                point_position = position + direction.multiply(t)
            except ValueError:
                position_inside_map = False

        return obstacle_position, position_inside_map

    def find_ray_traced_cells(self, position, direction, distance):
        '''Finds the coordinates of the cells traced by a range scanner at the given position
        and direction, given a measurement of a certain distance.

        Keyword arguments:
        position -- A 'Coordinates' object representing the position of a range sensor.
        direction -- A 'Coordinates' object representing the direction vector of the sensor's ray.
        distance -- The measured distance by the sensor (in meters).

        Returns:
        included_cells_map_coordinates -- Map coordinates of the traced cells.
        included_cells_world_coordinates -- World coordinates of the traced cells.

        '''
        t = 0.
        t_increment = self.resolution / 2.
        point_position = Coordinates(position.x, position.y)
        tracing_over = False
        included_cells_map_coordinates = list()
        included_cells_world_coordinates = list()

        while not tracing_over:
            try:
                map_coordinates = self.world_to_map_coordinates(point_position.x, point_position.y)
                cell_world_coordinates = self.map_to_world_coordinates(map_coordinates.x, map_coordinates.y)
                if position.distance(cell_world_coordinates) > distance:
                    tracing_over = True

                if map_coordinates not in included_cells_map_coordinates and not tracing_over:
                    included_cells_map_coordinates.append(map_coordinates)
                    included_cells_world_coordinates.append(cell_world_coordinates)

                t = t + t_increment
                point_position = position + direction.multiply(t)
            except ValueError:
                tracing_over = True

        return included_cells_map_coordinates, included_cells_world_coordinates

    def map_to_world_coordinates(self, row_index, column_index):
        '''Converts the given map coordinates to world coordinates
        by finding the center of the appropriate map cell.
        Raises a ValueError if the input coordinates are out of the grid.

        Returns:
        coordinates -- A 'Coordinates' object with the coordinates of the cell's center.

        '''
        if self.invalid_indices(row_index, column_index):
            raise ValueError('OccupancyGridMap: Invalid coordinates')
        x = (self.x_boundaries[0] + self.resolution / 2.) + column_index * self.resolution
        y = (self.y_boundaries[0] + self.resolution / 2.) + row_index * self.resolution
        return Coordinates(x, y)

    def world_to_map_coordinates(self, x, y):
        '''Converts the given world coordinates to map coordinates.
        Raises a ValueError if the input coordinates are out of the map.

        Returns:
        coordinates -- A 'Coordinates' object with the coordinates of the appropriate cell.

        '''
        if self.invalid_coordinates(x, y):
            raise ValueError('OccupancyGridMap: Invalid coordinates')
        column = int((x - self.x_boundaries[0]) / self.resolution)
        row = int((y - self.y_boundaries[0]) / self.resolution)
        return Coordinates(row, column)

    def invalid_indices(self, row_index, column_index):
        '''Checks if the provided cell coordinates are outside the grid.

        Returns:
        True if the coordinates are outside the grid and False otherwise.

        '''
        return row_index < 0 or row_index >= self.rows or column_index < 0 or column_index >= self.columns

    def invalid_coordinates(self, x, y):
        '''Checks if the provided world coordinates are outside the grid.

        Returns:
        True if the coordinates are outside the grid and False otherwise.

        '''
        return x < self.x_boundaries[0] or x > self.x_boundaries[1] or y < self.y_boundaries[0] or y > self.y_boundaries[1]

    def _read_map(self, map_image_file_name):
        '''Reads a map from an image file and saves the values in 'self.occupancy_grid'.
        Raises an IOError if the given file cannot be read.

        Keyword arguments:
        map_image_file_name -- Name of a grayscale image representing a map.

        '''
        map_size = self.occupancy_grid.shape
        try:
            image = Image.open(map_image_file_name).convert('L')
            pixels = image.resize(map_size).load()
            for i in xrange(map_size[0]):
                for j in xrange(map_size[1]):
                    self.occupancy_grid[i,j] = 100. - pixels[i,j] / 255. * 100
        except IOError:
            raise
