import numpy as np
from coordinates import Coordinates

class OccupancyGrid(object):
    def __init__(self, width=10., height=10., resolution=0.1):
        '''Initialises an occupancy grid map with x coordinates in the range
        (-width/2,width/2) and y coordinates in the range (-height/2,height/2).

        Keyword arguments:
        width -- Map width in meters (default 10.)
        height -- Map height in meters (default 10.)
        resolution -- Map resolution in meters (default 0.1)

        '''
        self.rows = int(round(height / resolution))
        self.columns = int(round(width / resolution))
        self.occupancy_grid = np.zeros((self.rows, self.columns))

        self.x_boundaries = (-width/2., width/2.)
        self.y_boundaries = (-height/2., height/2.)

    def map_to_world_coordinates(self, row_index, column_index):
        '''Converts the given map coordinates to world coordinates
        by finding the center of the appropriate map cell.
        Raises a ValueError if the input coordinates are out of the grid.

        Returns:
        coordinates -- A 'Coordinates' object with the coordinates of the cell's center.

        '''
        if self.invalid_indices(row_index, column_index):
            raise ValueError('OccupancyGrid: Invalid coordinates')
        x = (x_boundaries[0] + resolution / 2.) + column_index * resolution
        y = (y_boundaries[0] + resolution / 2.) + row_index * resolution
        return Coordinates(x, y)

    def world_to_map_coordinates(self, x, y):
        '''Converts the given world coordinates to map coordinates.
        Raises a ValueError if the input coordinates are out of the map.

        Returns:
        coordinates -- A 'Coordinates' object with the coordinates of the appropriate cell.

        '''
        if self.invalid_coordinates(x, y):
            raise ValueError('OccupancyGrid: Invalid coordinates')
        column = int((x - x_boundaries[0]) / resolution)
        row = int((y - y_boundaries[0]) / resolution)
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
