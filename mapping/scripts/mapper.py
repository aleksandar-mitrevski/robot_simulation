import numpy as np
from math import exp, log
from copy import copy

from coordinates import Coordinates
from mapping_parameters import MappingParameters

class Mapper(object):
    '''Defines a utility for performing map updates.

    Author -- Aleksandar Mitrevski

    '''
    def __init__(self, mapping_params):
        self.mapping_params = copy(mapping_params)

    def update_map(self, occupancy_grid, sensor_position, max_range, traced_world_coordinates, traced_map_coordinates, measurement):
        '''Updates the values of the cells traced by a range sensor ray.

        Keyword arguments:
        occupancy_grid -- A 2D numpy array representing an occupancy grid. The occupancy values are expected to be probabilities between 0 and 100.
        sensor_position -- The position of a range sensor.
        max_range -- The maximum range that the sensor can return (in meters).
        traced_world_coordinates -- A list of 'Coordinates' objects representing the world coordinates of the traced cells.
        traced_map_coordinates -- A list of 'Coordinates' objects representing the map coordinates of the traced cells.
        measurement -- The measurement returned by the sensor.

        Returns:
        occupancy_grid -- The occupancy grid with updated values.

        '''
        number_of_cells = len(traced_world_coordinates)
        for i in xrange(number_of_cells):
            current_cell = traced_map_coordinates[i]

            occ_prob = occupancy_grid[current_cell.x, current_cell.y] / 100.
            if occ_prob < 1e-10:
                occ_prob = occ_prob + 1e-10
            log_prob = log(occ_prob / (1. - occ_prob))

            distance = sensor_position.distance(traced_world_coordinates[i])
            if measurement < max_range and abs(distance - measurement) < self.mapping_params.obstacle_thickness / 2.:
                log_prob = log_prob + self.mapping_params.log_occ_prob_increment
            elif distance < measurement:
                log_prob = log_prob + self.mapping_params.log_free_prob_increment

            occ_prob = (1. - 1. / (1. + exp(log_prob))) * 100.
            occupancy_grid[current_cell.x, current_cell.y] = self.clamp_value(occ_prob, 0., 100.)

        return occupancy_grid

    def clamp_value(self, value, min_value, max_value):
        '''Clamps 'value' between 'min_value' and 'max_value'.
        '''
        if value < min_value:
            return min_value
        elif value > max_value:
            return max_value
        return value
