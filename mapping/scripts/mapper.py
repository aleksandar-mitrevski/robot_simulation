import numpy as np
from math import exp, log
from copy import copy

from coordinates import Coordinates
from mapping_parameters import MappingParameters

class Mapper(object):
    def __init__(self, mapping_params):
        self.mapping_params = copy(mapping_params)

    def update_map(self, occupancy_grid, sensor_position, max_range, traced_world_coordinates, traced_map_coordinates, measurement):
        number_of_cells = len(traced_world_coordinates)
        for i in xrange(number_of_cells):
            current_cell = traced_map_coordinates[i]

            occ_prob = occupancy_grid[current_cell.x, current_cell.y] / 100. + 1e-10
            log_prob = log(occ_prob / (1. - occ_prob))

            distance = sensor_position.distance(traced_world_coordinates[i])
            if distance < max_range and abs(distance - measurement) < self.mapping_params.obstacle_thickness / 2.:
                log_prob = log_prob + self.mapping_params.log_occ_prob_increment
            elif distance < measurement:
                log_prob = log_prob + self.mapping_params.log_free_prob_increment

            occ_prob = (1. - 1. / (1. + exp(log_prob))) * 100.
            occupancy_grid[current_cell.x, current_cell.y] = self.normalise_value(occ_prob, 0., 100.)

        return occupancy_grid

    def normalise_value(self, value, min_value, max_value):
        if value < min_value:
            return min_value
        elif value > max_value:
            return max_value
        return value
