from copy import copy

class MappingParameters(object):
    def __init__(self, obstacle_thickness, occ_prob_increment, free_prob_increment):
        self.obstacle_thickness = obstacle_thickness
        self.occ_prob_increment = occ_prob_increment
        self.free_prob_increment = free_prob_increment

    def __copy__(self):
        return MappingParameters(self.obstacle_thickness, self.occ_prob_increment, self.free_prob_increment)
