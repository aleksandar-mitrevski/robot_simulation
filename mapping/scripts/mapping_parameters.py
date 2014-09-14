from copy import copy

class MappingParameters(object):
    def __init__(self, obstacle_thickness, log_occ_prob_increment, log_free_prob_increment):
        self.obstacle_thickness = obstacle_thickness
        self.log_occ_prob_increment = log_occ_prob_increment
        self.log_free_prob_increment = log_free_prob_increment

    def __copy__(self):
        return MappingParameters(self.obstacle_thickness, self.log_occ_prob_increment, self.log_free_prob_increment)
