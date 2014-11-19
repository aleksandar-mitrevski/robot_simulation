from copy import copy, deepcopy
from pose import Pose

class Particle(object):
    '''Defines a structure representing particles used in a particle filter.

    Author -- Aleksandar Mitrevski

    '''
    def __init__(self, pose=None, weight=0.):
        if pose != None:
            self.pose = copy(pose)
        else:
            self.pose = Pose()
        self.weight = weight

    @staticmethod
    def random_particle(neg_x_limit, x_limit, neg_y_limit, y_limit, weight):
        '''Returns a random particle within the limits of a map.
        '''
        return Particle(Pose.random_pose(neg_x_limit, x_limit, neg_y_limit, y_limit), weight)

    def __deepcopy__(self, memo):
        return Particle(deepcopy(self.pose, memo), self.weight)
