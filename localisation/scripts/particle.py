from copy import copy, deepcopy
from pose import Pose

class Particle(object):
    def __init__(self, pose=None, weight=0.):
        if pose != None:
            self.pose = copy(pose)
        else:
            self.pose = Pose()
        self.weight = weight

    @staticmethod
    def random_particle(neg_x_limit, x_limit, neg_y_limit, y_limit, weight):
        return Particle(Pose.random_pose(neg_x_limit, x_limit, neg_y_limit, y_limit), weight)

    def __deepcopy__(self, memo):
        return Particle(deepcopy(self.pose, memo), self.weight)
