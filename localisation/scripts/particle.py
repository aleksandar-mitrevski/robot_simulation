from pose import Pose

class Particle(object):
    def __init__(self, x=0, y=0, heading=0, weight=0.):
        self.pose = Pose(x, y, heading)
        self.weight = weight
