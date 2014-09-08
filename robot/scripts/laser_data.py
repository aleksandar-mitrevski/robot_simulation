import numpy as np

class LaserData(object):
    def __init__(self, frame):
        self.frame_id = frame
        self.laser_position = -1. * np.ones(2)
        self.heading = 0.
        self.obstacle_distances = None
