import numpy as np

class LaserData(object):
    '''Defines a structure for storing data related to a laser scanner.

    Author -- Aleksandar Mitrevski

    '''
    def __init__(self, frame):
        '''
        Keyword arguments:

        frame -- Name of the scanner's frame.

        '''
        self.frame_id = frame
        self.laser_position = -1. * np.ones(2)
        self.heading = 0.
        self.obstacle_distances = None
