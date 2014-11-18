from math import sqrt

class Velocity(object):
    '''Defines a velocity vector.

    Author -- Aleksandar Mitrevski

    '''
    def __init__(self, linear_x=0., linear_y=0., angular=0.):
        self.linear_x = linear_x
        self.linear_y = linear_y
        self.angular = angular
