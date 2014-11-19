from math import sqrt

class Coordinates(object):
    '''Defines a structure for storing coordinates of grid cells.

    Author -- Aleksandar Mitrevski

    '''
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def distance(self, other):
        return sqrt((self.x - other.x) * (self.x - other.x) + (self.y - other.y) * (self.y - other.y))
