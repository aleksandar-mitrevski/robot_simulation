from math import sqrt

class Coordinates(object):
    '''Defines a structure for storing coordinates.

    Author -- Aleksandar Mitrevski

    '''
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def __eq__(self, other):
        return abs(self.x - other.x) < 1e-5 and abs(self.y - other.y) < 1e-5

    def __ne__(self, other):
        return abs(self.x - other.x) > 1e-5 or abs(self.y - other.y) > 1e-5

    def __add__(self, other):
        return Coordinates(self.x + other.x, self.y + other.y)

    def multiply(self, factor):
        return Coordinates(factor * self.x, factor * self.y)

    def distance(self, other):
        return sqrt((self.x - other.x) * (self.x - other.x) + (self.y - other.y) * (self.y - other.y))
