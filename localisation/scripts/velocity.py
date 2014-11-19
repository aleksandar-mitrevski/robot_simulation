class Velocity(object):
    '''Generates a structure representing linear (both x and y) and angular velocity.

    Author -- Aleksandar Mitrevski

    '''
    def __init__(self, linear_x=0, linear_y=0, angular=0):
        self.linear_x = linear_x
        self.linear_y = linear_y
        self.angular = angular
