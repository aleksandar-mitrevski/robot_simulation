from random import uniform

class Pose(object):
    '''Defines a structure for storing a two-dimensional pose.

    Author -- Aleksandar Mitrevski

    '''
    def __init__(self, x=0, y=0, heading=0):
        self.x = x
        self.y = y
        self.heading = heading

    def __copy__(self):
        return Pose(self.x, self.y, self.heading)

    def __deepcopy__(self, memo):
        return Pose(self.x, self.y, self.heading)

    @staticmethod
    def random_pose(neg_x_limit, x_limit, neg_y_limit, y_limit):
        '''Generates a random pose with coordinates within the given limits
        and heading in the range (-pi,pi).
        '''
        x = uniform(neg_x_limit, x_limit)
        y = uniform(neg_y_limit, y_limit)
        heading = uniform(-3.14, 3.14)

        return Pose(x,y,heading)
