from random import uniform

class Pose(object):
    def __init__(self, x=0, y=0, heading=0):
        self.x = x
        self.y = y
        self.heading = heading

    def __copy__(self):
        return Pose(self.x, self.y, self.heading)

    @staticmethod
    def random_pose(self, neg_x_limit, x_limit, neg_y_limit, y_limit):
        x = uniform(neg_x_limit, x_limit)
        y = uniform(neg_y_limit, y_limit)
        heading = uniform(-3.14, 3.14)

        return Pose(x,y,heading)
