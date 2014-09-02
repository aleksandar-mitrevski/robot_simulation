from math import sqrt, atan2, acos

class Pose(object):
    def __init__(self, x=0., y=0., angle=0.):
        self.x = x
        self.y = y
        self.angle = angle

    def __add__(self, other):
        new_pose = Pose()
        new_pose.x = self.x + other.x
        new_pose.y = self.y + other.y
        return new_pose

    def __sub__(self, other):
        new_pose = Pose()
        new_pose.x = self.x - other.x
        new_pose.y = self.y - other.y
        return new_pose

    def dot_product(self, other):
        dot = self.x * other.x + self.y + other.y
        return dot

    def norm(self):
        vector_norm = sqrt((self.x * self.x) + (self.y * self.y))
        return vector_norm

    def calculate_angle(self, other):
        '''Calculates the angle between 'self' and 'other'.        
        '''
        angle = 0.
        if self.is_zero_vector():
            angle = atan2(other.y, other.x)
            if angle < 0.:
                angle = angle + 2 * 3.14
        elif other.is_zero_vector():
            angle = atan2(self.y, self.x)
            if angle < 0.:
                angle = angle + 2 * 3.14
        else:
            angle = acos(self.dot_product(other) / (self.norm() * other.norm()))
        return angle

    def is_zero_vector(self):
        return self.norm() < 1e-5
