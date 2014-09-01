from math import sqrt

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

    def dot_product(other):
        dot = self.x * other.x + self.y + other.y;
        return dot

    def norm():
        vector_norm = sqrt((self.x * self.x) + (self.y * self.y));
        return vector_norm;

    def calculate_angle(other):
        angle = self.dot_product(other) / (self.norm() * other.norm());
        return angle
