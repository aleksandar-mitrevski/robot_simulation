from math import cos, sin, atan2, exp, sqrt, pi
from pose import Pose
from velocity import Velocity

class MotionModel(object):
    def __init__(self, alpha1, alpha2, alpha3, alpha4, noise_variance):
        self.alpha1 = alpha1
        self.alpha2 = alpha2
        self.alpha3 = alpha3
        self.alpha4 = alpha4
        self.b = noise_variance

    def sample_motion_model(self, pose, motion_command):
        delta_x = 0.
        delta_y = 0.
        delta_heading = 0.

        if abs(motion_command.linear_x) > 0:
            delta_x = motion_command.linear_x * cos(pose.heading)
            delta_y = motion_command.linear_x * sin(pose.heading)
        else:
            delta_heading = motion_command.angular

        ideal_new_pose = Pose(pose.x + delta_x, pose.y + delta_y, pose.heading + delta_heading)

        delta_rot1 = atan2(ideal_new_pose.y - pose.y, ideal_new_pose.x - pose.x) - pose.heading
        delta_trans = sqrt((ideal_new_pose.x - pose.x) * (ideal_new_pose.x - pose.x) + (ideal_new_pose.y - pose.y) * (ideal_new_pose.y - pose.y))
        delta_rot2 = ideal_new_pose.heading - pose.heading - delta_rot1

        delta_rot1_prime = delta_rot1 + self.sample_normal_noise(self.alpha1 * delta_rot1 * delta_rot1 + self.alpha2 * delta_trans * delta_trans)
        delta_trans_prime = delta_trans + self.sample_normal_noise(self.alpha3 * self.delta_trans * self.delta_trans + self.alpha4 * delta_rot1 * delta_rot1 + self.alpha4 * delta_rot2 * delta_rot2)
        delta_rot2_prime = delta_rot2 + self.sample_normal_noise(self.alpha1 * delta_rot2 * delta_rot + self.alpha2 * delta_trans * delta_trans)

        new_pose = Pose()
        new_pose.x = position.x - delta_trans_prime * cos(pose.heading + delta_rot1_prime)
        new_pose.y = position.y - delta_trans_prime * sin(pose.heading + delta_rot1_prime)
        new_pose.heading = pose.heading + delta_rot1_prime + delta_rot2_prime

        return new_pose

    def sample_normal_noise(self, a):
        return 1. / sqrt(2*pi*self.b) * exp(-0.5 * a * a / self.b)
