from math import cos, sin, atan2, exp, sqrt, pi
from random import uniform
from copy import copy

from pose import Pose
from velocity import Velocity
from filter_parameters import MotionModelNoiseParameters

class MotionModel(object):
    def __init__(self, noise_params):
        self.noise_params = copy(noise_params)

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

        delta_rot1_prime = delta_rot1 + self.sample_normal_noise(self.noise_params.alpha1 * delta_rot1 * delta_rot1 + self.noise_params.alpha2 * delta_trans * delta_trans)
        delta_trans_prime = delta_trans + self.sample_normal_noise(self.noise_params.alpha3 * delta_trans * delta_trans + self.noise_params.alpha4 * delta_rot1 * delta_rot1 + self.noise_params.alpha4 * delta_rot2 * delta_rot2)
        delta_rot2_prime = delta_rot2 + self.sample_normal_noise(self.noise_params.alpha1 * delta_rot2 * delta_rot2 + self.noise_params.alpha2 * delta_trans * delta_trans)

        new_pose = Pose()
        new_pose.x = pose.x - delta_trans_prime * cos(pose.heading + delta_rot1_prime)
        new_pose.y = pose.y - delta_trans_prime * sin(pose.heading + delta_rot1_prime)
        new_pose.heading = pose.heading + delta_rot1_prime + delta_rot2_prime

        return new_pose

    def sample_normal_noise(self, variance):
        sample_sum = 0.
        std_dev = sqrt(variance)
        for i in xrange(12):
            sample_sum = sample_sum + uniform(-std_dev, std_dev)
        return 0.5 * sample_sum
