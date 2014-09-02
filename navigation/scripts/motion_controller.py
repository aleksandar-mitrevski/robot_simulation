from math import cos, sin, atan2, copysign
from velocity import Velocity
import rospy

class MotionController(object):
    def __init__(self, velocity=None):
        self.velocity = Velocity(velocity.linear_x, velocity.linear_y, velocity.angular)
        self.facing_goal = False
        self.goal_location_reached = False

    def calculate_velocity(self, current_pose, goal):
        goal_reached = False
        velocity = Velocity()

        vector_diff = goal - current_pose

        current_angle = self.normalise_angle(current_pose.angle)
        diff_vector_direction = self.normalise_angle(atan2(vector_diff.y, vector_diff.x))
        goal_vector_direction = self.normalise_angle(atan2(goal.y, goal.x))

        if(self.facing_goal):
            if not self.goal_location_reached and abs(vector_diff.norm()) < 0.1:
                self.goal_location_reached = True
            elif not self.goal_location_reached:
                linear_x_velocity = self.velocity.linear_x * cos(diff_vector_direction)
                linear_y_velocity = self.velocity.linear_y * sin(diff_vector_direction)
                velocity = Velocity(linear_x_velocity, linear_y_velocity, 0.)

            if self.goal_location_reached:
                if abs(current_angle - goal_vector_direction) < 0.5:
                    self.facing_goal = False
                    self.goal_location_reached = False
                    goal_reached = True
                else:
                    heading_diff = self.smallest_angle(current_angle, goal_vector_direction)
                    velocity = Velocity(0., 0., copysign(self.velocity.angular, heading_diff))
        else:
            heading_diff = self.smallest_angle(current_angle, diff_vector_direction)
            if abs(heading_diff) < 1e-2:
                self.facing_goal = True
            else:
                velocity = Velocity(0., 0., copysign(self.velocity.angular, heading_diff))

        return velocity, goal_reached

    def smallest_angle(self, angle1, angle2):
        diff = angle1 - angle2
        if abs(diff) > 3.14:
            diff = diff - copysign(2*3.14, diff)
        return -diff

    def normalise_angle(self, angle):
        if angle < 0.:
            angle = angle + 2 * 3.14
        return angle
