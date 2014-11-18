from math import cos, sin, atan2, copysign
from velocity import Velocity

class MotionController(object):
    '''Defines a utility class for controlling the motion of a robot
    when it's moving freely to a desired goal location.

    Author -- Aleksandar Mitrevski

    '''
    def __init__(self, velocity=None):
        if velocity != None:
            self.velocity = Velocity(velocity.linear_x, velocity.linear_y, velocity.angular)
        else:
            self.velocity = Velocity()
        self.facing_goal = False
        self.goal_location_reached = False

    def calculate_velocity(self, current_pose, goal):
        '''Calculates appropriate linear and angular velocities
        for moving from 'current_pose' to 'goal'. Supposes that the moving agent
        is as a differential drive, so first turns towards the goal location,
        then moves towards it, and finally rotates in order to reach the desired heading.

        Keyword arguments:
        current_pose -- A 'Pose' object describing the current pose of the moving agent.
        goal -- 'A' pose object describing the goal pose.

        Returns:
        velocity -- A 'Velocity' object with the calculated linear and angular velocities.

        '''
        goal_reached = False
        velocity = Velocity()

        vector_diff = goal - current_pose

        current_angle = self.normalise_angle(current_pose.angle)
        goal_angle = self.normalise_angle(goal.angle)
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
                if abs(current_angle - goal_angle) < 0.1:
                    goal_reached = True
                    self.reset_states()
                else:
                    heading_diff = self.smallest_angle(current_angle, goal_angle)
                    velocity = Velocity(0., 0., copysign(self.velocity.angular, heading_diff))
        else:
            heading_diff = self.smallest_angle(current_angle, diff_vector_direction)
            if abs(heading_diff) < 1e-2:
                self.facing_goal = True
            else:
                velocity = Velocity(0., 0., copysign(self.velocity.angular, heading_diff))

        return velocity, goal_reached

    def smallest_angle(self, angle1, angle2):
        '''Calculates the smallest (signed) angle from 'angle1' and 'angle2'.
        '''
        diff = angle1 - angle2
        if abs(diff) > 3.14:
            diff = diff - copysign(2*3.14, diff)
        return -diff

    def normalise_angle(self, angle):
        '''Converts 'angle' to the range (0,2*pi).
        '''
        if angle < 0.:
            angle = angle + 2 * 3.14
        return angle

    def reset_states(self):
        self.facing_goal = False
        self.goal_location_reached = False
