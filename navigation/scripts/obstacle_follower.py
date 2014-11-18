from math import cos, sin
from velocity import Velocity

from enums import Directions

class ObstacleFollower(object):
    '''Defines a utility class for controlling the motion of a robot
    when it's in an obstacle following mode.

    Author -- Aleksandar Mitrevski

    '''
    def __init__(self, velocity=None, safe_distance=0.1, direction=Directions.Left):
        '''
        Keyword arguments:
        velocity -- A 'Velocity' object containing linear and angular velocities of the robot (default None).
        safe_distance -- Safe distance from the obstacle in meters (default 0.1m).
        direction -- A 'Directions' value indicating whether the robot is in a right or left obstacle following mode (default Left).

        '''
        if velocity != None:
            self.velocity = Velocity(velocity.linear_x, velocity.linear_y, velocity.angular)
        else:
            self.velocity = Velocity()
        self.safe_distance = safe_distance
        self.direction = direction

        #used for advancing the position of the robot
        #even when it's following an obstacle
        self.tic = True

    def calculate_velocity(self, current_pose, measurements):
        '''Calculates an appropriate velocity for following an obstacle
        given the current robot pose and the set of distance measurements.

        Keyword arguments:
        current_pose -- A 'geometry.Pose' object.
        measurements -- A 'measurements.SensorMeasurements' object.

        '''
        heading_angle = self.normalise_angle(current_pose.angle)

        diagonal = 0.
        back_diagonal = 0.
        side = 0.

        safe_front = measurements.front > self.safe_distance
        safe_diagonal = False
        safe_back_diagonal = False
        safe_side = False

        if self.direction == Directions.Left:
            safe_diagonal = measurements.left_diagonal > self.safe_distance
            safe_back_diagonal = measurements.right_back_diagonal > self.safe_distance
            safe_side = measurements.left > self.safe_distance

            diagonal = measurements.left_diagonal
            back_diagonal = measurements.right_back_diagonal
            side = measurements.left
        else:
            safe_diagonal = measurements.right_diagonal > 2 * self.safe_distance
            safe_back_diagonal = measurements.left_back_diagonal > 2 * self.safe_distance
            safe_side = measurements.right > 2 * self.safe_distance

            diagonal = measurements.right_diagonal
            back_diagonal = measurements.left_back_diagonal
            side = measurements.right

        velocity = Velocity()

        if not safe_front or not safe_diagonal:
            #rotates the robot if either the front of the diagonal is unsafe
            if self.direction == Directions.Left:
                velocity = Velocity(0., 0., self.velocity.angular)
            else:
                velocity = Velocity(0., 0., -self.velocity.angular)
        elif safe_front:
            if back_diagonal < 1.5 * self.safe_distance:
                velocity = Velocity(self.velocity.linear_x * cos(heading_angle), self.velocity.linear_y * sin(heading_angle), 0.)
            elif side < 1.5 * self.safe_distance:
                if self.direction == Directions.Left:
                    velocity = Velocity(0., 0., self.velocity.angular)
                else:
                    velocity = Velocity(0., 0., -self.velocity.angular)
            elif back_diagonal > self.safe_distance and back_diagonal < 1.5 * self.safe_distance:
                velocity = Velocity(self.velocity.linear_x * cos(heading_angle), self.velocity.linear_y * sin(heading_angle), 0.)
            else:
                if self.direction == Directions.Left:
                    velocity = Velocity(0., 0., -self.velocity.angular)
                else:
                    velocity = Velocity(0., 0., self.velocity.angular)

            if self.tic:
                #we overwrite the previously calculated velocity
                #at each second call of the function because we don't want
                #the robot to get stuck in rotation mode if it is far from an obstacle
                velocity = Velocity(self.velocity.linear_x * cos(heading_angle), self.velocity.linear_y * sin(heading_angle), 0.)
                self.tic = False
            else:
                self.tic = True
        return velocity

    def normalise_angle(self, angle):
        '''Converts 'angle' to the range (0,2*pi).
        '''
        if angle < 0.:
            angle = angle + 2 * 3.14
        return angle
