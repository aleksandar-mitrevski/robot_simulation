from math import cos, sin
from velocity import Velocity

from enums import Directions

class ObstacleFollower(object):
    def __init__(self, velocity=None, safe_distance=0.1, direction=Directions.Left):
        '''
        Keyword arguments:
        velocity -- A 'Velocity' object containing linear and angular velocities of the robot,
        safe_distance -- Safe distance from the obstacle.

        '''
        if velocity != None:
            self.velocity = Velocity(velocity.linear_x, velocity.linear_y, velocity.angular)
        else:
            self.velocity = Velocity()
        self.safe_distance = safe_distance
        self.direction = direction
        self.tic = True

    def calculate_velocity(self, current_pose, measurements):
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
            safe_back_diagonal = measurements.left_back_diagonal > self.safe_distance
            safe_side = measurements.left > self.safe_distance

            diagonal = measurements.left_diagonal
            back_diagonal = measurements.left_back_diagonal
            side = measurements.left
        else:
            safe_diagonal = measurements.right_diagonal > 2 * self.safe_distance
            safe_back_diagonal = measurements.right_back_diagonal > 2 * self.safe_distance
            safe_side = measurements.right > 2 * self.safe_distance

            diagonal = measurements.right_diagonal
            back_diagonal = measurements.right_back_diagonal
            side = measurements.right

        velocity = Velocity()

        if not safe_front or not safe_diagonal:
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
