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
        side = 0.

        safe_front = measurements.front > 2 * self.safe_distance
        safe_diagonal = False
        safe_side = False

        if self.direction == Directions.Left:
            safe_diagonal = measurements.left_diagonal > 2 * self.safe_distance
            safe_side = measurements.left > 2 * self.safe_distance

            diagonal = measurements.left_diagonal
            side = measurements.left
        else:
            safe_diagonal = measurements.right_diagonal > 2 * self.safe_distance
            safe_side = measurements.right > 2 * self.safe_distance

            diagonal = measurements.right_diagonal
            side = measurements.right

        velocity = Velocity()

        if not safe_front or not safe_diagonal:
            print 'A'
            if self.direction == Directions.Left:
                velocity = Velocity(0., 0., self.velocity.angular)
            else:
                velocity = Velocity(0., 0., -self.velocity.angular)
        else:
            if side < 1.5 * self.safe_distance:
                print 'B'
                if self.direction == Directions.Left:
                    velocity = Velocity(0., 0., self.velocity.angular)
                else:
                    velocity = Velocity(0., 0., -self.velocity.angular)
            else:
                print 'C'
                if self.direction == Directions.Left:
                    velocity = Velocity(0., 0., -self.velocity.angular)
                else:
                    velocity = Velocity(0., 0., self.velocity.angular)

            if self.tic:
                print 'D'
                velocity = Velocity(0.5 * self.velocity.linear_x * cos(heading_angle), 0.5 * self.velocity.linear_y * sin(heading_angle), 0.)
                self.tic = False
            else:
                self.tic = True
        else:
            if self.direction == Directions.Left:
                velocity = Velocity(0., 0., self.velocity.angular)
            else:
                velocity = Velocity(0., 0., -self.velocity.angular)

        return velocity

    def normalise_angle(self, angle):
        '''Converts 'angle' to the range (0,2*pi).
        '''
        if angle < 0.:
            angle = angle + 2 * 3.14
        return angle
