from math import cos, sin
from velocity import Velocity

from directions import Directions

class ObstacleFollower(object):
    def __init__(self, velocity=None, safe_distance=0.1, direction=Directions.Counterclockwise):
        '''
        Keyword arguments:
        velocity -- A 'Velocity' object containing linear and angular velocities of the robot,
        safe_distance -- Safe distance from the obstacle.

        '''
        velocity = Velocity()

        if velocity != None:
            self.velocity = Velocity(velocity.linear_x, velocity.linear_y, velocity.angular)
        else:
            self.velocity = Velocity()
        self.safe_distance = safe_distance
        self.direction = direction

    def calculate_velocity(self, current_pose, sensor_measurements):
        heading_angle = self.normalise_angle(current_pose.angle)

        safe_front = sensor_measurements[0] > self.safe_distance
        safe_diagonal = sensor_measurements[1] > self.safe_distance
        safe_side = sensor_measurements[2] > self.safe_distance

        if not safe_front and not safe_diagonal and not safe_side:
            if self.direction == Directions.Counterclockwise:
                velocity = Velocity(0., 0., -self.velocity.angular)
            else:
                velocity = Velocity(0., 0., self.velocity.angular)
        elif not safe_front or (safe_front and safe_diagonal and safe_side):
            if self.direction == Directions.Counterclockwise:
                velocity = Velocity(0., 0., self.velocity.angular)
            else:
                velocity = Velocity(0., 0., -self.velocity.angular)
        else:
            velocity = Velocity(self.velocity.linear_x * cos(heading_angle), self.velocity.linear_y * sin(heading_angle), 0.)

        return velocity

    def normalise_angle(self, angle):
        '''Converts 'angle' to the range (0,2*pi).
        '''
        if angle < 0.:
            angle = angle + 2 * 3.14
        return angle
