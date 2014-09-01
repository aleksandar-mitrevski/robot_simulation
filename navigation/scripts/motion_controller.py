from math import cos, sin, atan2
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

        if(self.facing_goal):
            vector_diff = goal - current_pose
            if not self.goal_location_reached and abs(vector_diff.norm()) < 1e-2:
                self.goal_location_reached = True
            else:
                angle_from_origin = math.atan2(vector_diff.y, vector_diff.x)
                if angle_from_origin < 0.:
                    angle_from_origin = angle_from_origin + 2 * 3.14

                linear_x_velocity = self.velocity.linear_x * math.cos(angle_from_origin)
                linear_y_velocity = self.velocity.linear_y * math.sin(angle_from_origin)
                velocity = Velocity(linear_x_velocity, linear_y_velocity, 0.)

            if self.goal_location_reached:
                if abs(current_pose.angle - goal.pose.angle) < 1e-2:
                    self.facing_goal = False
                    self.goal_location_reached = False
                    goal_reached = True
                else:
                    velocity = Velocity(0., 0., self.velocity.angular)
        else:
            vector_diff = current_pose - goal
            heading_diff = current_pose.calculate_angle(vector_diff)
            if abs(heading_diff) < 1e-2:
                self.facing_goal = True
            else:
                velocity = Velocity(0., 0., self.velocity.angular)

        return velocity, goal_reached
