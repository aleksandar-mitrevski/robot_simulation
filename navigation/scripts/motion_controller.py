class MotionController(object):
    def __init__(self, pose=None, velocity=None):
        self.pose = Pose(pose.x, pose.y, pose.angle)
        self.velocity = Velocity(velocity.linear, velocity.angular)
        self.facing_goal = False
        self.goal_reached = False

    def calculate_velocity(self, goal):
        if(self.facing_goal):
            vector_diff = self.pose - goal
            if not self.goal_reached and abs(vector_diff.norm()) < 1e-2:
                self.goal_reached = True
            else:
                return Velocity(self.velocity.linear, 0.)

            if self.goal_reached:
                if abs(self.pose.angle - goal.pose.angle) < 1e-2:
                    self.facing_goal = False
                    self.goal_reached = False
                else:
                    return Velocity(0.0, self.velocity.angular)
        else:
            vector_diff = self.pose - goal
            heading_diff = self.pose.calculate_angle(vector_diff)
            if abs(heading_diff) < 1e-2:
                self.facing_goal = True
            else:
                return Velocity(0.0, self.velocity.angular)

        return Velocity()
