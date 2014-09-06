#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan

from sympy.geometry import Point

from scripts.motion_controller import MotionController
from scripts.obstacle_follower import ObstacleFollower
from scripts.pose import Pose
from scripts.velocity import Velocity
from scripts.directions import Directions

class MotionControllerNode(object):
    def __init__(self):
        self.x = 0.
        self.y = 0.
        self.heading = 0.
        self.scans = None

        self.velocity_linear = float(rospy.get_param('~linear_velocity', '0.01'))
        self.velocity_angular = float(rospy.get_param('~angular_velocity', '0.01'))
        self.safe_distance = float(rospy.get_param('~obstacle_safe_distance', '0.1'))
        self.direction = rospy.get_param('~obstacle_following_direction', 'ccw')

        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_broadcaster.sendTransform((0.,0.,0.),(0.,0.,0.,1.), rospy.Time.now(), "base_link", "odom")

        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        rospy.Subscriber('move_base_simple/goal', PoseStamped, self.move_to_goal_callback)
        rospy.Subscriber('laser_scan', LaserScan, self.get_scans)

        while not rospy.is_shutdown():
            self.publish_transform()

    def move_to_goal_callback(self, goal):
        goal_reached = False
        goal_unreachable = False
        obstacle_following = False

        orientation_quaternion = (goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w)
        euler_orientation = tf.transformations.euler_from_quaternion(orientation_quaternion)
        goal_pose = Pose(goal.pose.position.x, goal.pose.position.y, euler_orientation[2])

        motion_controller = MotionController(Velocity(self.velocity_linear, self.velocity_linear, self.velocity_angular))
        obstacle_follower = ObstacleFollower(Velocity(self.velocity_linear, self.velocity_linear, self.velocity_angular), self.safe_distance, self.direction)

        obstacle_following_start_point = Point(0,0)
        goal_point = Point(goal_pose.x, goal_pose.y)

        rospy.loginfo(self.safe_distance)
        rospy.loginfo('Goal pose: ' + str(goal_pose.x) + ' ' + str(goal_pose.y) + ' ' + str(goal_pose.angle))
        while not goal_reached and not goal_unreachable:
            current_pose = Pose(self.x, self.y, self.heading)

            scans = self.map_scans()
            if not obstacle_following:
                if self.close_to_obstacle(scans):
                    obstacle_following = True
                    obstacle_following_start_point = Point(self.x, self.y)
                    motion_controller.reset_states()

            if obstacle_following:
                current_point = Point(self.x, self.y)
                if obstacle_following_start_point.distance(current_point) > 1e-5 and Point.is_collinear(obstacle_following_start_point, goal_point, current_point):
                    obstacle_following = False
                else:
                    velocity = obstacle_follower.calculate_velocity(current_pose, scans)
                    print obstacle_follower.velocity.linear_x, ' ', obstacle_follower.velocity.linear_y, ' ', obstacle_follower.velocity.angular
                    print velocity.linear_x, ' ', velocity.linear_y, ' ', velocity.angular
            else:
                velocity, goal_reached = motion_controller.calculate_velocity(current_pose, goal_pose)

            self.publish_velocity(velocity)
            self.publish_transform(velocity)
            rospy.sleep(0.1)
        rospy.loginfo('Goal reached')

    def get_scans(self, scans):
        self.scans = scans

    def map_scans(self):
        front_scan_index = 0
        left_diagonal_scan_index = 0
        left_side_scan_index = 0
        right_diagonal_scan_index = 0
        right_side_scan_index = 0

        angle = self.scans.angle_min
        while abs(angle) > 1e-2:
            angle = angle + self.scans.angle_increment
            front_scan_index = front_scan_index + 1

        left_side_scan_index = 0
        right_side_scan_index = len(self.scans.ranges) - 1

        left_diagonal_scan_index = (front_scan_index + left_side_scan_index) / 2
        right_diagonal_scan_index = (front_scan_index + right_side_scan_index) / 2

        front_range = self.scans.ranges[front_scan_index]
        left_diagonal_range = self.scans.ranges[left_diagonal_scan_index]
        left_side_range = self.scans.ranges[left_side_scan_index]
        right_diagonal_range = self.scans.ranges[left_diagonal_scan_index]
        right_side_range = self.scans.ranges[left_side_scan_index]

        return [left_side_range, left_diagonal_range, front_range, right_diagonal_range, right_side_range]

    def close_to_obstacle(self, scans):
        for i in xrange(len(scans)):
            if scans[i] < self.safe_distance:
                return True
        return False

    def publish_velocity(self, velocity):
        velocity_msg = Twist()
        velocity_msg.linear.x = velocity.linear_x
        velocity_msg.linear.y = 0.
        velocity_msg.angular.z = velocity.angular
        self.velocity_publisher.publish(velocity_msg)

    def publish_transform(self, velocity=None):
        linear_x_velocity = 0.
        linear_y_velocity = 0.
        angular_velocity = 0.

        if velocity != None:
            linear_x_velocity = velocity.linear_x
            linear_y_velocity = velocity.linear_y
            angular_velocity = velocity.angular

        self.x = self.x + linear_x_velocity
        self.y = self.y + linear_y_velocity
        self.heading = self.heading + angular_velocity
        self.tf_broadcaster.sendTransform((self.x, self.y, 0.), tf.transformations.quaternion_from_euler(0, 0, self.heading), rospy.Time.now(), "base_link", "odom")

if __name__ == '__main__':
    rospy.init_node('motion_controller')
    try:
        MotionControllerNode()
    except rospy.ROSInterruptException: pass
