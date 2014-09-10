#!/usr/bin/env python
import numpy as np

import rospy
import tf
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan

from scripts.motion_controller import MotionController
from scripts.obstacle_follower import ObstacleFollower
from scripts.geometry import Pose, Point
from scripts.velocity import Velocity
from scripts.enums import Directions
from scripts.measurements import SensorMeasurements

from navigation.srv import GoToGoalPosition, GoToGoalPositionResponse
from navigation.msg import GoToGoalResponse

class MotionControllerNode(object):
    def __init__(self):
        self.x = 0.
        self.y = 0.
        self.heading = 0.
        self.front_scans = None
        self.back_scans = None

        self.velocity_linear = float(rospy.get_param('~linear_velocity', '0.01'))
        self.velocity_angular = float(rospy.get_param('~angular_velocity', '0.01'))
        self.noise_linear_x = float(rospy.get_param('~noise_linear_x', '0.000001'))
        self.noise_linear_y = float(rospy.get_param('~noise_linear_y', '0.000001'))
        self.noise_angular = float(rospy.get_param('~noise_angular', '0.00000001'))
        self.safe_distance = float(rospy.get_param('~obstacle_safe_distance', '0.1'))
        self.direction = rospy.get_param('~obstacle_following_direction', 'right')
        self.front_sensor_frame = rospy.get_param('~front_sensor_frame', '/laser_front')
        self.back_sensor_frame = rospy.get_param('~back_sensor_frame', '/laser_back')

        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_broadcaster.sendTransform((0.,0.,0.),(0.,0.,0.,1.), rospy.Time.now(), "base_link", "odom")

        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.map_velocity_publisher = rospy.Publisher('motion_command', Twist, queue_size=5)
        rospy.Subscriber('move_base_simple/goal', PoseStamped, self.move_to_goal_callback)
        rospy.Subscriber('laser_scan', LaserScan, self.get_scans)
        self.go_to_goal_service = rospy.Service('go_to_goal', GoToGoalPosition, self.go_to_goal_service_call_handler)

        while not rospy.is_shutdown():
            self.publish_transform()

    def move_to_goal_callback(self, goal):
        self.go_to_goal(goal)

    def go_to_goal_service_call_handler(self, request):
        goal_unreachable = self.go_to_goal(request.pose)

        response = GoToGoalResponse()
        if goal_unreachable:
            response.success = False
        else:
            response.success = True
        return response

    def go_to_goal(self, goal):
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

        rospy.loginfo('Goal pose: ' + str(goal_pose.x) + ' ' + str(goal_pose.y) + ' ' + str(goal_pose.angle))
        while not goal_reached and not goal_unreachable:
            current_pose = Pose(self.x, self.y, self.heading)

            if obstacle_following:
                current_point = Point(self.x, self.y)
                if obstacle_following_start_point.distance(current_point) > 0.5 and Point.collinear(obstacle_following_start_point, goal_point, current_point):
                    obstacle_following = False
                else:
                    velocity = obstacle_follower.calculate_velocity(current_pose, scans)
            else:
                velocity, goal_reached = motion_controller.calculate_velocity(current_pose, goal_pose)

            scans = self.map_scans()
            if not obstacle_following:
                if scans.less_than(self.safe_distance):
                    obstacle_following = True
                    obstacle_following_start_point = Point(self.x, self.y)
                    motion_controller.reset_states()
                    velocity = Velocity()

            self.publish_velocity(velocity)
            self.publish_transform(velocity)
            rospy.sleep(0.05)

        return goal_unreachable

    def get_scans(self, scans):
        if scans.header.frame_id == self.front_sensor_frame:
            self.front_scans = scans
        else:
            self.back_scans = scans

    def map_scans(self):
        measurements = SensorMeasurements()

        front_scan_index = 0
        angle = self.front_scans.angle_min
        while abs(angle) > 1e-2:
            angle = angle + self.front_scans.angle_increment
            front_scan_index = front_scan_index + 1

        left_side_scan_index = 0
        right_side_scan_index = len(self.front_scans.ranges) - 1

        left_diagonal_scan_index = (front_scan_index + left_side_scan_index) / 2 - 1
        right_diagonal_scan_index = (front_scan_index + right_side_scan_index) / 2 + 1

        measurements.front = self.front_scans.ranges[front_scan_index]
        measurements.left_diagonal = self.front_scans.ranges[left_diagonal_scan_index]
        measurements.left_back_diagonal = self.back_scans.ranges[left_diagonal_scan_index]
        measurements.left = self.front_scans.ranges[left_side_scan_index]
        measurements.right_diagonal = self.front_scans.ranges[left_diagonal_scan_index]
        measurements.right_back_diagonal = self.back_scans.ranges[right_diagonal_scan_index]
        measurements.right = self.front_scans.ranges[left_side_scan_index]

        return measurements

    def publish_velocity(self, velocity):
        velocity_msg = Twist()
        velocity_msg.linear.x = velocity.linear_x
        velocity_msg.linear.y = 0.
        velocity_msg.angular.z = velocity.angular
        self.velocity_publisher.publish(velocity_msg)

        #publish the actual motion command in terms of the map in case it is being used in a simulation
        velocity_msg.linear.y = velocity.linear_y
        self.map_velocity_publisher.publish(velocity_msg)

    def publish_transform(self, velocity=None):
        linear_x_velocity = 0.
        linear_y_velocity = 0.
        angular_velocity = 0.

        if velocity != None:
            linear_x_velocity = velocity.linear_x
            linear_y_velocity = velocity.linear_y
            angular_velocity = velocity.angular

        linear_x_velocity = self.noise_linear_x * np.random.randn() + linear_x_velocity
        linear_y_velocity = self.noise_linear_y * np.random.randn() + linear_y_velocity
        angular_velocity = self.noise_angular * np.random.randn() + angular_velocity

        self.x = self.x + linear_x_velocity
        self.y = self.y + linear_y_velocity
        self.heading = self.heading + angular_velocity
        self.tf_broadcaster.sendTransform((self.x, self.y, 0.), tf.transformations.quaternion_from_euler(0, 0, self.heading), rospy.Time.now(), "base_link", "odom")

if __name__ == '__main__':
    rospy.init_node('motion_controller')
    try:
        MotionControllerNode()
    except rospy.ROSInterruptException: pass
