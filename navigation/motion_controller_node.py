#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Twist, PoseStamped

from scripts.motion_controller import MotionController
from scripts.pose import Pose
from scripts.velocity import Velocity
from math import atan2

class MotionControllerNode(object):
    def __init__(self):
        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        rospy.Subscriber('move_base_simple/goal', PoseStamped, self.move_to_goal_callback)

    def move_to_goal_callback(self, goal):
        goal_reached = False
        goal_pose = Pose(goal.pose.position.x, goal.pose.position.y, goal.pose.orientation.z)
        motion_controller = MotionController(Velocity(0.01, 0.01, 0.01))

        listener = tf.TransformListener()
        listener.waitForTransform("/odom", "/base_link", rospy.Time(), rospy.Duration(10.0))
        rospy.loginfo('Goal pose: ' + str(goal_pose.x) + ' ' + str(goal_pose.y) + ' ' + str(goal_pose.angle))
        while not goal_reached:
            try:
                (translation, rotation_quaternion) = listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
                rotation_euler = tf.transformations.euler_from_quaternion(rotation_quaternion)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                return

            current_pose = Pose(translation[0], translation[1], rotation_euler[2])
            velocity, goal_reached = motion_controller.calculate_velocity(current_pose, goal_pose)
            self.publish_velocity(velocity)
        rospy.loginfo('Goal reached')

    def publish_velocity(self, velocity):
        velocity_msg = Twist()
        velocity_msg.linear.x = velocity.linear_x
        velocity_msg.linear.y = velocity.linear_y
        velocity_msg.angular.z = velocity.angular
        self.velocity_publisher.publish(velocity_msg)

if __name__ == '__main__':
    rospy.init_node('motion_controller')
    try:
        MotionControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException: pass
