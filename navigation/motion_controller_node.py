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
        self.x = 0.
        self.y = 0.
        self.heading = 0.

        self.velocity_linear = float(rospy.get_param('linear_velocity', '0.01'))
        self.velocity_angular = float(rospy.get_param('angular_velocity', '0.01'))

        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_broadcaster.sendTransform((0.,0.,0.),(0.,0.,0.,1.), rospy.Time.now(), "base_link", "odom")

        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        rospy.Subscriber('move_base_simple/goal', PoseStamped, self.move_to_goal_callback)

        while not rospy.is_shutdown():
            self.publish_transform()

    def move_to_goal_callback(self, goal):
        goal_reached = False

        orientation_quaternion = (goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w)
        euler_orientation = tf.transformations.euler_from_quaternion(orientation_quaternion)
        goal_pose = Pose(goal.pose.position.x, goal.pose.position.y, euler_orientation[2])

        motion_controller = MotionController(Velocity(self.velocity_linear, self.velocity_linear, self.velocity_angular))

        rospy.loginfo('Goal pose: ' + str(goal_pose.x) + ' ' + str(goal_pose.y) + ' ' + str(goal_pose.angle))
        while not goal_reached:
            current_pose = Pose(self.x, self.y, self.heading)
            velocity, goal_reached = motion_controller.calculate_velocity(current_pose, goal_pose)
            self.publish_velocity(velocity)
            self.publish_transform(velocity)
            rospy.sleep(0.1)
        rospy.loginfo('Goal reached')

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
