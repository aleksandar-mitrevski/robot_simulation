#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Twist, Pose2D

from scripts.motion_controller import MotionController
from scripts.pose import Pose
from scripts.velocity import Velocity

class MotionControllerNode(object):
    def __init__(self):
        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        rospy.Subscriber('goal_location', Pose2D, self.move_to_goal_callback)

    def move_to_goal_callback(self, goal_pose):
        goal_reached = False
        goal = Pose(goal_pose.x, goal_pose.y, goal_pose.theta)
        motion_controller = MotionController(Velocity(1., 1., 0.5))

        while not goal_reached:
            listener = tf.TransformListener()
            try:
                (trans, rot) = listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                return

            current_pose = Pose(trans[0], trans[1], rot[2])
            velocity, goal_reached = motion_controller.calculate_velocity(current_pose, goal)
            self.publish_velocity(velocity)

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
