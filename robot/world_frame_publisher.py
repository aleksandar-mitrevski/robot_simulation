#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Twist

class WorldFrameNode(object):
    '''Manages the position of a robot in a ground truth frame.

    Author -- Aleksandar Mitrevski

    '''
    def __init__(self):
        self.x = 0.
        self.y = 0.
        self.heading = 0.

        rospy.Subscriber('motion_command', Twist, self.update_robot_position)

        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_broadcaster.sendTransform((0.,0.,0.),(0.,0.,0.,1.), rospy.Time.now(), "base_link_copy", "world")

        while not rospy.is_shutdown():
            self.update_robot_position()

    def update_robot_position(self, velocity=None):
        '''Updates the pose of a robot in the ground truth frame.
        '''
        linear_x_velocity = 0.
        linear_y_velocity = 0.
        angular_velocity = 0.

        if velocity != None:
            linear_x_velocity = velocity.linear.x
            linear_y_velocity = velocity.linear.y
            angular_velocity = velocity.angular.z

        self.x = self.x + linear_x_velocity
        self.y = self.y + linear_y_velocity
        self.heading = self.heading + angular_velocity
        self.tf_broadcaster.sendTransform((self.x, self.y, 0.), tf.transformations.quaternion_from_euler(0, 0, self.heading), rospy.Time.now(), "base_link_copy", "world")

if __name__ == '__main__':
    rospy.init_node('world_frame_publisher')

    try:
        node = WorldFrameNode()
    except rospy.ROSInterruptException: pass
