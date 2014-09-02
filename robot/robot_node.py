#!/usr/bin/env python
import roslib
roslib.load_manifest('robot')

import rospy
import tf
from geometry_msgs.msg import Twist

class RobotNode(object):
    def __init__(self):
        self.x = 0.
        self.y = 0.
        self.heading = 0.

        rospy.Subscriber('cmd_vel', Twist, self.velocity_callback, queue_size=5)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_broadcaster.sendTransform((0.,0.,0.),(0.,0.,0.,1.), rospy.Time.now(), "base_link", "odom")

        while not rospy.is_shutdown():
            self.publish_transform()

    def velocity_callback(self, velocity_message):
        self.publish_transform(velocity_message)

    def publish_transform(self, velocity_message=None):
        linear_x_velocity = 0.
        linear_y_velocity = 0.
        angular_velocity = 0.

        if velocity_message != None:
            linear_x_velocity = velocity_message.linear.x
            linear_y_velocity = velocity_message.linear.y
            angular_velocity = velocity_message.angular.z
            #rospy.logerr('Velocity received ' + str(linear_x_velocity) + ' ' + str(linear_y_velocity) + ' ' + str(angular_velocity))

        self.x = self.x + linear_x_velocity
        self.y = self.y + linear_y_velocity
        self.heading = self.heading + angular_velocity
        self.tf_broadcaster.sendTransform((self.x, self.y, 0.), tf.transformations.quaternion_from_euler(0, 0, self.heading), rospy.Time.now(), "base_link", "odom")
        rospy.sleep(0.1)

if __name__ == '__main__':
    rospy.init_node('robot')

    try:
        node = RobotNode()
    except rospy.ROSInterruptException: pass
