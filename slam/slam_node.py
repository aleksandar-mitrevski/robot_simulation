#!/usr/bin/env python
import numpy as np
from math import cos, sin

import rospy
import tf
from sensor_msgs.msg import LaserScan

class SLAMNode(object):
    def __init__(self):
        self.map_frame = rospy.get_param('map_frame', '/map')

        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_broadcaster.sendTransform((0.,0.,0.),(0.,0.,0.,1.), rospy.Time.now(), self.odom_frame, self.map_frame)

        while not rospy.is_shutdown():
            self.tf_broadcaster.sendTransform((0.,0.,0.),(0.,0.,0.,1.), rospy.Time.now(), self.odom_frame, self.map_frame)

if __name__ == '__main__':
    rospy.init_node('slam')

    try:
        node = SLAMNode()
    except rospy.ROSInterruptException: pass
