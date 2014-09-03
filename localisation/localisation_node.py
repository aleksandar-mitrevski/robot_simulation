#!/usr/bin/env python
import rospy
import tf

class LocalisationNode(object):
    def __init__(self):
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_broadcaster.sendTransform((0.,0.,0.),(0.,0.,0.,1.), rospy.Time.now(), "odom", "map")

        while not rospy.is_shutdown():
            self.publish_transform()            

    def publish_transform(self):
        self.tf_broadcaster.sendTransform((0., 0., 0.), tf.transformations.quaternion_from_euler(0, 0, 0.), rospy.Time.now(), "odom", "map")

if __name__ == '__main__':
    rospy.init_node('localisation')
    try:
        LocalisationNode()
    except rospy.ROSInterruptException: pass
