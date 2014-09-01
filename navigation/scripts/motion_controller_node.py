#!/usr/bin/env python
import rospy
from std_msgs.msg import String

from motion_controller import MotionController
from pose import Pose
from velocity import Velocity

class MotionControllerNode(object):
    def __init__(self):
            

def publish_velocity(goal):
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        s = "hello world %s"%rospy.get_time()
        rospy.loginfo(s)
        pub.publish(s)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
