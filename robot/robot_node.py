#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Twist

from robot.srv import GenerateSensorMeasurements, GenerateSensorMeasurementsResponse
from robot.msg import SensorMeasurements

class RobotNode(object):
    '''
    Author -- Aleksandar Mitrevski
    '''
    def __init__(self):
        rospy.Subscriber('cmd_vel', Twist, self.velocity_callback, queue_size=5)

if __name__ == '__main__':
    rospy.init_node('robot')

    try:
        node = RobotNode()
        rospy.spin()
    except rospy.ROSInterruptException: pass
