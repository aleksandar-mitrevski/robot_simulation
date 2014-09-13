#!/usr/bin/env python
import rospy
import tf
from sensor_msgs.msg import LaserScan

from fault_detector.msg import FaultAlarm

class FaultDetectorNode(object):
    def __init__(self):
        rospy.Subscriber('laser_scan', LaserScan, self.velocity_callback)
        self.fault_alarm_publisher = rospy.Publisher('fault_alarm', FaultAlarm, queue_size=10)

    def process_measurements(self, scans):
        pass

if __name__ == '__main__':
    rospy.init_node('fault_detector')

    try:
        node = FaultDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException: pass
