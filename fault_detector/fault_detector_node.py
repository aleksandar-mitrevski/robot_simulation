#!/usr/bin/env python
import rospy
import tf
from sensor_msgs.msg import LaserScan

from fault_detector.msg import FaultReport, FaultAlarm
from scripts.fault_detector import *
from scripts.detector_types import DetectorTypes

class FaultDetectorNode(object):
    def __init__(self):
        self.fault_detector_type = int(rospy.get_param('~fault_detector_type', '1'))
        self.dbn_file_name = rospy.get_param('~dbn_file_name', None)

        self.fault_detector = None
        if self.fault_detector_type == DetectorTypes.DynamicBayesianNetwork:
            self.fault_detector = DBNFaultDetector(self.dbn_file_name)
        elif self.fault_detector_type == DetectorTypes.RestrictedBoltzmannMachine:
            self.fault_detector = TRBMFaultDetector()
        elif self.fault_detector_type == DetectorTypes.ContinuousRestrictedBoltzmannMachine:
            self.fault_detector = TRBMContinuousFaultDetector()
        elif self.fault_detector_type == DetectorTypes.DeepBeliefNetwork:
            self.fault_detector = DeepBNFaultDetector()
        elif self.fault_detector_type == DetectorTypes.ContinuousDeepBeliefNetwork:
            self.fault_detector == DeepBNContinuousFaultDetector()
        self.added_sensors = list()

        rospy.Subscriber('laser_scan', LaserScan, self.process_measurements)
        self.fault_report_publisher = rospy.Publisher('fault_report', FaultReport, queue_size=10)
        self.fault_alarm_publisher = rospy.Publisher('fault_alarm', FaultAlarm, queue_size=10)

    def process_measurements(self, scans):
        if scans.header.frame_id not in self.added_sensors:
            sensor_keys = list()
            angle = scans.angle_min
            counter = 0
            while angle < scans.angle_max:
                sensor_keys.append((scans.header.frame_id,counter))
                angle = angle + scans.angle_increment
                counter = counter + 1
            self.fault_detector.add_sensor(sensor_keys)
            self.added_sensors.append(scans.header.frame_id)

        angle = scans.angle_min
        counter = 0
        while angle < scans.angle_max:
            key = (scans.header.frame_id, counter)

            if self.fault_detector == DetectorTypes.DynamicBayesianNetwork:
                self.fault_detector.update_belief(key, scans.ranges[counter])
                current_belief = self.fault_detector.get_current_belief(key)
                self.publish_failure_report(scans.header.frame_id, counter, current_belief)
            else:
                faulty_measurement = self.fault_detector.check_measurement(key, scans.ranges[counter])
                self.publish_fault_alarm(scans.header.frame_id, counter, faulty_measurement)

            angle = angle + scans.angle_increment
            counter = counter + 1

    def publish_failure_report(self, frame_id, scan_id, current_belief):
        msg = FaultReport()
        msg.frame_id = frame_id
        msg.scan_id = scan_id
        for state in current_belief.keys():
            msg.states.append(state)
            msg.probabilities.append(current_belief[state])
        self.fault_report_publisher.publish(msg)

    def publish_fault_alarm(self, frame_id, scan_id, fault):
        msg = FaultAlarm()
        msg.frame_id = frame_id
        msg.scan_id = scan_id
        msg.fault = fault
        self.fault_alarm_publisher.publish(msg)

if __name__ == '__main__':
    rospy.init_node('fault_detector')

    try:
        node = FaultDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException: pass
