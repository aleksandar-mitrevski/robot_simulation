#!/usr/bin/env python
import sys, tty, termios

import rospy
from fault_injector.msg import InjectFault

class Commands(object):
    InjectFault = 1
    RepairSensor = 2
    RepairAllSensors = 3
    Quit = 4
    Unknown = 5

class FaultInjectorNode(object):
    def __init__(self):
        self.faulty_sensor_frames = list()
        self.fault_message_publisher = rospy.Publisher('inject_fault', InjectFault, queue_size=10)

        shutdown = False
        self.print_instructions()
        while not shutdown:
            character = self.read_character()
            command = self.read_command(character)
            if command == Commands.InjectFault or command == Commands.RepairSensor:
                self.manage_sensor(command)
            elif command == Commands.RepairAllSensors:
                self.repair_all_sensors()
            elif command == Commands.Quit:
                shutdown = True
            print 'Faulty sensors: ', self.faulty_sensor_frames

    def inject_fault(self, request):
        if request.frame_id in self.faulty_sensor_frames:
            return InjectFaultResponse(True)
        return InjectFaultResponse(False)

    def print_instructions(self):
        print 'Use the following keys:\ni for injecting a fault\nr for "repairing" the sensor (removing the fault)\na for repairing all sensors\nq to quit\n'

    def read_character(self):
        '''Code used from http://stackoverflow.com/questions/510357/python-read-a-single-character-from-the-user
        '''
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def read_command(self, character):
        if character == 'i':
            return Commands.InjectFault
        elif character == 'r':
            return Commands.RepairSensor
        elif character == 'a':
            return Commands.RepairAllSensors
        elif character == 'q':
            return Commands.Quit

        print 'unknown command ', character
        return Commands.Unknown

    def manage_sensor(self, command):
        sensor_frame = raw_input('Please enter the name of a sensor frame\n')
        if command == Commands.InjectFault:
            if sensor_frame not in self.faulty_sensor_frames:
                fault_msg = InjectFault()
                fault_msg.frame_id = sensor_frame
                fault_msg.inject_fault = True
                self.fault_message_publisher.publish(fault_msg)

                self.faulty_sensor_frames.append(sensor_frame)
            else:
                print 'Faults are already being injected to this sensor'

        if command == Commands.RepairSensor:
            if sensor_frame in self.faulty_sensor_frames:
                fault_msg = InjectFault()
                fault_msg.frame_id = sensor_frame
                fault_msg.inject_fault = False
                self.fault_message_publisher.publish(fault_msg)

                self.faulty_sensor_frames.remove(sensor_frame)
            else:
                print 'Faults have not been injected to this sensor; ignoring command'

    def repair_all_sensors(self):
        for sensor_frame in self.faulty_sensor_frames:
            fault_msg = InjectFault()
            fault_msg.frame_id = sensor_frame
            fault_msg.inject_fault = False
            self.fault_message_publisher.publish(fault_msg)

        self.faulty_sensor_frames[:] = []

if __name__ == '__main__':
    rospy.init_node('fault_injector')
    try:
        FaultInjectorNode()
    except rospy.ROSInterruptException: pass
