#!/usr/bin/env python
import sys, tty, termios

import rospy
import tf
from geometry_msgs.msg import Twist

class KeyboardTeleopNode(object):
    def __init__(self):
        self.linear_velocity = float(rospy.get_param('linear_velocity', '0.01'))
        self.angular_velocity = float(rospy.get_param('angular_velocity', '0.01')) 

        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        shutdown = False
        self.print_instructions()
        while not shutdown:
            character = self.read_character()
            shutdown = self.send_motion_command(character)
            rospy.sleep(0.1)

    def print_instructions(self):
        print 'Use the following keys for operating your robot:\nw for moving forward\ns for moving backward\ne for rotating counterclockwise\nr for rotating clockwise\nq to exit\n'

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

    def send_motion_command(self, character):
        velocity = Twist()
        shutdown = False

        if character == 'w':
            velocity.linear.x = self.linear_velocity
        elif character == 's':
            print 'fuck'
            velocity.linear.x = -self.linear_velocity
        elif character == 'e':
            velocity.angular.z = self.angular_velocity
        elif character == 'r':
            velocity.angular.z = -self.angular_velocity
        elif character == 'q':
            shutdown = True

        self.velocity_publisher.publish(velocity)
        return shutdown

if __name__ == '__main__':
    rospy.init_node('keyboard_teleop')

    try:
        node = KeyboardTeleopNode()
    except rospy.ROSInterruptException: pass
