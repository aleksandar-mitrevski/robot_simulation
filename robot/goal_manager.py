#!/usr/bin/env python
import numpy as np

import rospy
import tf
from geometry_msgs.msg import PoseStamped, Quaternion
from navigation.srv import GoToGoalPosition
from scripts.laser_data import LaserData

class GoalManagerNode(object):
    def __init__(self):
        self.goal_file_name = rospy.get_param('~goal_file_name', None)
        self.goals = self.read_goal_file(self.goal_file_name)

        rospy.sleep(10)
        self.go_to_goals()

    def go_to_goals(self):
        number_of_goals = self.goals.shape[0]
        for i in xrange(number_of_goals):
            rospy.wait_for_service('go_to_goal')
            try:
                pose_msg = PoseStamped()
                pose_msg.pose.position.x = self.goals[i,0]
                pose_msg.pose.position.y = self.goals[i,1]
                pose_msg.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, self.goals[i,2]))

                proxy = rospy.ServiceProxy('go_to_goal', GoToGoalPosition)
                result = proxy(pose_msg)
                if result.response.success:
                    print 'Goal reached'
                else:
                    print 'Goal unreachable'
            except rospy.ServiceException, e:
                rospy.logerr('go_to_goal service call failed')

    def read_goal_file(self, goal_file_name):
        goals = np.genfromtxt(goal_file_name)
        return goals

if __name__ == '__main__':
    rospy.init_node('goal_manager')

    try:
        node = GoalManagerNode()
        rospy.spin()
    except rospy.ROSInterruptException: pass
