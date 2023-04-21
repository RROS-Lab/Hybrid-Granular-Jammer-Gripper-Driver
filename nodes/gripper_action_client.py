#!/usr/bin/python3


import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryActionResult
from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Float64

import numpy


class RobotiqGripper(object):
    ### Robotiq Gripper class for connecting to gripper action client.
    def __init__(self, name):
        self.name = name
        self._connect_to_gripper_action()

    def _connect_to_gripper_action(self):
        rospy.loginfo("Waiting for action server gripper_controller/gripper_action...")
        # setup action client for Joint Trajectory Action
        self._client = actionlib.SimpleActionClient('gripper_controller/gripper_action', FollowJointTrajectoryAction)
        self._client.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to action server")
    
rospy.init_node('robotiq_action_clinet')

if __name__ == '__main__':
  rospy.loginfo('Initiated client')

rospy.spin()