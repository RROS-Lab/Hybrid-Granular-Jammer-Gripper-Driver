#!/usr/bin/python3


import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryActionResult
from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Float64

import numpy


class RobotiqGripper(object):
    def __init__(self, name):
        self.name = name
        self._connect_to_gripper_action()

    def _connect_to_gripper_action(self):
        rospy.loginfo("Waiting for action server gripper_controller/gripper_action...")
        self._client = actionlib.SimpleActionClient('gripper_controller/gripper_action', FollowJointTrajectoryAction)
        self._client.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to action server")
    
rospy.init_node('robotiq_action_clinet')
pub = rospy.Publisher('/robotiq_pos', Int16, queue_size = 1)
rate = rospy.Rate(5)

# def callback(msg):
#     # print the actual message in its raw format
#     position_given = msg.goal.command.position
#     rospy.loginfo("%s", msg.goal.command.position)
#     # Set up goal
#     goal = FollowJointTrajectoryGoal()
#     print(goal)
#     print(str(position_given))
#     robotiq_gripper = RobotiqGripper('RobotiqGripper/main')
#     pos_val = position_given
#     print(str(pos_val))
#     # Publish to /myservo so that it arduino will servo write
#     pub.publish(pos_val)
#     rate.sleep()

if __name__ == '__main__':
  rospy.loginfo('Initiated client')
  robotiq_gripper = RobotiqGripper('RobotiqGripper/main')

rospy.spin()