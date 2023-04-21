# Hybrid-Granular-Jammer-Gripper-Driver

## Please refer to different branch for just Robotiq Gripper with action client for joint state feedback without Arduino or Vaccuum.


notice that this code is updated based on [Robotiq controller](https://github.com/ros-industrial/robotiq).

I only made updates to [Robitiq_control](https://github.com/ros-industrial/robotiq/tree/kinetic-devel/robotiq_2f_gripper_control) and added Arduino connection
functionality and feedback for the vaccuum to activate according to Robotiq_joint_state.

Arduino code for Vaccum activation and ROS communication is included in this repository.

inside the [nodes file](/nodes), it includes action server and client for the gripper so that it will be able to read point joint command from 0 -> 0.8.

Also you are able to visualize the change in joint state in RViz.

