ur5
===

ROS packages for the UR5 Robot with MoveIt and RI environment

## Installation

Clone this and the gripper (robotiq) repositories
  ```
  $ git clone https://github.com/Raoufsawas/UR5_Gazebo.git
  ```
Build using catkin_make
  ```
  $ cd ur5_ws; catkin_make
  ```

## Visualization of UR5 environment

To visualize the UR5 environment, launch the following:
  ```
  $ roslaunch ur5_gazebo ur5_PiH.launch
  ```
To start the MoveIt packages, launch the following:
  ```
  $ roslaunch ur5_moveit_ws demo_planning_execution.launch
  ```

## start the training

launch the following:
  ```
  $ cd ur5_ws/stable_basline; python3 main.py
  ```
