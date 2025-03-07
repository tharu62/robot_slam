#!/bin/bash

MODEL=/home/utonto/ros2_ws/src/robot_slam/description/robot.urdf.xacro
RVIZ=/home/utonto/ros2_ws/src/robot_slam/rviz/urdf.rviz

ros2 launch robot_slam talker.launch.py model:=$MODEL rvizconfig:=$RVIZ