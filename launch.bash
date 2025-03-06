#!/bin/bash

MODEL=/mnt/c/Users/yehan/Documents/robot_slam/description/robot.urdf.xacro
RVIZ=/mnt/c/Users/yehan/Documents/robot_slam/rviz/urdf.rviz

ros2 launch robot_slam talker.launch.py model:=$MODEL rvizconfig:=$RVIZ