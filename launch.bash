#!/bin/bash
source env_init.bash
ros2 launch robot_slam talker.launch.py model:=/mnt/c/Users/yehan/Documenti/robot_slam/description/robot.urdf.xacro rvizconfig:=/mnt/c/Users/yehan/Documenti/robot_slam/rviz/urdf.rviz
