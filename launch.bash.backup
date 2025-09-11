#!/bin/bash

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Get the directory of the script
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

MODEL=${SCRIPT_DIR}/src/robot_slam/description/robot.urdf.xacro

RVIZ_CONFIG=${SCRIPT_DIR}/src/robot_slam/rviz/urdf.rviz

# echo "File path is: ${SCRIPT_DIR}"

echo -e "${BLUE}Model path:${NC}\t\t${MODEL}"

echo -e "${BLUE}rvizconfig path:${NC}\t${RVIZ_CONFIG}"

sleep 3

ros2 launch robot_slam talker.launch.py model:=${MODEL} rvizconfig:=${RVIZ_CONFIG}