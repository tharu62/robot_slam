#!/bin/bash

# Get the directory of the script
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# Set the ROS2 path
num=0
for i in $(ls -d /opt/ros/*/); do num=$((num + 1)); done
if [ $num -eq 1 ]; then
    ROS2_PATH=$(ls -d /opt/ros/*/)
else
    echo -e "${RED}Multiple ROS2 installations found${NC}"
    echo -e "${RED}Please set the ROS2_PATH variable manually${NC}"
    exit 1
fi

# Set the ROS2 workspace path
# ROS2_PATH=/opt/ros/humble

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}Script Directory${NC}:     : [$SCRIPT_DIR]"

echo -e "${GREEN}Setting up environment${NC}"

echo -e "${BLUE}sourcing ROS2 path:${NC} $ROS2_PATH"
source "$ROS2_PATH/setup.bash"

echo -e "${BLUE}Setting up ROS2 workspace:${NC} $SCRIPT_DIR/install/setup.bash"
source "$SCRIPT_DIR/install/setup.bash"