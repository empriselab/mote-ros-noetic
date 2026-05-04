#!/bin/bash
set -e

if [ -z "$ROBOT_IP" ]; then
    echo "Error: ROBOT_IP environment variable is required."
    echo "Usage: docker run -e ROBOT_IP=192.168.x.x mote-ros-noetic"
    exit 1
fi

source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
