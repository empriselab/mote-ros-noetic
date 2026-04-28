#!/bin/bash
set -e

if [ -z "$ROBOT_IP" ]; then
    echo "Error: ROBOT_IP environment variable is required."
    echo "Usage: docker run -e ROBOT_IP=192.168.x.x mote-ros-noetic"
    exit 1
fi

source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

exec roslaunch /catkin_ws/src/mote_ros_noetic/mote_base/launch/mote.launch \
    robot_ip:="$ROBOT_IP" \
    ${WHEEL_SEPARATION:+wheel_separation:="$WHEEL_SEPARATION"} \
    ${WHEEL_RADIUS:+wheel_radius:="$WHEEL_RADIUS"} \
    ${LASER_FRAME:+laser_frame:="$LASER_FRAME"} \
    ${IMU_FRAME:+imu_frame:="$IMU_FRAME"}
