#!/bin/bash
set -euo pipefail

apt-get update
apt-get install -y --no-install-recommends python3-pip
pip install catkin_lint
catkin_lint --rosdistro noetic src/mote_ros_noetic
