#!/bin/bash
set -eo pipefail

# ROS's setup.bash references unset variables, so it's incompatible with -u.
source /opt/ros/noetic/setup.bash
set -u

catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_CXX_FLAGS="-Wall -Wextra -Werror"
catkin_make run_tests
catkin_test_results build

apt-get update
apt-get install -y --no-install-recommends clang-tidy
find src/mote_ros_noetic/mote_base/src src/mote_ros_noetic/mote_base/include \( -name '*.cpp' -o -name '*.h' \) -print0 \
  | xargs -0 -r clang-tidy -p build --warnings-as-errors='*'
