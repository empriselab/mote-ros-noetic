#!/bin/bash
set -eo pipefail

apt-get update
apt-get install -y --no-install-recommends libxml2-utils clang-format clang-tidy python3-pip
pip install catkin_lint

echo "=== xmllint ==="
find src/mote_ros_noetic \( -name "package.xml" -o -name "*.launch" \) -print0 \
  | xargs -0 -r -I{} xmllint --noout {}

echo "=== clang-format ==="
clang-format --dry-run --Werror \
  src/mote_ros_noetic/mote_base/src/*.cpp \
  src/mote_ros_noetic/mote_base/include/mote_base/*.h \
  src/mote_ros_noetic/mote_base/test/*.cpp

echo "=== catkin_lint ==="
catkin_lint --rosdistro noetic src/mote_ros_noetic

# ROS's setup.bash references unset variables, so it's incompatible with -u.
source /opt/ros/noetic/setup.bash
set -u

echo "=== build + test ==="
catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_CXX_FLAGS="-Wall -Wextra -Werror"
catkin_make run_tests
catkin_test_results build

echo "=== clang-tidy ==="
find src/mote_ros_noetic/mote_base/src src/mote_ros_noetic/mote_base/include \( -name '*.cpp' -o -name '*.h' \) -print0 \
  | xargs -0 -r clang-tidy -p build --warnings-as-errors='*'
