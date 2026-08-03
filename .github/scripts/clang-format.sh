#!/bin/bash
set -euo pipefail

apt-get update
apt-get install -y --no-install-recommends clang-format
clang-format --dry-run --Werror \
  src/mote_ros_noetic/mote_base/src/*.cpp \
  src/mote_ros_noetic/mote_base/include/mote_base/*.h \
  src/mote_ros_noetic/mote_base/test/*.cpp
