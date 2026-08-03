#!/bin/bash
set -euo pipefail

apt-get update
apt-get install -y --no-install-recommends libxml2-utils
find src/mote_ros_noetic \( -name "package.xml" -o -name "*.launch" \) -print0 \
  | xargs -0 -r -I{} xmllint --noout {}
