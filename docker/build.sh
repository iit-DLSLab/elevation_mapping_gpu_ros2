#!/bin/bash
set -euo pipefail

WS="${WS:-$HOME/workspace}"
BUILD_TYPE="${BUILD_TYPE:-RelWithDebInfo}"

cd "$WS"
source "/opt/ros/$ROS_DISTRO/setup.bash"

colcon build \
  --symlink-install \
  --merge-install \
  --event-handlers console_direct+ \
  --cmake-args "-DCMAKE_BUILD_TYPE=${BUILD_TYPE}"

