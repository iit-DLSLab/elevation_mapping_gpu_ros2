#!/bin/bash
set -euo pipefail

WS="${WS:-$HOME/workspace}"
cd "$WS"

source "/opt/ros/$ROS_DISTRO/setup.bash"

# Optional: pull extra sources listed in docker/src.repos (kept for historical reasons).
if [ -f "src/elevation_mapping_cupy/docker/src.repos" ]; then
  vcs import < "src/elevation_mapping_cupy/docker/src.repos" src/ --recursive -w "$(($(nproc)/2))"
fi

rosdep update
rosdep install --from-paths src --ignore-src -y -r
