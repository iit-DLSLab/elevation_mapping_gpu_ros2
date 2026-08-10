#!/bin/bash

set -euo pipefail

WORKSPACE_DIR="/home/ros/workspace"
BUILD_TYPE="${BUILD_TYPE:-RelWithDebInfo}" # Debug, Release, RelWithDebInfo, MinSizeRel
SKIP_SEMANTIC_SENSOR="${SKIP_SEMANTIC_SENSOR:-true}"

PLANE_SEGMENTATION_PACKAGES=(
  cgal5_catkin
  grid_map_filters_rsl
  convex_plane_decomposition_msgs
  convex_plane_decomposition
  convex_plane_decomposition_ros
)

COMMON_ARGS=(
  --merge-install
  --symlink-install
  --base-paths src
  --cmake-args
    "-DCMAKE_BUILD_TYPE=${BUILD_TYPE}"
    "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
    "-DBUILD_TESTING=OFF"
    "-DCMAKE_CXX_FLAGS=-Wl,--allow-shlib-undefined -Wall -Wextra -Wpedantic -Wshadow"
)

cd "${WORKSPACE_DIR}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [ -f "${WORKSPACE_DIR}/.map_env/bin/activate" ]; then
  # Keep the Python environment aligned with the one prepared in the image.
  source "${WORKSPACE_DIR}/.map_env/bin/activate"
fi

PACKAGES_TO_SKIP=("${PLANE_SEGMENTATION_PACKAGES[@]}")
if [ "${SKIP_SEMANTIC_SENSOR,,}" = "true" ]; then
  PACKAGES_TO_SKIP+=(semantic_sensor)
fi

echo "[build.sh] Stage 1/2: building elevation mapping workspace."
colcon build \
  "${COMMON_ARGS[@]}" \
  --packages-skip "${PACKAGES_TO_SKIP[@]}"

if [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
  source "${WORKSPACE_DIR}/install/setup.bash"
fi

echo "[build.sh] Stage 2/2: building plane segmentation workspace."
colcon build \
  "${COMMON_ARGS[@]}" \
  --packages-select "${PLANE_SEGMENTATION_PACKAGES[@]}"
