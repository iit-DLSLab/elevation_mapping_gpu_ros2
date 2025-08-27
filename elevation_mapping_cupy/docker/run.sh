#!/usr/bin/env bash
set -euo pipefail

# Image paths 
IMAGE_NAME="${IMAGE_NAME:-elevation_mapping_cupy_ros2:latest}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HOST_WORKSPACE="$(cd "${SCRIPT_DIR}/.." && pwd)"     # repo root
ROS_ENV="${ROS_ENV:-$SCRIPT_DIR/../../../config/.ros_env}"

# Path inside container where repo is mounted (source tree)
PKG_IN_CONTAINER="/home/ros/workspace/src/elevation_mapping_cupy"

# RViz config from HOST
RVIZ_HOST_DIR="${RVIZ_HOST_DIR:-$HOST_WORKSPACE/elevation_mapping_cupy/rviz}"
RVIZ_FILE="${RVIZ_FILE:-elevation_mapping.rviz}"
RVIZ_CFG_IN_CONTAINER="/rviz_config/${RVIZ_FILE}"

# Select robot config (RELATIVE path under installed share/config/setups) 
# Keep default menzi/base.yaml unless you have another file INSTALLED.
ROBOT_CONFIG_REL="${ROBOT_CONFIG_REL:-menzi/base.yaml}"

# X11 
DISPLAY="${DISPLAY:-:0}"

# Command inside container 
LAUNCH_CMD=$(cat <<'BASH'
set -e
source /opt/ros/humble/setup.bash
source /home/ros/workspace/install/setup.bash 2>/dev/null || true
echo "[INFO] ROS_DOMAIN_ID=$ROS_DOMAIN_ID  RMW=$RMW_IMPLEMENTATION"

# Call your launch with the args it actually supports.
ros2 launch elevation_mapping_cupy elevation_mapping.launch.py \
  robot_config:="$ROBOT_CONFIG_REL" \
  rviz_config:="$RVIZ_CFG_IN_CONTAINER"
BASH
)

# Run container
docker run --rm -it \
  --name=elevation_mapping \
  --network host \
  --ipc host \
  --gpus all \
  --env-file "$ROS_ENV" \
  -e DISPLAY="$DISPLAY" \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,video \
  -e ROBOT_CONFIG_REL="$ROBOT_CONFIG_REL" \
  -e RVIZ_CFG_IN_CONTAINER="$RVIZ_CFG_IN_CONTAINER" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -v "${HOST_WORKSPACE}:${PKG_IN_CONTAINER}:ro" \
  -v "${RVIZ_HOST_DIR}:/rviz_config:ro" \
  "$IMAGE_NAME" bash -lc "$LAUNCH_CMD"

