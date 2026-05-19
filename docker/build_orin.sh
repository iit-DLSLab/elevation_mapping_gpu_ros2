#!/usr/bin/env bash
# build_orin.sh - build the Jetson/Orin elevation_mapping Docker image.

set -euo pipefail

IMAGE_NAME="${IMAGE_NAME:-elevation_mapping:orin}"
ROS_DISTRO="${ROS_DISTRO:-humble}" # humble, jazzy
RMW_NAME="${RMW_NAME:-fastrtps}" # fastrtps, cyclonedds
ORIN_HUMBLE_BASE="${ORIN_HUMBLE_BASE:-base_image:l4t-r36.4.3}"
ORIN_JAZZY_BASE="${ORIN_JAZZY_BASE:-base_image:l4t-r36.4.3}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
DOCKERFILE="${SCRIPT_DIR}/Dockerfile.orin"

echo "======================================================="
echo " Building image   : $IMAGE_NAME"
echo " Dockerfile       : $DOCKERFILE"
echo " Build context    : $REPO_ROOT"
echo " ROS_DISTRO       : $ROS_DISTRO"
echo " RMW_NAME         : $RMW_NAME"
echo " ORIN_HUMBLE_BASE : $ORIN_HUMBLE_BASE"
echo " ORIN_JAZZY_BASE  : $ORIN_JAZZY_BASE"
echo "======================================================="

DOCKER_BUILDKIT=1 docker build \
    --file "$DOCKERFILE" \
    --target runtime \
    --tag "$IMAGE_NAME" \
    --build-arg ROS_DISTRO="$ROS_DISTRO" \
    --build-arg RMW_NAME="$RMW_NAME" \
    --build-arg ORIN_HUMBLE_BASE="$ORIN_HUMBLE_BASE" \
    --build-arg ORIN_JAZZY_BASE="$ORIN_JAZZY_BASE" \
    --build-arg USERNAME=ros \
    --build-arg USER_UID="$(id -u)" \
    --build-arg USER_GID="$(id -g)" \
    --build-arg INSTALL_EMCUPY_ROSDEPS=true \
    "$REPO_ROOT"

echo ""
echo "✓ Image '$IMAGE_NAME' built successfully."
echo "  Run it with: ./docker/run.sh"
