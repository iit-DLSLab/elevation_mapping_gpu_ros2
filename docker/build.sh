#!/usr/bin/env bash
# build.sh – build the elevation_mapping Docker image.

set -euo pipefail

IMAGE_NAME="${IMAGE_NAME:-elevation_mapping:latest}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}" # jazzy, humble
RMW_NAME="${RMW_NAME:-fastrtps}" # fastrtps, cyclonedds

case "$RMW_NAME" in
    fast|fastdds|fastrtps)
        RMW_NAME=fastrtps
        ;;
    cyclone|cyclonedds)
        RMW_NAME=cyclonedds
        ;;
    *)
        echo "ERROR: Invalid RMW_NAME='$RMW_NAME'. Use one of: fastrtps, fastdds, fast, cyclonedds, cyclone."
        exit 1
        ;;
esac

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
DOCKERFILE="${SCRIPT_DIR}/Dockerfile.x64"

echo "======================================================="
echo " Building image : $IMAGE_NAME"
echo " Dockerfile     : $DOCKERFILE"
echo " Build context  : $REPO_ROOT"
echo " ROS_DISTRO     : $ROS_DISTRO"
echo " RMW_NAME       : $RMW_NAME"
echo "======================================================="

DOCKER_BUILDKIT=1 docker build \
    --file "$DOCKERFILE" \
    --target runtime \
    --tag "$IMAGE_NAME" \
    --build-arg ROS_DISTRO="$ROS_DISTRO" \
    --build-arg RMW_NAME="$RMW_NAME" \
    --build-arg USERNAME=ros \
    --build-arg USER_UID="$(id -u)" \
    --build-arg USER_GID="$(id -g)" \
    --build-arg INSTALL_EMCUPY_ROSDEPS=true \
    "$REPO_ROOT"

echo ""
echo "✓ Image '$IMAGE_NAME' built successfully."
echo "  Run it with:  ./docker/run.sh"
