#!/usr/bin/env bash
# build.sh – build the elevation_mapping Docker image.

set -euo pipefail

IMAGE_NAME="${IMAGE_NAME:-elevation_mapping:latest}"
# Default ROS 2 middleware of the image (RMW_IMPLEMENTATION=rmw_<RMW_NAME>_cpp).
RMW_NAME="${RMW_NAME:-cyclonedds}"
case "$RMW_NAME" in
    cyclonedds|fastrtps) ;;
    *) echo "ERROR: RMW_NAME must be 'cyclonedds' or 'fastrtps', got '$RMW_NAME'" >&2; exit 1 ;;
esac

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
DOCKERFILE="${SCRIPT_DIR}/Dockerfile.x64"

echo "======================================================="
echo " Building image : $IMAGE_NAME"
echo " Dockerfile     : $DOCKERFILE"
echo " Build context  : $REPO_ROOT"
echo " RMW            : rmw_${RMW_NAME}_cpp"
echo "======================================================="

DOCKER_BUILDKIT=1 docker build \
    --file "$DOCKERFILE" \
    --target runtime \
    --tag "$IMAGE_NAME" \
    --build-arg ROS_DISTRO=humble \
    --build-arg USERNAME=ros \
    --build-arg USER_UID="$(id -u)" \
    --build-arg USER_GID="$(id -g)" \
    --build-arg INSTALL_EMCUPY_ROSDEPS=true \
    --build-arg RMW_NAME="$RMW_NAME" \
    "$REPO_ROOT"

echo ""
echo "✓ Image '$IMAGE_NAME' built successfully."
echo "  Run it with:  ./docker/run.sh"