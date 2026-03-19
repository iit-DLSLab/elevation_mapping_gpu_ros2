#!/usr/bin/env bash
# run.sh – launch the elevation_mapping container.

set -euo pipefail

IMAGE_NAME="${IMAGE_NAME:-elevation_mapping:latest}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# ── elevation_mapping_cupy config ────────────────────────────────────────────
HOST_EM_CONFIG="${REPO_ROOT}/elevation_mapping_cupy/config"
CONTAINER_EM_CONFIG="/home/ros/ros_ws/install/share/elevation_mapping_cupy/config"

# ── convex_plane_decomposition_ros config ────────────────────────────────────
HOST_CPD_CONFIG="${REPO_ROOT}/plane_segmentation_ros2/convex_plane_decomposition_ros/config"
CONTAINER_CPD_CONFIG="/home/ros/ros_ws/install/share/convex_plane_decomposition_ros/config"

# ── Validate host paths ──────────────────────────────────────────────────────
for dir in "$HOST_EM_CONFIG" "$HOST_CPD_CONFIG"; do
    if [ ! -d "$dir" ]; then
        echo "ERROR: Config directory not found: $dir"
        exit 1
    fi
done

# ── X11 / Display setup for GUI (RViz) ──────────────────────────────────────
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth

if [ ! -f "$XAUTH" ]; then
    touch "$XAUTH"
    xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f "$XAUTH" nmerge -
    chmod a+r "$XAUTH"
fi

echo "======================================================="
echo " Elevation Mapping Container"
echo " Image        : $IMAGE_NAME"
echo " Config mounts:"
echo "   $HOST_EM_CONFIG"
echo "     → $CONTAINER_EM_CONFIG"
echo "   $HOST_CPD_CONFIG"
echo "     → $CONTAINER_CPD_CONFIG"
echo "======================================================="

docker run --rm -it \
    --name elevation_mapping_container \
    --privileged \
    --network host \
    --ipc host \
    --gpus all \
    \
    --env DISPLAY="$DISPLAY" \
    --env XAUTHORITY="$XAUTH" \
    --env QT_X11_NO_MITSHM=1 \
    --env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    --env ROS_DOMAIN_ID=1 \
    --env ROS_LOCALHOST_ONLY=0 \
    \
    --volume "$XSOCK:$XSOCK:rw" \
    --volume "$XAUTH:$XAUTH:rw" \
    --volume "${HOST_EM_CONFIG}:${CONTAINER_EM_CONFIG}:rw" \
    --volume "${HOST_CPD_CONFIG}:${CONTAINER_CPD_CONFIG}:rw" \
    \
    --user ros \
    "$IMAGE_NAME" \
    bash