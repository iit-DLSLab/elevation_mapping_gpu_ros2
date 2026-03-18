#!/usr/bin/env bash
# run.sh – launch the elevation_mapping_cupy development container.
#
# ──────────────────────────────────────────────────────────────────────────────
# HOST_CONFIG_DIR   →  $ROS_WS/install/share/elevation_mapping_cupy/config
# ──────────────────────────────────────────────────────────────────────────────
set -euo pipefail

IMAGE_NAME="${IMAGE_NAME:-elevation_mapping:latest}"

# ── Host-side config directory ───────────────────────────────────────────────
# Set HOST_CONFIG_DIR to wherever your config/ folder lives on the host.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
HOST_CONFIG_DIR="${HOST_CONFIG_DIR:-${REPO_ROOT}/elevation_mapping_cupy/config}"

if [ ! -d "$HOST_CONFIG_DIR" ]; then
    echo "ERROR: HOST_CONFIG_DIR='$HOST_CONFIG_DIR' does not exist."
    echo "       Set HOST_CONFIG_DIR to the path of your config folder."
    exit 1
fi

# ── Container-side config path ───────────────────────────────────────────────
CONTAINER_CONFIG_DIR="/home/ros/ros_ws/install/share/elevation_mapping_cupy/config"

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
echo " Config mount : $HOST_CONFIG_DIR"
echo "            → $CONTAINER_CONFIG_DIR"
echo "======================================================="

docker run --rm -it \
    --name elevation_dev_container \
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
    \
    --volume "${HOST_CONFIG_DIR}:${CONTAINER_CONFIG_DIR}:rw" \
    \
    --user ros \
    "$IMAGE_NAME" \
    bash