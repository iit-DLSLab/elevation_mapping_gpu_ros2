#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="elevation_dev_new:latest"
HOST_WORKSPACE="/home/saeed/colcon_ws"

ROBOT_CONFIG_REL="${ROBOT_CONFIG_REL:-menzi/base.yaml}"

# 2. X11 / Display Setup for GUI (Rviz)
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]; then
    touch $XAUTH
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
    chmod a+r $XAUTH
fi

echo "-------------------------------------------------------"
echo "Launching Elevation Mapping Development Environment"
echo "Robot Config: $ROBOT_CONFIG_REL"
echo "-------------------------------------------------------"

# 3. The Run Command
# --rm: automatically clean up the container when you exit
# -it: interactive terminal
docker run --rm -it \
  --name elevation_dev_container \
  --privileged \
  --network host \
  --ipc host \
  --gpus all \
  --env="DISPLAY=$DISPLAY" \
  --env="XAUTHORITY=$XAUTH" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="RMW_IMPLEMENTATION=rmw_fastrtps_cpp" \
  --volume=$XSOCK:$XSOCK:rw \
  --volume=$XAUTH:$XAUTH:rw \
  --user ros \
  -v "${HOST_WORKSPACE}:/home/ros/colcon_ws" \
  "$IMAGE_NAME" \
  bash -c "source /home/ros/colcon_ws/install/setup.bash 2>/dev/null || true; \
           exec bash"