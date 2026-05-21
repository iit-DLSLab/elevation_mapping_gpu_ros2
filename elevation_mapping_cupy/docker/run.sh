#!/bin/bash

set -euo pipefail

IMAGE_NAME="elevation_mapping_cupy_ros2:latest"
CONTAINER_NAME="elevation_mapping_package"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="${SCRIPT_DIR}/.ros-env"
CONFIG_DIR="$(realpath "${SCRIPT_DIR}/../elevation_mapping_cupy/config")"

RUN_COMMAND=(
  docker run
  --name "${CONTAINER_NAME}"
  --runtime nvidia
  --privileged
  --network host
  --tty
  --interactive
  --env-file "${ENV_FILE}"
  --env "DISPLAY=${DISPLAY:-}"
  --env "NVIDIA_DRIVER_CAPABILITIES=all"
  --env "NVIDIA_VISIBLE_DEVICES=all"
  --env "QT_X11_NO_MITSHM=1"
  --volume /tmp/.X11-unix:/tmp/.X11-unix
  --volume "${HOME}/.Xauthority:/root/.Xauthority"
  --volume /dev/bus/usb:/dev/bus/usb
  --volume "${CONFIG_DIR}:/home/ros/workspace/install/share/elevation_mapping_cupy/config"
  --device-cgroup-rule 'c 189:* rmw'
  --device-cgroup-rule 'c 81:* rmw'
  "${IMAGE_NAME}"
  # /bin/bash
  # -c
  # "source /home/ros/workspace/.map_env/bin/activate; source /home/ros/workspace/install/setup.bash;"
)

if docker ps -a --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
  echo "[run.sh] Removing existing container ${CONTAINER_NAME} to match docker compose recreate behavior."
  docker rm -f "${CONTAINER_NAME}" >/dev/null
fi

echo "[run.sh] Launching ${CONTAINER_NAME} from ${IMAGE_NAME}."
printf '[run.sh] Command:'
printf ' %q' "${RUN_COMMAND[@]}"
printf '\n'

"${RUN_COMMAND[@]}"
