#!/bin/bash

# The Docker image name
IMAGE_NAME="elevation_mapping_cupy_ros2:latest"

# Path to your ROS workspace on the host
HOST_WORKSPACE="/home/fischer/Desktop/Fischer/iit/elevation_mapping_ros2" # Path to your ROS workspace in the container

# Define environment variables for graphical output
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]; then
    touch $XAUTH
    xauth nlist :0 | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
    chmod a+r $XAUTH
fi

echo "---------------------"
RUN_COMMAND="docker run \
  --volume=$XSOCK:$XSOCK:rw \
  --volume=$XAUTH:$XAUTH:rw \
  --env=\"QT_X11_NO_MITSHM=1\" \
  --env=\"XAUTHORITY=$XAUTH\" \
  --env=\"DISPLAY=$DISPLAY\" \
  --ulimit rtprio=99 \
  --cap-add=sys_nice \
  --privileged \
  --net=host \
  -e HOST_USERNAME=$(whoami) \
  -v ${HOST_WORKSPACE}:/home/ros/workspace/src \
  -v /media:/media \
  --gpus all \
  --user 1000:1000 \
  -it $IMAGE_NAME"

echo -e "[run.sh]: \e[1;32mThe final run command is:\n\e[0;35m$RUN_COMMAND\e[0m."
eval $RUN_COMMAND
echo -e "[run.sh]: \e[1;32mDocker terminal closed.\e[0m"

