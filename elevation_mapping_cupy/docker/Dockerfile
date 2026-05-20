# Build from the repository root:
#   docker build -f elevation_mapping_cupy/docker/Dockerfile -t elevation_mapping_cupy_ros2:jetson .
#
# Cross-build/check from a non-Jetson host:
#   docker buildx build --platform linux/arm64 -f elevation_mapping_cupy/docker/Dockerfile -t elevation_mapping_cupy_ros2:jetson .
#
# Runtime on Jetson:
#   docker run --rm -it --runtime nvidia --network host --privileged elevation_mapping_cupy_ros2:jetson

ARG L4T_BASE=nvcr.io/nvidia/l4t-jetpack:r36.4.0
FROM ${L4T_BASE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=humble
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000
ARG WORKSPACE=/home/ros/workspace

ENV LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=${ROS_DISTRO} \
    ROS_ROOT=/opt/ros/${ROS_DISTRO} \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all \
    CUDA_HOME=/usr/local/cuda \
    PATH=/usr/local/cuda/bin:${PATH} \
    LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH}

RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates \
        curl \
        gnupg \
        locales \
        lsb-release \
        software-properties-common \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

RUN add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo ${UBUNTU_CODENAME}) main" \
        > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        gfortran \
        libboost-all-dev \
        libeigen3-dev \
        libgeos-dev \
        libgmp-dev \
        libmpfr-dev \
        libopencv-dev \
        pybind11-dev \
        python3-dev \
        python3-pip \
        python3-venv \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        python3-argcomplete \
        python3-empy \
        python3-matplotlib \
        python3-numpy \
        python3-opencv \
        python3-scipy \
        python3-shapely \
        python3-skimage \
        python3-sklearn \
        python3-transforms3d \
        python3-yaml \
        ros-${ROS_DISTRO}-ros-base \
        ros-${ROS_DISTRO}-ament-cmake-gtest \
        ros-${ROS_DISTRO}-ament-cmake-python \
        ros-${ROS_DISTRO}-cv-bridge \
        ros-${ROS_DISTRO}-grid-map \
        ros-${ROS_DISTRO}-image-transport \
        ros-${ROS_DISTRO}-message-filters \
        ros-${ROS_DISTRO}-pcl-ros \
        ros-${ROS_DISTRO}-point-cloud-transport \
        ros-${ROS_DISTRO}-pybind11-vendor \
        ros-${ROS_DISTRO}-python-cmake-module \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-tf-transformations \
        ros-${ROS_DISTRO}-tf2-eigen \
        ros-${ROS_DISTRO}-tf2-ros \
        ros-${ROS_DISTRO}-turtlebot3-msgs \
        ros-${ROS_DISTRO}-turtlebot3-teleop \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir --upgrade pip wheel \
    && python3 -m pip install --no-cache-dir \
        "numpy<2.0" \
        cupy-cuda12x \
        networkx==3.0 \
        ruamel.yaml \
        simple-parsing

RUN rosdep init || true \
    && rosdep update --rosdistro ${ROS_DISTRO}

RUN if ! getent group ${USER_GID} >/dev/null; then groupadd --gid ${USER_GID} ${USERNAME}; fi \
    && if ! id -u ${USERNAME} >/dev/null 2>&1; then useradd --uid ${USER_UID} --gid ${USER_GID} --create-home --shell /bin/bash ${USERNAME}; fi \
    && mkdir -p ${WORKSPACE}/src \
    && chown -R ${USER_UID}:${USER_GID} /home/${USERNAME}

WORKDIR ${WORKSPACE}

COPY elevation_mapping_cupy/docker/src.repos /tmp/src.repos
RUN vcs import src < /tmp/src.repos

COPY elevation_mapping_cupy ${WORKSPACE}/src/elevation_mapping_cupy

RUN rosdep install --from-paths \
        src/ros2_numpy \
        src/elevation_mapping_cupy/elevation_map_msgs \
        src/elevation_mapping_cupy/elevation_mapping_cupy \
        src/elevation_mapping_cupy/plane_segmentation/cgal5_ament \
        src/elevation_mapping_cupy/plane_segmentation/convex_plane_decomposition_msgs \
        src/elevation_mapping_cupy/plane_segmentation/grid_map_filters_rsl \
        --ignore-src \
        --rosdistro ${ROS_DISTRO} \
        --skip-keys "cupy-cuda12x numpy_lessthan_2 simple-parsing" \
        -y

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build \
        --merge-install \
        --symlink-install \
        --event-handlers console_cohesion+ \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=RelWithDebInfo \
            -DCMAKE_EXPORT_COMPILE_COMMANDS=On \
            -DBUILD_TESTING=OFF \
        --packages-skip \
            convex_plane_decomposition \
            convex_plane_decomposition_ros \
            semantic_sensor

RUN printf '%s\n' \
        '#!/bin/bash' \
        'set -e' \
        "source /opt/ros/${ROS_DISTRO}/setup.bash" \
        "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" \
        'exec "$@"' \
        > /ros_entrypoint.sh \
    && chmod +x /ros_entrypoint.sh \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc \
    && echo "source ${WORKSPACE}/install/setup.bash" >> /etc/bash.bashrc \
    && chown -R ${USER_UID}:${USER_GID} ${WORKSPACE}

USER ${USERNAME}
WORKDIR ${WORKSPACE}
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
	