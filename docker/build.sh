#!/bin/bash
set -e

echo "Checking system dependencies..."
sudo apt-get update
sudo apt-get install -y libcgal-dev

cd /home/ros/colcon_ws
source /opt/ros/$ROS_DISTRO/setup.bash

BUILD_TYPE=RelWithDebInfo


colcon build \
    --packages-select \
        elevation_mapping_cupy \
        elevation_map_msgs \
        convex_plane_decomposition \
        convex_plane_decomposition_ros \
        convex_plane_decomposition_msgs \
        grid_map_filters_rsl \
    --parallel-workers $(nproc) \
    --merge-install \
    --symlink-install \
    --event-handlers console_cohesion+ \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
        -DBUILD_TESTING=OFF \
        -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined -Wall -Wextra -Wpedantic -Wshadow -Wno-error=shadow"