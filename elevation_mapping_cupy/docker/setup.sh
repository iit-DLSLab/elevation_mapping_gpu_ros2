#!/bin/bash

set -e

cd /home/ros/workspace

echo "[setup.sh] Importing repos..."
vcs import src < src/elevation_mapping_cupy/docker/src.repos --recursive -w $(($(nproc)/2))

echo "[setup.sh] Updating APT & rosdep..."
sudo apt update
rosdep update

echo "[setup.sh] Installing dependencies (skipping problematic ones)..."
rosdep install --from-paths src --ignore-src -y -r \
  --skip-keys="numpy catkin roscpp"
