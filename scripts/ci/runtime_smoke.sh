#!/usr/bin/env bash

set -euo pipefail

print_description() {
  local package=$1
  local launch_file=$2
  echo "=== launch description: ${package} ${launch_file} ==="
  ros2 launch "${package}" "${launch_file}" --print-description >/dev/null
}

assert_launch_stays_up() {
  local package=$1
  local launch_file=$2
  local timeout_seconds=${3:-15}

  echo "=== runtime smoke: ${package} ${launch_file} (${timeout_seconds}s) ==="
  set +e
  timeout "${timeout_seconds}" ros2 launch "${package}" "${launch_file}"
  local status=$?
  set -e

  if [[ ${status} -ne 124 ]]; then
    echo "Launch '${package} ${launch_file}' exited before timeout with status ${status}." >&2
    exit "${status}"
  fi
}

print_description semantic_sensor semantic_image.launch.py
print_description semantic_sensor semantic_pointcloud.launch.py
print_description elevation_mapping_cupy turtlesim_semantic_image_example.launch.py
print_description elevation_mapping_cupy turtlesim_semantic_pointcloud_example.launch.py

assert_launch_stays_up semantic_sensor semantic_image.launch.py 15
assert_launch_stays_up semantic_sensor semantic_pointcloud.launch.py 15
