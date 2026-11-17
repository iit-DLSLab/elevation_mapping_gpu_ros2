#!/usr/bin/env bash

# This script is meant to be sourced: never call a bare `exit`, it would close
# the caller's shell. `return` works when sourced, `exit` when executed.

echo "Setup ROS2 environment for DLS2"

if [ -n "${CONDA_PREFIX:-}" ]; then
	echo "Warning: conda environment '${CONDA_DEFAULT_ENV:-${CONDA_PREFIX}}' is active." >&2
	echo "ROS 2 tools may use conda's python instead of the system one; run 'conda deactivate' if ros2 fails." >&2
fi

ros_distro="${DLS_ROS_DISTRO:-humble}"
ros_prefix="/opt/ros/${ros_distro}"

echo "Sourcing ROS 2 setup file from: ${ros_prefix}"
if [ -r "${ros_prefix}/setup.bash" ]; then
	source "${ros_prefix}/setup.bash"
elif [ -r "${ros_prefix}/setup.sh" ]; then
	source "${ros_prefix}/setup.sh"
else
	echo "Cannot read ROS 2 setup file in: ${ros_prefix}" >&2
	return 1 2>/dev/null || exit 1
fi

if ! command -v ros2 >/dev/null 2>&1; then
	echo "ros2 command not found after sourcing ${ros_prefix}/setup.*" >&2
	echo "Check that ROS 2 ${ros_distro} is installed and provides ${ros_prefix}/bin/ros2." >&2
	return 1 2>/dev/null || exit 1
fi

ros_discovery_server=""
servers_path="${DLS_SERVERS_PATH:-/opt/dls2/include/dls2/util/messaging/servers.yaml}"

echo "Reading DLS servers from: ${servers_path}"

if [ ! -r "${servers_path}" ]; then
	echo "Cannot read DLS servers file: ${servers_path}" >&2
	echo "Please set DLS_SERVERS_PATH to a readable YAML file containing the discovery servers, or ensure the default path is correct." >&2 
	# set variable to false to avoid further processing
	ros_discovery_server=false
fi

if [ "${ros_discovery_server}" = false ]; then
	echo "Using default Discovery Servers"
	ros_discovery_server="127.0.0.1:11812;127.0.0.1:11813;127.0.0.1:11814;127.0.0.1:11818"
else
	ros_discovery_server="$(
		awk '
			{
				line = $0
				sub(/[[:space:]]*#.*/, "", line)
				if (line ~ /^[[:space:]]*ip:[[:space:]]*/) {
					ip = line
					sub(/^[[:space:]]*ip:[[:space:]]*/, "", ip)
					gsub(/["'\''[:space:]]/, "", ip)
				}
				if (line ~ /^[[:space:]]*port:[[:space:]]*/ && ip != "") {
					port = line
					sub(/^[[:space:]]*port:[[:space:]]*/, "", port)
					gsub(/["'\''[:space:]]/, "", port)
					if (server != "") {
						server = server ";"
					}
					server = server ip ":" port
					ip = ""
				}
			}
			END {
				print server
			}
		' "${servers_path}"
	)"
fi

if [ -n "${ros_discovery_server}" ]; then
	echo "Setting Middleware to Fast RTPS"
	export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

	echo "exporting ROS_DISCOVERY_SERVER=${ros_discovery_server}"
	export ROS_DISCOVERY_SERVER="${ros_discovery_server}"

	echo "exporting ROS_SUPER_CLIENT=TRUE"
	export ROS_SUPER_CLIENT=TRUE

	echo "Restarting ROS 2 daemon..."
	ros2 daemon stop
	ros2 daemon start
else
	echo "No active discovery servers found in: ${servers_path}" >&2
fi

