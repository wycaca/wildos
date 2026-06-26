#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-/home/ks-server3/han/wildos_ws}"

source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"

set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-3}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

exec ros2 launch graph_construction graph_construction_sim.launch.py "$@"
