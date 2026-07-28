#!/usr/bin/env bash
set -euo pipefail

readonly ROS_SETUP="/opt/ros/humble/setup.bash"
readonly LOCATION_SCRIPT="/mnt/ssd/Yao_ws/script/start_location.sh"
readonly CAMERA_WORKSPACE="/mnt/ssd/Yao_ws/bridge_ws"
readonly CAMERA_SETUP="${CAMERA_WORKSPACE}/install/setup.bash"
readonly LOCATION_SESSION="lio_core"
readonly CAMERA_SESSION="wildos_cameras"
readonly LOCATION_LOG="/tmp/wildos_location_start.log"

# Use one ROS 2 communication domain for every input source
export ROS_DOMAIN_ID=2
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

require_file() {
  local path="$1"
  if [[ ! -f "${path}" ]]; then
    echo "Missing required file: ${path}" >&2
    exit 1
  fi
}

# Validate dependencies before starting any process
require_file "${ROS_SETUP}"
require_file "${LOCATION_SCRIPT}"
require_file "${CAMERA_SETUP}"

# The location script creates the lio_core session before attaching to it
if tmux has-session -t "${LOCATION_SESSION}" 2>/dev/null; then
  echo "Location topics are already running in tmux session ${LOCATION_SESSION}"
else
  if ! "${LOCATION_SCRIPT}" </dev/null >"${LOCATION_LOG}" 2>&1 \
    && ! tmux has-session -t "${LOCATION_SESSION}" 2>/dev/null; then
    echo "Failed to start location topics, see ${LOCATION_LOG}" >&2
    exit 1
  fi
  echo "Location topics started in tmux session ${LOCATION_SESSION}"
fi

# Keep the camera bridge alive in a detached tmux session
if tmux has-session -t "${CAMERA_SESSION}" 2>/dev/null; then
  echo "Camera topics are already running in tmux session ${CAMERA_SESSION}"
else
  tmux new-session -d -s "${CAMERA_SESSION}" -n cameras \
    "bash -lc 'export ROS_DOMAIN_ID=2 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; source ${ROS_SETUP}; source ${CAMERA_SETUP}; cd ${CAMERA_WORKSPACE}; exec ros2 launch ros_bridge rtsp_cameras_launch.py'"
  echo "Camera topics started in tmux session ${CAMERA_SESSION}"
fi

echo "WildOS input topics use ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
