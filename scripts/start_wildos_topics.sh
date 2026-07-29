#!/usr/bin/env bash
set -euo pipefail

readonly LOCATION_SCRIPT="/mnt/ssd/Yao_ws/script/start_location.sh"
readonly LOCATION_SESSION="lio_core"
readonly LOCATION_LOG="/tmp/wildos_location_start.log"

export ROS_DOMAIN_ID=2
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if [[ ! -f "${LOCATION_SCRIPT}" ]]; then
  echo "Missing location startup script: ${LOCATION_SCRIPT}" >&2
  exit 1
fi

# The location script creates the lio_core session before attaching to it
if tmux has-session -t "${LOCATION_SESSION}" 2>/dev/null; then
  echo "Point cloud topics are already running in tmux session ${LOCATION_SESSION}"
else
  if ! "${LOCATION_SCRIPT}" </dev/null >"${LOCATION_LOG}" 2>&1 \
    && ! tmux has-session -t "${LOCATION_SESSION}" 2>/dev/null; then
    echo "Failed to start point cloud topics, see ${LOCATION_LOG}" >&2
    exit 1
  fi
  echo "Point cloud topics started in tmux session ${LOCATION_SESSION}"
fi

echo "Point cloud topics use ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
