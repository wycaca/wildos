#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-/home/ks-server3/han/wildos_ws}"
REPO_DIR="${REPO_DIR:-${WORKSPACE_DIR}/src/nebula2-wildos}"
VENV_DIR="${VENV_DIR:-${REPO_DIR}/.venv}"
VISUAL_NAV_BIN="${WORKSPACE_DIR}/install/visual_navigation/lib/visual_navigation"

source /opt/ros/humble/setup.bash
source "${VENV_DIR}/bin/activate"
export PYTHONNOUSERSITE=1
export PYTHONPATH="${REPO_DIR}:${PYTHONPATH:-}"

source "${WORKSPACE_DIR}/install/setup.bash"

set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-3}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

if [[ -d "${VISUAL_NAV_BIN}" ]]; then
  # ROS generated scripts can pin /usr/bin/python3, normalize them to the active venv
  sed -i '1c#!/usr/bin/env python3' "${VISUAL_NAV_BIN}"/*
fi

exec ros2 launch visual_navigation wildos_sim_launch.py "$@"
