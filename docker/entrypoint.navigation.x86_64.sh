#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
source /opt/wildos_navigation_ws/install/setup.bash
set -u

if [[ $# -gt 0 ]]; then
  exec "$@"
fi

readonly config_file="${NAVIGATION_CONFIG_FILE:-/config/navigation.yaml}"
if [[ ! -f "${config_file}" ]]; then
  echo "Missing navigation config file: ${config_file}" >&2
  exit 1
fi

exec ros2 launch wildos_navigation navigation.launch.py \
  config_file:="${config_file}"
