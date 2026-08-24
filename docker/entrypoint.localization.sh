#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
source /opt/wildos_localization_ws/install/setup.bash
set -u

if [[ $# -gt 0 ]]; then
  exec "$@"
fi

require_variable() {
  local variable_name="$1"
  if [[ -z "${!variable_name:-}" ]]; then
    echo "Missing localization variable: ${variable_name}" >&2
    exit 1
  fi
}

for variable_name in DLIO_CONFIG_FILE; do
  require_variable "${variable_name}"
done

if [[ ! -f "${DLIO_CONFIG_FILE}" ]]; then
  echo "Missing DLIO config file: ${DLIO_CONFIG_FILE}" >&2
  exit 1
fi

exec ros2 launch graph_construction dlio_localization.launch.py \
  dlio_config_file:="${DLIO_CONFIG_FILE}" \
  output_pointcloud_rate_hz:="${OUTPUT_POINTCLOUD_RATE_HZ:-0.0}" \
  log_level:=info
