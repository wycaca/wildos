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

for variable_name in \
  DLIO_CONFIG_FILE \
  POINTCLOUD_INPUT_TOPIC \
  IMU_INPUT_TOPIC \
  POINTCLOUD_OUTPUT_TOPIC \
  ODOM_OUTPUT_TOPIC \
  GLOBAL_FRAME \
  BASE_FRAME \
  LIDAR_FRAME \
  IMU_FRAME; do
  require_variable "${variable_name}"
done

if [[ ! -f "${DLIO_CONFIG_FILE}" ]]; then
  echo "Missing DLIO config file: ${DLIO_CONFIG_FILE}" >&2
  exit 1
fi

exec ros2 launch graph_construction dlio_localization.launch.py \
  dlio_config_file:="${DLIO_CONFIG_FILE}" \
  pointcloud_topic:="${POINTCLOUD_INPUT_TOPIC}" \
  imu_topic:="${IMU_INPUT_TOPIC}" \
  output_pointcloud_topic:="${POINTCLOUD_OUTPUT_TOPIC}" \
  output_odom_topic:="${ODOM_OUTPUT_TOPIC}" \
  global_frame:="${GLOBAL_FRAME}" \
  base_frame:="${BASE_FRAME}" \
  lidar_frame:="${LIDAR_FRAME}" \
  imu_frame:="${IMU_FRAME}" \
  log_level:=info
