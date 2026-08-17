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
    echo "Missing MID-360 variable: ${variable_name}" >&2
    exit 1
  fi
}

for variable_name in MID360_CONFIG_FILE; do
  require_variable "${variable_name}"
done

if [[ ! -f "${MID360_CONFIG_FILE}" ]]; then
  echo "Missing MID-360 config file: ${MID360_CONFIG_FILE}" >&2
  exit 1
fi

python3 -m json.tool "${MID360_CONFIG_FILE}" >/dev/null

exec ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -r __node:=livox_mid360_driver \
  -r /livox/lidar:=/livox/lidar \
  -r /livox/imu:=/livox/imu \
  -p xfer_format:=0 \
  -p multi_topic:=0 \
  -p data_src:=0 \
  -p publish_freq:=10.0 \
  -p output_data_type:=0 \
  -p frame_id:=lidar_link \
  -p user_config_path:="${MID360_CONFIG_FILE}"
