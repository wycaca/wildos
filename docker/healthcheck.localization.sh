#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
source /opt/wildos_localization_ws/install/setup.bash
set -u

pgrep -f "dlio_odom_node" >/dev/null
pgrep -f "dlio_tf_adapter" >/dev/null
pgrep -f "dlio_output_guard" >/dev/null

timeout 4 ros2 topic echo "${ODOM_OUTPUT_TOPIC:-/odom}" --once >/dev/null
timeout 4 ros2 topic echo "${POINTCLOUD_OUTPUT_TOPIC:-/cloud_registered}" \
  --once >/dev/null
