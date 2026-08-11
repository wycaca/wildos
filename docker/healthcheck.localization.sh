#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
source /opt/wildos_localization_ws/install/setup.bash
set -u

pgrep -f "dlio_odom_node" >/dev/null
pgrep -f "dlio_tf_adapter" >/dev/null
pgrep -f "dlio_output_guard" >/dev/null

timeout 10 ros2 topic echo "${ODOM_OUTPUT_TOPIC:-/odom}" --no-daemon \
  --spin-time 5 --once \
  --qos-reliability best_effort >/dev/null
