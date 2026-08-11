#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
source /opt/wildos_localization_ws/install/setup.bash
set -u

readonly pointcloud_topic="${POINTCLOUD_INPUT_TOPIC:-/livox/lidar}"
readonly imu_topic="${IMU_INPUT_TOPIC:-/livox/imu}"

pgrep -f "livox_ros_driver2_node" >/dev/null

[[ "$(timeout 4 ros2 topic type "${pointcloud_topic}" --no-daemon --spin-time 1)" == "sensor_msgs/msg/PointCloud2" ]]
[[ "$(timeout 4 ros2 topic type "${imu_topic}" --no-daemon --spin-time 1)" == "sensor_msgs/msg/Imu" ]]

pointcloud_fields="$(
  timeout 4 ros2 topic echo "${pointcloud_topic}" \
    --no-daemon \
    --spin-time 1 \
    --qos-reliability best_effort \
    --once \
    --field fields
)"
grep -Eq "name(: |=)'?timestamp" <<<"${pointcloud_fields}"
timeout 4 ros2 topic echo "${imu_topic}" \
  --no-daemon \
  --spin-time 1 \
  --qos-reliability best_effort \
  --once >/dev/null
