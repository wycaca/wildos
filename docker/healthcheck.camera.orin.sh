#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
set -u

camera_namespace="/${CAMERA_NAMESPACE:-spot1/realsense}"
for name in front left right; do
  timeout 4 ros2 topic echo \
    "${camera_namespace}/${name}/color/camera_info" \
    --once >/dev/null
done
