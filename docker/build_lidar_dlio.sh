#!/usr/bin/env bash
set -euo pipefail

readonly workspace=/opt/wildos_localization_ws
readonly sdk_source="${workspace}/src/Livox-SDK2"
readonly driver_source="${workspace}/src/livox_ros_driver2"
readonly sdk_build=/tmp/livox-sdk2-build

cmake \
  -S "${sdk_source}" \
  -B "${sdk_build}" \
  -DCMAKE_BUILD_TYPE=Release
cmake --build "${sdk_build}" --parallel "$(nproc)"
cmake --install "${sdk_build}"
ldconfig

cp "${driver_source}/package_ROS2.xml" "${driver_source}/package.xml"

set +u
source /opt/ros/humble/setup.bash
set -u

cd "${workspace}"
colcon build \
  --merge-install \
  --event-handlers console_direct+ \
  --packages-select \
    livox_ros_driver2 \
    direct_lidar_inertial_odometry \
    graph_construction \
  --cmake-args \
    -DBUILD_TESTING=OFF \
    -DROS_EDITION=ROS2 \
    -DDISTRO_ROS=humble

rm -rf "${sdk_build}" "${workspace}/build" "${workspace}/log"
