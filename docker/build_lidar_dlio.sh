#!/usr/bin/env bash
set -euo pipefail

readonly workspace=/opt/wildos_localization_ws
readonly sdk_source="${workspace}/src/Livox-SDK2"
readonly driver_source="${workspace}/src/livox_ros_driver2"
readonly dlio_source="${workspace}/src/direct_lidar_inertial_odometry"
readonly sdk_build=/tmp/livox-sdk2-build

# sensor_msgs/Imu 要求加速度单位为 m/s²
sed -i \
  -e '/void Lddc::InitImuMsg/a\  constexpr double kStandardGravity = 9.80665;' \
  -e 's/imu_msg.linear_acceleration.x = imu_data.acc_x;/imu_msg.linear_acceleration.x = imu_data.acc_x * kStandardGravity;/' \
  -e 's/imu_msg.linear_acceleration.y = imu_data.acc_y;/imu_msg.linear_acceleration.y = imu_data.acc_y * kStandardGravity;/' \
  -e 's/imu_msg.linear_acceleration.z = imu_data.acc_z;/imu_msg.linear_acceleration.z = imu_data.acc_z * kStandardGravity;/' \
  "${driver_source}/src/lddc.cpp"
grep -Fq \
  'imu_msg.linear_acceleration.z = imu_data.acc_z * kStandardGravity;' \
  "${driver_source}/src/lddc.cpp"

git -C "${dlio_source}" apply /tmp/dlio-livox-timestamp-precision.patch
git -C "${dlio_source}" apply /tmp/dlio-publish-path-on-demand.patch
grep -Fq \
  'this->prev_imu_stamp = imu_stamp_secs;' \
  "${dlio_source}/src/dlio/odom.cc"
grep -Fq \
  'if (this->path_pub->get_subscription_count() == 0) { return; }' \
  "${dlio_source}/src/dlio/odom.cc"
grep -Fq \
  'for (int i = 0; i < this->original_scan->points.size(); i++) {' \
  "${dlio_source}/src/dlio/odom.cc"
grep -Fq \
  'if (original_scan_->empty()) { return; }' \
  "${dlio_source}/src/dlio/odom.cc"

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
