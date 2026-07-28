#!/usr/bin/env bash
set -euo pipefail

environment_script=/etc/profile.d/wildos_ros.sh
if [[ ! -f "${environment_script}" ]]; then
  environment_script="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/ros_environment.sh"
fi
set +u
source "${environment_script}"
set -u

check_config=false
if [[ "${1:-}" == "--check-config" ]]; then
  check_config=true
  shift
elif [[ $# -gt 0 ]]; then
  exec "$@"
fi

require_variable() {
  local variable_name="$1"
  if [[ -z "${!variable_name:-}" ]]; then
    echo "Missing Orin deployment variable: ${variable_name}" >&2
    echo "Copy .env.docker.example to .env and complete it" >&2
    exit 1
  fi
}

launch_arguments=()

# Map one deployment variable to one or more ROS launch arguments
append_launch_arguments() {
  local variable_name="$1"
  local launch_name
  shift
  require_variable "${variable_name}"
  for launch_name in "$@"; do
    launch_arguments+=("${launch_name}:=${!variable_name}")
  done
}

# Orin runtime settings must come from the single .env deployment entry
for variable_name in \
  WILDOS_TOPIC_PROFILE \
  ROS_DOMAIN_ID \
  ROS_LOCALHOST_ONLY \
  RMW_IMPLEMENTATION; do
  require_variable "${variable_name}"
done

append_launch_arguments DO_OBJECT_SEARCH do_object_search
append_launch_arguments USE_SIM_TIME use_sim_time
append_launch_arguments LAUNCH_PAPER_RVIZ launch_paper_rviz
append_launch_arguments WILDOS_LOG_LEVEL log_level
append_launch_arguments LOCALIZATION_BACKEND localization_backend
append_launch_arguments LAUNCH_DLIO launch_dlio
append_launch_arguments GLOBAL_FRAME \
  global_frame parent_frame odom_parent_frame
append_launch_arguments BASE_FRAME odom_child_frame base_frame
append_launch_arguments POINTCLOUD_INPUT_TOPIC pointcloud_input_topic
append_launch_arguments ODOM_INPUT_TOPIC odom_input_topic
append_launch_arguments LIDAR_FRAME lidar_frame
append_launch_arguments PUBLISH_CAMERA_STATIC_TF publish_camera_static_tf
append_launch_arguments CAM_FRAME cam_frame
append_launch_arguments CAMERA_IMG_TOPIC camera_img_topic
append_launch_arguments CAMERA_INFO_TOPIC camera_info_topic

if [[ "${check_config}" == "true" ]]; then
  printf '%s\n' "${launch_arguments[@]}"
  exit 0
fi

/usr/bin/python3 "${WILDOS_REPO_ROOT}/docker/verify_runtime.py"
exec "${WILDOS_REPO_ROOT}/scripts/start_wildos_elevation.sh" \
  "${launch_arguments[@]}"
