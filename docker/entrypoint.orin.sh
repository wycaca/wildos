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
    echo "Copy .env.orin.wildos-cameras.example to .env.orin.wildos-cameras and complete it" >&2
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

# DDS settings must be identical on both deployment hosts
for variable_name in \
  ROS_DOMAIN_ID \
  ROS_LOCALHOST_ONLY \
  RMW_IMPLEMENTATION; do
  require_variable "${variable_name}"
done

append_launch_arguments DO_OBJECT_SEARCH do_object_search
append_launch_arguments WILDOS_LOG_LEVEL log_level
export WILDOS_TOPIC_PROFILE=robot
launch_arguments+=(
  "use_sim_time:=false"
  "launch_paper_rviz:=false"
  "launch_performance_monitor:=false"
  "localization_backend:=platform"
  "launch_dlio:=false"
  "publish_camera_static_tf:=false"
  "visual_config:=${WILDOS_VISUAL_CONFIG:-wildos_nav_conf.yaml}"
)

if [[ "${check_config}" == "true" ]]; then
  printf '%s\n' "${launch_arguments[@]}"
  exit 0
fi

/usr/bin/python3 "${WILDOS_REPO_ROOT}/docker/verify_runtime.py"
exec "${WILDOS_REPO_ROOT}/scripts/start_wildos_elevation.sh" \
  "${launch_arguments[@]}"
