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

launch_arguments=()

export WILDOS_TOPIC_PROFILE=robot
launch_arguments+=(
  "do_object_search:=true"
  "log_level:=info"
  "launch_paper_rviz:=false"
  "launch_performance_monitor:=false"
  "visual_config:=wildos_nav_conf.yaml"
)

if [[ "${check_config}" == "true" ]]; then
  printf '%s\n' "${launch_arguments[@]}"
  exit 0
fi

/usr/bin/python3 "${WILDOS_REPO_ROOT}/docker/verify_runtime.py"
exec "${WILDOS_REPO_ROOT}/scripts/start_wildos_elevation.sh" \
  "${launch_arguments[@]}"
