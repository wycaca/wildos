#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
AGX_HOST="${AGX_HOST:-agx@192.168.50.2}"
AGX_REPO_ROOT="${AGX_REPO_ROOT:-/mnt/ssd/wildos_ws/src/nebula2-wildos}"
TARGET_TOPIC="/spot1/object_search_target"
ROS_CONTAINER="wildos-x86-localization-1"

usage() {
  cat <<'EOF'
Usage: go2_search_test.sh <start TARGET|stop|target TARGET|status|logs COMPONENT>

Actions:
  start TARGET    Start the full system safely, then publish the search target
  stop            Stop navigation first, then stop the GO2 motion gateway
  target TARGET   Change the running search target
  status          Show x86, camera AGX, and motion gateway status
  logs COMPONENT  Follow logs for lidar, localization, navigation, cameras,
                  wildos, or gateway

Run this script on the x86 host. Override AGX_HOST or AGX_REPO_ROOT when needed.
EOF
}

fail() {
  echo "$1" >&2
  exit 1
}

agx() {
  local argument command quoted
  printf -v command 'cd %q &&' "${AGX_REPO_ROOT}"
  for argument in "$@"; do
    printf -v quoted ' %q' "${argument}"
    command+="${quoted}"
  done
  ssh -t -o ConnectTimeout=5 -o StrictHostKeyChecking=accept-new "${AGX_HOST}" "${command}"
}

publish_target() {
  local target="$1"
  [[ -n "${target}" ]] || fail "Search target must not be empty"
  [[ "${target}" != *"'"* && "${target}" != *$'\n'* ]] || \
    fail "Search target must not contain a single quote or newline"
  docker exec -e "WILDOS_SEARCH_TARGET=${target}" "${ROS_CONTAINER}" bash -lc \
    'source /opt/ros/humble/setup.bash && ros2 topic pub --once /spot1/object_search_target std_msgs/msg/String "{data: '\''${WILDOS_SEARCH_TARGET}'\''}"'
}

stop_motion() {
  # Stop the velocity source first, then let the gateway timeout publish zero velocity
  "${SCRIPT_DIR}/wildos_docker.sh" x86 stop navigation || true
  sleep 1
  agx scripts/go2_motion_gateway.sh stop
}

[[ $# -ge 1 ]] || {
  usage >&2
  exit 1
}

action="$1"
shift
cd "${REPO_ROOT}"

case "${action}" in
  start)
    [[ $# -ge 1 ]] || fail "start requires a search target"
    target="$*"
    "${SCRIPT_DIR}/wildos_docker.sh" x86 start lidar localization
    agx scripts/wildos_docker.sh orin start cameras wildos
    agx scripts/go2_motion_gateway.sh start
    if ! "${SCRIPT_DIR}/wildos_docker.sh" x86 start navigation; then
      agx scripts/go2_motion_gateway.sh stop || true
      fail "Navigation failed to start, motion gateway was stopped"
    fi
    if ! publish_target "${target}"; then
      stop_motion || true
      fail "Target publication failed, motion chain was stopped"
    fi
    ;;
  stop)
    [[ $# -eq 0 ]] || fail "stop does not accept arguments"
    stop_motion
    ;;
  target)
    [[ $# -ge 1 ]] || fail "target requires a search target"
    publish_target "$*"
    ;;
  status)
    [[ $# -eq 0 ]] || fail "status does not accept arguments"
    "${SCRIPT_DIR}/wildos_docker.sh" x86 status
    agx scripts/wildos_docker.sh orin status
    agx scripts/go2_motion_gateway.sh status || true
    ;;
  logs)
    [[ $# -eq 1 ]] || fail "logs requires one component"
    case "$1" in
      lidar|localization|navigation)
        "${SCRIPT_DIR}/wildos_docker.sh" x86 logs -f "$1"
        ;;
      cameras|wildos)
        agx scripts/wildos_docker.sh orin logs -f "$1"
        ;;
      gateway)
        agx scripts/go2_motion_gateway.sh logs
        ;;
      *)
        fail "Unsupported log component: $1"
        ;;
    esac
    ;;
  -h|--help)
    usage
    ;;
  *)
    fail "Unsupported action: ${action}"
    ;;
esac
