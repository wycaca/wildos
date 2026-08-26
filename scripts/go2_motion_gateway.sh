#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_ROOT="$(cd -- "${REPO_ROOT}/../.." && pwd)"
SERVICE_NAME="wildos-go2-motion-gateway.service"
SUDOERS_NAME="wildos-go2-motion-gateway"
MOTION_SETUP="${MOTION_SETUP:-/home/agx/agent_ws/install/setup.bash}"

usage() {
  cat <<'EOF'
Usage: go2_motion_gateway.sh <build|install|start|stop|restart|logs|status|run>

Actions:
  build    Build the host-side velocity receiver
  install  Build and install the systemd service
  start    Start the installed service
  stop     Stop the installed service
  restart  Restart the installed service
  logs     Follow service logs
  status   Show service status
  run      Run the gateway in the foreground
EOF
}

source_workspaces() {
  set +u
  source /opt/ros/humble/setup.bash
  source "${MOTION_SETUP}"
  if [[ -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
    source "${WORKSPACE_ROOT}/install/setup.bash"
  fi
  set -u
}

build_gateway() {
  source_workspaces
  colcon --log-base "${WORKSPACE_ROOT}/log" build \
    --base-paths "${REPO_ROOT}" \
    --build-base "${WORKSPACE_ROOT}/build" \
    --install-base "${WORKSPACE_ROOT}/install" \
    --packages-select wildos_navigation \
    --symlink-install
}

[[ $# -eq 1 ]] || {
  usage >&2
  exit 1
}

case "$1" in
  build)
    build_gateway
    ;;
  install)
    build_gateway
    unit_file="$(mktemp)"
    sudoers_file="$(mktemp)"
    trap 'rm -f "${unit_file}" "${sudoers_file}"' EXIT
    sed \
      -e "s|@SERVICE_USER@|${USER}|g" \
      -e "s|@REPO_ROOT@|${REPO_ROOT}|g" \
      "${REPO_ROOT}/systemd/wildos-go2-motion-gateway.service.in" \
      > "${unit_file}"
    sed \
      -e "s|@SERVICE_USER@|${USER}|g" \
      "${REPO_ROOT}/systemd/wildos-go2-motion-gateway.sudoers.in" \
      > "${sudoers_file}"
    sudo /usr/sbin/visudo -cf "${sudoers_file}"
    sudo install -m 0644 "${unit_file}" "/etc/systemd/system/${SERVICE_NAME}"
    sudo install -m 0440 "${sudoers_file}" "/etc/sudoers.d/${SUDOERS_NAME}"
    sudo systemctl daemon-reload
    # 实机运动验收前禁止开机自启, 现场确认安全后仍使用 start 手动启动
    sudo systemctl disable "${SERVICE_NAME}"
    ;;
  start|stop|restart)
    sudo -n /usr/bin/systemctl "$1" "${SERVICE_NAME}"
    ;;
  status)
    systemctl show "${SERVICE_NAME}" \
      --property=LoadState --property=ActiveState --property=SubState
    ;;
  logs)
    journalctl -u "${SERVICE_NAME}" -n 200 -f
    ;;
  run)
    source_workspaces
    export ROS_DOMAIN_ID=0
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eno1" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>'
    exec ros2 launch wildos_navigation go2_motion_gateway.launch.py \
      config_file:="${REPO_ROOT}/wildos_navigation/config/navigation.yaml"
    ;;
  *)
    usage >&2
    exit 1
    ;;
esac
