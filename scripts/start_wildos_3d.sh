#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_ROOT="$(cd "${REPO_ROOT}/../.." && pwd)"

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
INSTALL_SETUP="${INSTALL_SETUP:-${WORKSPACE_ROOT}/install/setup.bash}"
DEFAULT_VENV_ACTIVATE="${REPO_ROOT}/.venv/bin/activate"
if [[ ! -f "${DEFAULT_VENV_ACTIVATE}" ]]; then
  DEFAULT_VENV_ACTIVATE="${REPO_ROOT}/wildos_venv/bin/activate"
fi
VENV_ACTIVATE="${VENV_ACTIVATE:-${DEFAULT_VENV_ACTIVATE}}"
FAST_DDS_PROFILE="${FAST_DDS_PROFILE:-${REPO_ROOT}/configs/fastdds_shm_profile.xml}"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "Missing ROS setup: ${ROS_SETUP}" >&2
  exit 1
fi

source "${ROS_SETUP}"

if [[ -f "${VENV_ACTIVATE}" ]]; then
  source "${VENV_ACTIVATE}"
fi

if [[ ! -f "${INSTALL_SETUP}" ]]; then
  echo "Missing workspace setup: ${INSTALL_SETUP}" >&2
  echo "Build the workspace before launching WildOS" >&2
  exit 1
fi

source "${INSTALL_SETUP}"

set -u

INSTALL_ROOT="$(cd "$(dirname "${INSTALL_SETUP}")" && pwd)"
export PYTHONNOUSERSITE="${PYTHONNOUSERSITE:-1}"
export WILDOS_TOPIC_PROFILE="${WILDOS_TOPIC_PROFILE:-isaac}"
if [[ -n "${PYTHONPATH:-}" ]]; then
  export PYTHONPATH="${REPO_ROOT}:${PYTHONPATH}"
else
  export PYTHONPATH="${REPO_ROOT}"
fi

ROS_DOMAIN_LAUNCH_ARG=()
if [[ -n "${ROS_DOMAIN_ID:-}" ]]; then
  ROS_DOMAIN_LAUNCH_ARG=(ros_domain_id:="${ROS_DOMAIN_ID}")
fi

RMW_LAUNCH_ARG=()
if [[ -n "${RMW_IMPLEMENTATION:-}" ]]; then
  RMW_LAUNCH_ARG=(rmw_implementation:="${RMW_IMPLEMENTATION}")
fi

FAST_DDS_LAUNCH_ARG=()
if [[ -f "${FAST_DDS_PROFILE}" ]]; then
  export FASTDDS_DEFAULT_PROFILES_FILE="${FAST_DDS_PROFILE}"
  export FASTRTPS_DEFAULT_PROFILES_FILE="${FAST_DDS_PROFILE}"
  FAST_DDS_LAUNCH_ARG=(fastdds_profile:="${FAST_DDS_PROFILE}")
fi

PYTHON_BIN="${PYTHON_BIN:-$(command -v python3)}"

fix_executable_shebang() {
  local executable="$1"
  local script_path
  script_path="$(command -v "${executable}" || true)"
  if [[ -z "${script_path}" || ! -f "${script_path}" ]]; then
    return 0
  fi
  if [[ "$(head -n 1 "${script_path}")" == "#!/usr/bin/python3" ]]; then
    sed -i "1s|.*|#!${PYTHON_BIN}|" "${script_path}"
  fi
}

launch_arg_enabled() {
  local name="$1"
  local arg
  local value
  for arg in "$@"; do
    if [[ "${arg}" == "${name}:="* ]]; then
      value="${arg#*:=}"
      case "${value,,}" in
        true|1|yes|on)
          return 0
          ;;
      esac
    fi
  done
  return 1
}

ensure_installed_executable() {
  local executable="$1"
  local package="$2"
  local script_path
  script_path="$(find "${INSTALL_ROOT}" -path "*/lib/${package}/${executable}" -type f -print -quit 2>/dev/null || true)"
  if [[ -z "${script_path}" ]]; then
    echo "缺少已安装可执行脚本: ${package}/${executable}" >&2
    echo "请重新构建后再启动: colcon build --packages-select visual_navigation graph_construction --symlink-install" >&2
    exit 1
  fi
}

fix_executable_shebang "elevation_mapping_node.py"
fix_executable_shebang "wildos"
fix_executable_shebang "odom_frame_adapter"
fix_executable_shebang "object_search_goal_mux"

if launch_arg_enabled "do_object_search" "$@"; then
  ensure_installed_executable "object_search_goal_mux" "visual_navigation"
fi

exec ros2 launch graph_construction elevation_visual_navigation_sim.launch.py \
  topic_profile:="${WILDOS_TOPIC_PROFILE}" \
  wildos_python_executable:="${PYTHON_BIN}" \
  "${ROS_DOMAIN_LAUNCH_ARG[@]}" \
  "${RMW_LAUNCH_ARG[@]}" \
  "${FAST_DDS_LAUNCH_ARG[@]}" \
  "$@"
