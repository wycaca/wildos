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
OBJECT_SEARCH_BUILD_PACKAGES="object_search_msgs triangulation3d visual_navigation graph_construction"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "缺少 ROS 环境文件: ${ROS_SETUP}" >&2
  exit 1
fi

source "${ROS_SETUP}"

if [[ -f "${VENV_ACTIVATE}" ]]; then
  source "${VENV_ACTIVATE}"
fi

if [[ ! -f "${INSTALL_SETUP}" ]]; then
  echo "缺少工作空间环境文件: ${INSTALL_SETUP}" >&2
  echo "请先构建工作空间后再启动 WildOS elevation 后端" >&2
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
export WILDOS_REPO_ROOT="${REPO_ROOT}"

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
    print_object_search_rebuild_hint
    exit 1
  fi
}

print_object_search_rebuild_hint() {
  echo "请重新构建后再启动:" >&2
  echo "  cd ${WORKSPACE_ROOT}" >&2
  echo "  colcon build --packages-select ${OBJECT_SEARCH_BUILD_PACKAGES} --symlink-install" >&2
}

ensure_object_search_interfaces() {
  if ros2 interface show object_search_msgs/msg/TargetEstimate >/dev/null 2>&1; then
    return 0
  fi

  echo "已安装的 object_search_msgs 缺少 TargetEstimate, 当前 install 与源码不一致" >&2
  print_object_search_rebuild_hint
  exit 1
}

if ! ros2 pkg prefix elevation_mapping_cupy >/dev/null 2>&1; then
  echo "缺少 elevation_mapping_cupy, 无法启动论文一致的 2.5D GridMap 后端" >&2
  echo "请确认工作空间已包含并构建 elevation_mapping_cupy" >&2
  exit 1
fi

fix_executable_shebang "elevation_mapping_node.py"
fix_executable_shebang "wildos"
fix_executable_shebang "odom_frame_adapter"
fix_executable_shebang "object_search_goal_mux"
fix_executable_shebang "object_target_fusion"

if launch_arg_enabled "do_object_search" "$@"; then
  ensure_object_search_interfaces
  ensure_installed_executable "object_search_goal_mux" "visual_navigation"
  ensure_installed_executable "object_target_fusion" "visual_navigation"
fi

echo "启动 WildOS elevation/2.5D, profile=${WILDOS_TOPIC_PROFILE}, python=${PYTHON_BIN}"

exec ros2 launch graph_construction elevation_visual_navigation_sim.launch.py \
  topic_profile:="${WILDOS_TOPIC_PROFILE}" \
  wildos_python_executable:="${PYTHON_BIN}" \
  "${ROS_DOMAIN_LAUNCH_ARG[@]}" \
  "${RMW_LAUNCH_ARG[@]}" \
  "${FAST_DDS_LAUNCH_ARG[@]}" \
  "$@"
