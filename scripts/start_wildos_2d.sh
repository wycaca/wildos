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

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "缺少 ROS 环境文件: ${ROS_SETUP}" >&2
  exit 1
fi

source "${ROS_SETUP}"

if [[ ! -f "${INSTALL_SETUP}" ]]; then
  echo "缺少工作空间环境文件: ${INSTALL_SETUP}" >&2
  echo "请先构建工作空间后再启动 WildOS" >&2
  exit 1
fi

source "${INSTALL_SETUP}"

if [[ -f "${VENV_ACTIVATE}" ]]; then
  source "${VENV_ACTIVATE}"
else
  echo "未找到 Python venv: ${VENV_ACTIVATE}" >&2
  echo "将继续使用当前 python, 如果缺少 omegaconf 等依赖, 请先安装 requirements.txt" >&2
fi

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

PYTHON_BIN="${PYTHON_BIN:-$(command -v python3)}"

ensure_python_module() {
  local module="$1"
  if ! "${PYTHON_BIN}" -c "import ${module}" >/dev/null 2>&1; then
    echo "当前 Python 缺少模块: ${module}" >&2
    echo "当前 Python: ${PYTHON_BIN}" >&2
    echo "请确认已激活项目 venv 或已安装 requirements.txt" >&2
    exit 1
  fi
}

fix_executable_shebang() {
  local executable="$1"
  local script_path
  script_path="$(command -v "${executable}" || true)"
  if [[ -z "${script_path}" ]]; then
    script_path="$(find "${INSTALL_ROOT}" -path "*/lib/*/${executable}" -type f -print -quit 2>/dev/null || true)"
  fi
  if [[ -z "${script_path}" || ! -f "${script_path}" ]]; then
    echo "未找到可执行脚本, 跳过 shebang 修复: ${executable}" >&2
    return 0
  fi
  if [[ "$(head -n 1 "${script_path}")" != "#!${PYTHON_BIN}" ]]; then
    sed -i "1s|.*|#!${PYTHON_BIN}|" "${script_path}"
    echo "已修正 Python 启动器: ${script_path}"
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

ensure_python_module "omegaconf"
ensure_python_module "explorfm"
fix_executable_shebang "wildos"
fix_executable_shebang "odom_frame_adapter"
fix_executable_shebang "object_search_goal_mux"

if launch_arg_enabled "do_object_search" "$@"; then
  ensure_installed_executable "object_search_goal_mux" "visual_navigation"
fi

echo "启动 WildOS 2D, profile=${WILDOS_TOPIC_PROFILE}, python=${PYTHON_BIN}"

exec ros2 launch graph_construction wildos_2d_sim.launch.py \
  topic_profile:="${WILDOS_TOPIC_PROFILE}" \
  wildos_python_executable:="${PYTHON_BIN}" \
  "${ROS_DOMAIN_LAUNCH_ARG[@]}" \
  "${RMW_LAUNCH_ARG[@]}" \
  "$@"
