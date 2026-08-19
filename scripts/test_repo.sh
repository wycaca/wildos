#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
if [[ -n "${WILDOS_WS_ROOT:-}" ]]; then
  WORKSPACE_ROOT="${WILDOS_WS_ROOT}"
elif [[ "$(basename -- "$(dirname -- "${REPO_ROOT}")")" == "src" ]]; then
  WORKSPACE_ROOT="$(cd -- "${REPO_ROOT}/../.." && pwd)"
else
  WORKSPACE_ROOT="${REPO_ROOT}/.colcon"
fi
ROS_SETUP="/opt/ros/humble/setup.bash"
VENV_PYTHON="${REPO_ROOT}/.venv/bin/python"

run_stage() {
  local name="$1"
  shift
  echo "[test] ${name}"
  "$@"
}

test -f "${ROS_SETUP}"
test -x "${VENV_PYTHON}"
set +u
source "${ROS_SETUP}"
set -u

python_version="$(${VENV_PYTHON} -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
test "${python_version}" = "3.10"

mkdir -p /tmp/ros-log-nebula2 /tmp/matplotlib-nebula2
mkdir -p "${WORKSPACE_ROOT}"
export ROS_LOG_DIR=/tmp/ros-log-nebula2
export MPLCONFIGDIR=/tmp/matplotlib-nebula2
export PYTHONDONTWRITEBYTECODE=1

run_stage "package manifests" \
  "${VENV_PYTHON}" "${SCRIPT_DIR}/check_package_manifests.py"

run_stage "build" \
  colcon --log-base "${WORKSPACE_ROOT}/log" build \
    --base-paths "${REPO_ROOT}" \
    --build-base "${WORKSPACE_ROOT}/build" \
    --install-base "${WORKSPACE_ROOT}/install" \
    --packages-select \
      graphnav_msgs object_search_msgs triangulation3d \
      graph_construction visual_navigation graphnav_planner \
    --cmake-args -DBUILD_TESTING=ON

set +u
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u

run_stage "graph_construction functional tests" \
  "${VENV_PYTHON}" -m pytest \
    "${REPO_ROOT}/graph_construction/test" \
    -p no:cacheprovider

run_stage "visual_navigation functional tests" \
  "${VENV_PYTHON}" -m pytest \
    "${REPO_ROOT}/visual_navigation/test" \
    -p no:cacheprovider \
    --ignore="${REPO_ROOT}/visual_navigation/test/test_copyright.py" \
    --ignore="${REPO_ROOT}/visual_navigation/test/test_flake8.py" \
    --ignore="${REPO_ROOT}/visual_navigation/test/test_pep257.py"

run_stage "triangulation3d functional tests" \
  "${VENV_PYTHON}" -m pytest \
    "${REPO_ROOT}/triangulation3d/test" \
    -p no:cacheprovider \
    --ignore="${REPO_ROOT}/triangulation3d/test/test_copyright.py" \
    --ignore="${REPO_ROOT}/triangulation3d/test/test_flake8.py" \
    --ignore="${REPO_ROOT}/triangulation3d/test/test_pep257.py"

run_stage "Planner CTest" \
  ctest --test-dir "${WORKSPACE_ROOT}/build/graphnav_planner" \
    --output-on-failure -R '^test_'

run_stage "Docker contract tests" bash "${SCRIPT_DIR}/test_wildos_docker.sh"

echo "[test] all functional test stages passed"
