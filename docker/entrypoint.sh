#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
source /opt/wildos_ws/install/setup.bash
set -u

export INSTALL_SETUP=/opt/wildos_ws/install/setup.bash
export PYTHON_BIN=/usr/bin/python3
export PYTHONNOUSERSITE=1
export WILDOS_REPO_ROOT=/opt/wildos_ws/src/nebula2-wildos
export PYTHONPATH="${WILDOS_REPO_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
export HF_HOME="${HF_HOME:-${WILDOS_REPO_ROOT}/ckpts}"
export HF_HUB_OFFLINE="${HF_HUB_OFFLINE:-1}"
export TRANSFORMERS_OFFLINE="${TRANSFORMERS_OFFLINE:-1}"
export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/wildos-matplotlib}"

if [[ $# -gt 0 && "${1}" != *":="* ]]; then
  exec "$@"
fi

/usr/bin/python3 "${WILDOS_REPO_ROOT}/docker/verify_runtime.py"
exec "${WILDOS_REPO_ROOT}/scripts/start_wildos_elevation.sh" "$@"
