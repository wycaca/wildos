#!/usr/bin/env bash

# Load ROS and the built workspace for Orin debug shells
if [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
fi
if [[ -f /opt/wildos_ws/install/setup.bash ]]; then
  source /opt/wildos_ws/install/setup.bash
fi

export WILDOS_REPO_ROOT=/opt/wildos_ws/src/nebula2-wildos
export INSTALL_SETUP=/opt/wildos_ws/install/setup.bash
export PYTHONNOUSERSITE=1
export PYTHONPATH="${WILDOS_REPO_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
export HF_HOME="${HF_HOME:-${WILDOS_REPO_ROOT}/ckpts}"
export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/wildos-matplotlib}"

WILDOS_VENV="${WILDOS_REPO_ROOT}/.venv"
if [[ -f "${WILDOS_VENV}/bin/activate" ]] \
  && [[ "${VIRTUAL_ENV:-}" != "${WILDOS_VENV}" ]]; then
  source "${WILDOS_VENV}/bin/activate"
fi
