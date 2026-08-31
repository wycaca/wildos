#!/usr/bin/env bash
set -euo pipefail

GRAAF_COMMIT="${1:?Missing graaf commit}"
BUILD_PHASE="${2:-all}"
GRAAF_SOURCE=/opt/wildos_deps/graaf

DEPENDENCY_PACKAGES=(
  elevation_map_msgs
  elevation_mapping_cupy
  graaf_vendor
  graphnav_msgs
  object_search_msgs
  triangulation3d
)
APPLICATION_PACKAGES=(
  visual_navigation
  graph_construction
  graphnav_planner
  wildos_visualization
)

case "${BUILD_PHASE}" in
  dependencies)
    BUILD_PACKAGES=("${DEPENDENCY_PACKAGES[@]}")
    ;;
  application)
    BUILD_PACKAGES=("${APPLICATION_PACKAGES[@]}")
    ;;
  all)
    BUILD_PACKAGES=("${DEPENDENCY_PACKAGES[@]}" "${APPLICATION_PACKAGES[@]}")
    ;;
  *)
    echo "Unknown build phase: ${BUILD_PHASE}" >&2
    exit 1
    ;;
esac

if [[ "${BUILD_PHASE}" != "application" ]]; then
  mkdir -p "$(dirname "${GRAAF_SOURCE}")"
  if [[ ! -d "${GRAAF_SOURCE}" ]]; then
    git clone https://github.com/bobluppes/graaf.git "${GRAAF_SOURCE}"
  fi
  git -C "${GRAAF_SOURCE}" checkout "${GRAAF_COMMIT}"
fi

set +u
source /opt/ros/humble/setup.bash
if [[ -f /opt/wildos_ws/install/setup.bash ]]; then
  source /opt/wildos_ws/install/setup.bash
fi
set -u
cd /opt/wildos_ws
colcon build \
  --symlink-install \
  --event-handlers console_direct+ \
  --packages-select \
    "${BUILD_PACKAGES[@]}" \
  --cmake-args \
    -DBUILD_TESTING=OFF \
    -DFETCHCONTENT_SOURCE_DIR_GRAAF_UPSTREAM="${GRAAF_SOURCE}"

rm -rf /opt/wildos_ws/log
if [[ -d "${GRAAF_SOURCE}/.git" ]]; then
  rm -rf "${GRAAF_SOURCE}/.git"
fi
