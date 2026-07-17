#!/usr/bin/env bash
set -euo pipefail

GRAAF_COMMIT="${1:?Missing graaf commit}"
GRAAF_SOURCE=/opt/wildos_deps/graaf

mkdir -p "$(dirname "${GRAAF_SOURCE}")"
git clone https://github.com/bobluppes/graaf.git "${GRAAF_SOURCE}"
git -C "${GRAAF_SOURCE}" checkout "${GRAAF_COMMIT}"

set +u
source /opt/ros/humble/setup.bash
set -u
cd /opt/wildos_ws
colcon build \
  --symlink-install \
  --event-handlers console_direct+ \
  --packages-select \
    elevation_map_msgs \
    elevation_mapping_cupy \
    graaf_vendor \
    graphnav_msgs \
    object_search_msgs \
    gps_visualization \
    triangulation3d \
    visual_navigation \
    graph_construction \
    graphnav_planner \
  --cmake-args \
    -DBUILD_TESTING=OFF \
    -DFETCHCONTENT_SOURCE_DIR_GRAAF_UPSTREAM="${GRAAF_SOURCE}"

rm -rf /opt/wildos_ws/log "${GRAAF_SOURCE}/.git"
