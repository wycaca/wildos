#!/usr/bin/env bash
set -euo pipefail

pgrep -f "ros2 launch graph_construction elevation_visual_navigation.launch.py" >/dev/null
