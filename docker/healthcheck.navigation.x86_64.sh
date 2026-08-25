#!/usr/bin/env bash
set -euo pipefail

pgrep -f "/wildos_navigation/map_pub" >/dev/null
pgrep -f "/wildos_navigation/astar" >/dev/null
pgrep -f "/wildos_navigation/controller" >/dev/null
pgrep -f "/wildos_navigation/velocity_sender" >/dev/null
