#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
set -u

exec python3 /usr/local/lib/wildos-camera-healthcheck.py
