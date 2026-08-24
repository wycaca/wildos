#!/usr/bin/env bash
set -euo pipefail

pgrep -f "/wildos_navigation/controller" >/dev/null
pgrep -f "/wildos_navigation/velocity_sender" >/dev/null
