#!/usr/bin/env bash
set -euo pipefail

driver_pid="$(pgrep -o -f "/livox_ros_driver2_node")"

if grep -q "^sock_alloc_send_pskb$" /proc/"${driver_pid}"/task/*/wchan; then
  echo "Livox DDS sender is blocked" >&2
  exit 1
fi
