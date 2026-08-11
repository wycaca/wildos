#!/usr/bin/env bash
set -euo pipefail

dlio_pid="$(pgrep -o -f "/dlio_odom_node")"
pgrep -f "dlio_tf_adapter" >/dev/null
pgrep -f "dlio_output_guard" >/dev/null

if grep -q "^sock_alloc_send_pskb$" /proc/"${dlio_pid}"/task/*/wchan; then
  echo "DLIO DDS sender is blocked" >&2
  exit 1
fi
