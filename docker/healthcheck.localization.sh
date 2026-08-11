#!/usr/bin/env bash
set -euo pipefail

pgrep -f "dlio_odom_node" >/dev/null
pgrep -f "dlio_tf_adapter" >/dev/null
pgrep -f "dlio_output_guard" >/dev/null
