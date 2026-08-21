#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TEST_DIR="$(mktemp -d)"
CALLS_FILE="${TEST_DIR}/calls"

cleanup() {
  rm -rf "${TEST_DIR}"
}
trap cleanup EXIT

ip() {
  return 0
}

nmcli() {
  if [[ "$*" == "connection show wildos-agx-link" ]]; then
    return 1
  fi
  printf '%s\n' "$*" >> "${CALLS_FILE}"
}

sudo() {
  "$@"
}

ping() {
  return 0
}

export -f ip nmcli sudo ping
export CALLS_FILE

bash "${SCRIPT_DIR}/configure_x86_agx_link.sh" enx-test

rg -q -Fx 'connection add type ethernet ifname enx-test con-name wildos-agx-link ipv4.method manual ipv4.addresses 192.168.50.1/24 ipv4.never-default yes ipv6.method disabled connection.autoconnect yes' "${CALLS_FILE}"
rg -q -Fx 'connection up wildos-agx-link ifname enx-test' "${CALLS_FILE}"
