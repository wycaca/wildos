#!/usr/bin/env bash
set -euo pipefail

PROFILE_NAME="wildos-agx-link"
INTERFACE="${1:-}"

if [[ -z "${INTERFACE}" || "${INTERFACE}" == "-h" || "${INTERFACE}" == "--help" ]]; then
  echo "Usage: configure_x86_agx_link.sh <x86_interface>" >&2
  exit 1
fi

ip link show dev "${INTERFACE}" >/dev/null

# 此网卡只承载 x86 与相机 AGX 的 DDS, 不设置默认路由
if nmcli connection show "${PROFILE_NAME}" >/dev/null 2>&1; then
  sudo nmcli connection modify "${PROFILE_NAME}" \
    connection.interface-name "${INTERFACE}" \
    ipv4.method manual \
    ipv4.addresses 192.168.50.1/24 \
    ipv4.gateway "" \
    ipv4.never-default yes \
    ipv6.method disabled \
    connection.autoconnect yes
else
  sudo nmcli connection add \
    type ethernet \
    ifname "${INTERFACE}" \
    con-name "${PROFILE_NAME}" \
    ipv4.method manual \
    ipv4.addresses 192.168.50.1/24 \
    ipv4.never-default yes \
    ipv6.method disabled \
    connection.autoconnect yes
fi

sudo nmcli connection up "${PROFILE_NAME}" ifname "${INTERFACE}"
ip -br address show dev "${INTERFACE}"
ping -c 3 -W 1 192.168.50.2
