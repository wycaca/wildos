#!/usr/bin/env bash
set -euo pipefail

mirror_scheme="${1:-auto}"
case "${mirror_scheme}" in
  auto)
    if [[ -s /etc/ssl/certs/ca-certificates.crt ]]; then
      mirror_scheme=https
    else
      mirror_scheme=http
    fi
    ;;
  http|https)
    ;;
  *)
    echo "Unsupported mirror scheme: ${mirror_scheme}" >&2
    exit 1
    ;;
esac

if [[ ! -f /etc/os-release ]]; then
  echo "Missing operating system metadata: /etc/os-release" >&2
  exit 1
fi

os_id="$(. /etc/os-release && printf '%s' "${ID:-}")"
if [[ "${os_id}" != "ubuntu" ]]; then
  echo "APT mirror configuration requires Ubuntu, actual=${os_id:-unknown}" >&2
  exit 1
fi

ubuntu_mirror="${mirror_scheme}://mirrors.tuna.tsinghua.edu.cn/ubuntu"
ubuntu_ports_mirror="${mirror_scheme}://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports"
ros_mirror="${mirror_scheme}://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu"

mkdir -p /etc/apt/sources.list.d
source_files=(/etc/apt/sources.list)
shopt -s nullglob
source_files+=(/etc/apt/sources.list.d/*.list)
source_files+=(/etc/apt/sources.list.d/*.sources)
shopt -u nullglob

# Normalize Ubuntu and ROS repository URLs in list and deb822 formats
for source_file in "${source_files[@]}"; do
  [[ -f "${source_file}" ]] || continue
  sed -i -E \
    -e "s@https?://(archive\\.ubuntu\\.com/ubuntu|security\\.ubuntu\\.com/ubuntu|mirrors\\.tuna\\.tsinghua\\.edu\\.cn/ubuntu)@${ubuntu_mirror}@g" \
    -e "s@https?://(ports\\.ubuntu\\.com/ubuntu-ports|mirrors\\.tuna\\.tsinghua\\.edu\\.cn/ubuntu-ports)@${ubuntu_ports_mirror}@g" \
    -e "s@https?://(packages\\.ros\\.org/ros2/ubuntu|mirrors\\.tuna\\.tsinghua\\.edu\\.cn/ros2/ubuntu)@${ros_mirror}@g" \
    -e '/^[[:space:]]*deb-src[[:space:]]/d' \
    -e 's@^Types:.*deb-src.*$@Types: deb@g' \
    "${source_file}"
done

# Remove indexes downloaded from repositories used by the base image
apt-get clean
rm -rf /var/lib/apt/lists/*
