#!/usr/bin/env bash
set -euo pipefail

export DEBIAN_FRONTEND=noninteractive

# ROS 2 Humble binary packages require Ubuntu 22.04 Jammy
ubuntu_codename="$(. /etc/os-release && printf '%s' "${VERSION_CODENAME:-}")"
if [[ "${ubuntu_codename}" != "jammy" ]]; then
  echo "ROS 2 Humble requires Ubuntu Jammy, actual=${ubuntu_codename:-unknown}" >&2
  exit 1
fi

# Replace Ubuntu Ports URLs in all supported apt source formats
replace_ubuntu_ports() {
  local mirror_url="$1"
  local source_file
  for source_file in "${source_files[@]}"; do
    [[ -f "${source_file}" ]] || continue
    sed -i -E \
      "s@https?://(ports\\.ubuntu\\.com/ubuntu-ports|mirrors\\.tuna\\.tsinghua\\.edu\\.cn/ubuntu-ports)@${mirror_url}@g" \
      "${source_file}"
  done
}

mkdir -p /etc/apt/sources.list.d/
source_files=(/etc/apt/sources.list)
shopt -s nullglob
source_files+=(/etc/apt/sources.list.d/*.list)
source_files+=(/etc/apt/sources.list.d/*.sources)
shopt -u nullglob

ubuntu_ports_mirror=https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports
if [[ ! -s /etc/ssl/certs/ca-certificates.crt ]]; then
  ubuntu_ports_mirror=http://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports
fi
replace_ubuntu_ports "${ubuntu_ports_mirror}"

# Drop stale indexes inherited from the base image before using the mirror
apt-get clean
rm -rf /var/lib/apt/lists/*
apt-get -o Acquire::Retries=5 update
apt-get install -y --no-install-recommends \
  ca-certificates \
  curl \
  gnupg \
  locales \
  lsb-release \
  software-properties-common

# Use HTTPS after CA certificates are available
replace_ubuntu_ports \
  https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports

# Replace any existing ROS 2 source before writing the canonical entry
for source_file in "${source_files[@]}"; do
  [[ -f "${source_file}" ]] || continue
  sed -i -E \
    's@https?://packages\.ros\.org/ros2/ubuntu@https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu@g' \
    "${source_file}"
done

locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
add-apt-repository --yes --no-update universe

curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu ${ubuntu_codename} main" \
  > /etc/apt/sources.list.d/ros2.list

rm -rf /var/lib/apt/lists/*
apt-get update
apt-get install -y --no-install-recommends \
  build-essential \
  cmake \
  git \
  libeigen3-dev \
  libgl1 \
  libglib2.0-0 \
  libopenblas-dev \
  libsm6 \
  libxext6 \
  libxrender1 \
  ninja-build \
  procps \
  python3-colcon-common-extensions \
  python3-dev \
  python3-opencv \
  python3-pip \
  ros-humble-cv-bridge \
  ros-humble-grid-map-msgs \
  ros-humble-grid-map-ros \
  ros-humble-image-transport \
  ros-humble-message-filters \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-rmw-fastrtps-cpp \
  ros-humble-rmw-zenoh-cpp \
  ros-humble-robot-state-publisher \
  ros-humble-ros-base \
  ros-humble-rosbag2-py \
  ros-humble-rviz2 \
  ros-humble-sensor-msgs-py \
  ros-humble-tf-transformations \
  ros-humble-tf2-eigen \
  ros-humble-tf2-geometry-msgs \
  ros-humble-tf2-sensor-msgs

rm -rf /var/lib/apt/lists/*
