#!/usr/bin/env bash
set -euo pipefail

export DEBIAN_FRONTEND=noninteractive

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run this script as root" >&2
  exit 1
fi

# ROS 2 Humble binary packages require Ubuntu 22.04 Jammy
ubuntu_codename="$(. /etc/os-release && printf '%s' "${VERSION_CODENAME:-}")"
if [[ "${ubuntu_codename}" != "jammy" ]]; then
  echo "ROS 2 Humble requires Ubuntu Jammy, actual=${ubuntu_codename:-unknown}" >&2
  exit 1
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
mirror_script="${APT_MIRROR_SCRIPT:-${script_dir}/configure_apt_mirrors.sh}"
if [[ ! -f "${mirror_script}" ]]; then
  echo "Missing APT mirror script: ${mirror_script}" >&2
  exit 1
fi

# Avoid stale package indexes from mirror synchronization or caching proxies
apt_update() {
  apt-get \
    -o Acquire::Retries=5 \
    -o Acquire::http::No-Cache=true \
    -o Acquire::https::No-Cache=true \
    -o Acquire::BrokenProxy=true \
    update
}

bash "${mirror_script}" auto
apt_update
apt-get install -y --no-install-recommends \
  ca-certificates \
  curl \
  gnupg \
  locales \
  lsb-release \
  software-properties-common

# Use HTTPS after CA certificates are available
bash "${mirror_script}" https

locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
add-apt-repository --yes --no-update universe

curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu ${ubuntu_codename} main" \
  > /etc/apt/sources.list.d/ros2.list

rm -rf /var/lib/apt/lists/*
apt_update
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
  ros-humble-rmw-fastrtps-cpp \
  ros-humble-robot-state-publisher \
  ros-humble-ros-base \
  ros-humble-rosbag2-py \
  ros-humble-rviz2 \
  ros-humble-grid-map-rviz-plugin \
  ros-humble-sensor-msgs-py \
  ros-humble-tf-transformations \
  ros-humble-tf2-eigen \
  ros-humble-tf2-geometry-msgs \
  ros-humble-tf2-sensor-msgs

rm -rf /var/lib/apt/lists/*
