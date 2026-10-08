#!/usr/bin/env bash
# Install system dependencies only. Building and running do not require root.
set -euo pipefail
with_video=false
for arg in "$@"; do
  case "$arg" in
    --with-video) with_video=true ;;
    *) echo "Usage: bash scripts/install_deps.sh [--with-video]" >&2; exit 2 ;;
  esac
done
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 22.04 ]]; then
  echo 'Supported environment: Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress.' >&2; exit 1
fi
if [[ $(dpkg --print-architecture) != amd64 ]]; then
  echo 'This project installer is validated for Ubuntu 22.04 amd64.' >&2; exit 1
fi
ros_apt_url="${ROS_APT_BASE_URL:-https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu}"
if [[ ! "$ros_apt_url" =~ ^https://[a-zA-Z0-9./_-]+$ ]]; then
  echo 'ROS_APT_BASE_URL must be an HTTPS URL without credentials or query parameters.' >&2; exit 2
fi
if [[ $EUID != 0 ]]; then
  exec sudo env ROS_APT_BASE_URL="$ros_apt_url" bash "$0" "$@"
fi
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y --no-install-recommends curl ca-certificates gnupg locales software-properties-common
locale-gen en_US.UTF-8
add-apt-repository -y universe
temp_dir=$(mktemp -d)
trap 'rm -f "$temp_dir/ros.key"; rmdir "$temp_dir"' EXIT
curl --fail --location --retry 3 https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o "$temp_dir/ros.key"
gpg --dearmor --yes -o /usr/share/keyrings/ros-archive-keyring.gpg "$temp_dir/ros.key"
printf 'deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] %s jammy main\n' "$ros_apt_url" > /etc/apt/sources.list.d/ros2.list
apt-get update
packages=(
  ros-humble-ros-base ros-humble-rviz2 ros-humble-ros-gz ros-humble-xacro
  ros-humble-robot-state-publisher ros-humble-tf2-ros ros-humble-sensor-msgs-py
  python3-colcon-common-extensions python3-numpy python3-yaml python3-pytest
  git libgl1-mesa-dri mesa-utils util-linux xvfb xauth
)
if "$with_video"; then packages+=(ffmpeg fonts-dejavu-core x11-utils); fi
apt-get install -y --no-install-recommends "${packages[@]}"
apt-get clean
echo 'Dependencies installed. Run build.sh and run.sh as your ordinary user.'
