#!/usr/bin/env bash
# Installs what the hexapod workspace needs on Ubuntu 24.04:
# ROS 2 Jazzy (desktop), Gazebo Harmonic (through ROS vendor packages), and build/URDF tools.
# Run it yourself: bash ~/hexapod_ws/scripts/install_deps.sh
# It does not edit ~/.bashrc, and it does not run a full system upgrade.
set -euo pipefail

. /etc/os-release
if [ "${VERSION_CODENAME}" != "noble" ]; then
  echo "Expected Ubuntu 24.04 (noble), found ${VERSION_CODENAME}. Stopping." >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y git curl python3-venv

# Register the official ROS 2 apt repository (ros-apt-source).
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  tag=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | grep -F '"tag_name"' | awk -F'"' '{print $4}')
  if [ -z "${tag}" ]; then
    echo "Could not determine the latest ros-apt-source release. Stopping." >&2
    exit 1
  fi
  deb="$(mktemp -d)/ros2-apt-source.deb"
  curl -fsSL -o "${deb}" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${tag}/ros2-apt-source_${tag}.noble_all.deb"
  sudo dpkg -i "${deb}"
  sudo apt-get update
fi

# ros-jazzy-ros-gz is the Gazebo-recommended way to get Harmonic with Jazzy.
sudo apt-get install -y \
  ros-dev-tools \
  ros-jazzy-desktop \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-ros-gz \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-gz-ros2-control \
  liburdfdom-tools

# Report the result. ROS setup scripts reference unset variables, so relax -u first.
set +u
# shellcheck disable=SC1091
. /opt/ros/jazzy/setup.bash
echo "--- installed"
echo "ROS_DISTRO=${ROS_DISTRO}"
echo "git:        $(git --version)"
echo "colcon:     $(command -v colcon)"
echo "xacro:      $(command -v xacro)"
echo "check_urdf: $(command -v check_urdf)"
echo "gazebo:     $(gz sim --version 2>&1 | head -1)"
