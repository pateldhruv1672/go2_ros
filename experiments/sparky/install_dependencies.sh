#!/usr/bin/env bash
# Intended for a normal Ubuntu 24.04 host or a GitHub ubuntu-24.04 runner.
set -euo pipefail
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 24.04 ]] || { echo 'Ubuntu 24.04 required'; exit 2; }
sudo apt-get update
sudo apt-get install -y curl ca-certificates gnupg
curl --fail --location https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  | sudo tee /usr/share/keyrings/ros-archive-keyring.gpg >/dev/null
echo 'deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main' \
  | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  ros-jazzy-ros-base ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control ros-jazzy-robot-localization ros-jazzy-ros2-controllers \
  ros-jazzy-xacro ros-jazzy-realsense2-description ros-jazzy-velodyne-description \
  python3-colcon-common-extensions python3-yaml python3-numpy python3-matplotlib
dpkg-query -W -f='${Package}\t${Version}\n' 'ros-jazzy-*'
