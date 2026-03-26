#!/usr/bin/env sh
set -e

echo "\n\nInstalling apt dependencies...................................."
apt-get update
apt-get install -y --no-install-recommends \
    libegl1 \
    libgl1 \
    libgl1-mesa-dri \
    mesa-utils \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-actuator-msgs \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-ros-gz-sim

echo "\n\nCleaning up apt cache.........................................."
rm -rf /var/lib/apt/lists/*
