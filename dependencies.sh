#!/usr/bin/env sh
set -e

echo "\n\nInstalling apt dependencies...................................."
apt-get update
apt-get install -y --no-install-recommends \
    libopencv-dev \
    libegl1 \
    libgl1 \
    libgl1-mesa-dri \
    mesa-utils \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-actuator-msgs \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-ros-gz-sim \
    libasio-dev \
    libgeographiclib-dev \
    geographiclib-tools \
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-eigen-stl-containers \
    ros-${ROS_DISTRO}-geographic-msgs \
    ros-${ROS_DISTRO}-marine-acoustic-msgs \
    ros-${ROS_DISTRO}-mavlink \
    wget

echo "\n\nInstalling PX4 SITL build dependencies............................"
RUNS_IN_DOCKER=true /bin/bash /workspace/src/PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx

echo "\n\nInstalling GeographicLib datasets for MAVROS......................"
bash /workspace/src/mavros_src/mavros/scripts/install_geographiclib_datasets.sh

echo "\n\nCleaning up apt cache.........................................."
rm -rf /var/lib/apt/lists/*
