FROM osrf/ros:jazzy-desktop-full
LABEL description="Dockerfile for ROS 2 Humble on Ubuntu 22.04"
RUN apt-get update && apt-get install -y \
      libgl1-mesa-dri \
      libgl1-mesa-glx \
      libegl1-mesa \
      mesa-utils \
    && rm -rf /var/lib/apt/lists/*
