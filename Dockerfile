FROM osrf/ros:jazzy-desktop-full
LABEL description="Dockerfile for ROS 2 Jazzy on Ubuntu 24.04"
WORKDIR /workspace
RUN apt-get update && apt-get install -y \
  libegl1 \
  libgl1 \
  libgl1-mesa-dri \
  mesa-utils \
  && rm -rf /var/lib/apt/lists/*

# Set environment variable correctly inside container
ENV GZ_SIM_RESOURCE_PATH=/workspace/gazebo_models/
