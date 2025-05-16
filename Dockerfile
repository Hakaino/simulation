FROM osrf/ros:jazzy-desktop-full
LABEL description="Dockerfile for ROS 2 Jazzy on Ubuntu 24.04"
WORKDIR /workspace
COPY . /workspace
RUN sh dependencies.sh
RUN bash -c " \
    source /opt/ros/jazzy/setup.bash && \
    colcon build \
    "

# Set environment variable correctly inside container
ENV GZ_SIM_RESOURCE_PATH=/workspace/gazebo_models/
ENTRYPOINT [ "/workspace/docker_entrypoint.sh" ]