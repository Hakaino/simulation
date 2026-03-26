FROM osrf/ros:jazzy-desktop-full

LABEL description="Dockerfile for a ROS 2 Jazzy quadcopter simulation workspace"

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /workspace

COPY . /workspace

RUN /bin/bash /workspace/dependencies.sh
RUN /bin/bash -lc "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

ENTRYPOINT ["/bin/bash", "/workspace/docker_entrypoint.bash"]
