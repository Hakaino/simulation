FROM osrf/ros:jazzy-desktop-full

LABEL description="Dockerfile for a ROS 2 Jazzy quadcopter simulation workspace"

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /workspace

COPY . /workspace

# Clone MAVROS from source — the pre-built apt packages (mavros-msgs Apr 2026) reference
# fastcdr::Cdr::serialize(uint8_t) which was removed in the installed fastcdr 2.2.5.
# Building from source compiles against the fastcdr that is actually installed.
RUN git clone --depth 1 --branch 2.14.0 https://github.com/mavlink/mavros.git /workspace/src/mavros_src \
    || git clone --depth 1 --branch ros2 https://github.com/mavlink/mavros.git /workspace/src/mavros_src

RUN /bin/bash /workspace/dependencies.sh
RUN /bin/bash -lc "cd /workspace/src/PX4-Autopilot && make px4_sitl_default"
RUN /bin/bash -lc "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-skip px4"

ENTRYPOINT ["/bin/bash", "/workspace/docker_entrypoint.bash"]
