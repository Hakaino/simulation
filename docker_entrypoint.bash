#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/$ROS_DISTRO/setup.bash
source /workspace/install/setup.bash

if [ $# -ne 0 ]; then
    exec "$@"
fi

WORLD="${SIM_WORLD:-outdoor}"
GUI="${SIM_GUI:-true}"
CONTROLLER="${SIM_CONTROLLER:-true}"
TAKEOFF_ALTITUDE="${SIM_TAKEOFF_ALTITUDE:-1.5}"

echo ""
echo "Starting PX4 quadcopter simulation..."
echo "  world=${WORLD}"
echo "  gui=${GUI}"
echo "  controller=${CONTROLLER}"
echo "  takeoff_altitude=${TAKEOFF_ALTITUDE}"

exec ros2 launch napoleon quad_sim.launch.py \
    "world:=${WORLD}" \
    "gui:=${GUI}" \
    "controller:=${CONTROLLER}" \
    "takeoff_altitude:=${TAKEOFF_ALTITUDE}"
