#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/$ROS_DISTRO/setup.bash
source /workspace/install/setup.bash

if [ $# -ne 0 ]; then
    exec "$@"
fi

WORLD="${SIM_WORLD:-warehouse}"
GUI="${SIM_GUI:-true}"
CONTROLLER="${SIM_CONTROLLER:-true}"
TAKEOFF_ALTITUDE="${SIM_TAKEOFF_ALTITUDE:-1.5}"
DEMO="${SIM_DEMO:-none}"

echo ""
echo "Starting quadcopter simulation..."
echo "  world=${WORLD}"
echo "  gui=${GUI}"
echo "  controller=${CONTROLLER}"
echo "  takeoff_altitude=${TAKEOFF_ALTITUDE}"
echo "  demo=${DEMO}"

exec ros2 launch napoleon quad_sim.launch.py \
    "world:=${WORLD}" \
    "gui:=${GUI}" \
    "controller:=${CONTROLLER}" \
    "takeoff_altitude:=${TAKEOFF_ALTITUDE}" \
    "demo:=${DEMO}"
