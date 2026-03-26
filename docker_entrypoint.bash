#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/$ROS_DISTRO/setup.bash
source /workspace/install/setup.bash

if [ $# -ne 0 ]; then
    exec "$@"
fi

WORLD="${SIM_WORLD:-warehouse}"
GUI="${SIM_GUI:-true}"
DEMO="${SIM_DEMO:-takeoff}"

echo ""
echo "Starting quadcopter simulation..."
echo "  world=${WORLD}"
echo "  gui=${GUI}"
echo "  demo=${DEMO}"

exec ros2 launch napoleon quad_sim.launch.py "world:=${WORLD}" "gui:=${GUI}" "demo:=${DEMO}"
