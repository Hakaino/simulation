#!/usr/bin/env bash
set -e

. /opt/ros/$ROS_DISTRO/setup.bash
if [ $# -ne 0 ]; then
    exec "$@"
    exit 0
fi
echo "\n\n Installing apt dependencies........................................."
    cd /workspace/src/Micro-XRCE-DDS-Agent/
    MicroXRCEAgent udp4 -p 8888 &
    make px4_sitl gz_x500