#!/usr/bin/env bash
set -e

. /opt/ros/$ROS_DISTRO/setup.bash
. /workspace/install/setup.bash
#Start Server
MicroXRCEAgent udp4 -p 8888 &

if [ $# -ne 0 ]; then
    exec "$@"
    exit 0
fi
echo "\n\n Start simulation........................................."
    cd /workspace/src/Micro-XRCE-DDS-Agent/
    make px4_sitl gz_x500 &
    sleep 5
    ros2 launch px4_ros_com sensor_combined_listener.launch.py