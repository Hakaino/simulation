#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    TimerAction,
    SetEnvironmentVariable,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # locate your package & paths
    pkg_napoleon = get_package_share_directory("napoleon")
    world_path = os.path.join(pkg_napoleon, "worlds", "tugbot_warehouse.sdf")
    model_path = os.path.join(pkg_napoleon, "models")

    # allow overriding the world on the command line
    declared_world = DeclareLaunchArgument(
        "world",
        default_value=TextSubstitution(text=world_path),
        description="Path to the SDF world file",
    )

    # make your models visible to ignition
    set_gz_res = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", model_path)
    set_gz_model = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)
    log_model = LogInfo(msg=["[Launch] GAZEBO_MODEL_PATH = ", model_path])

    # --- 1) start PX4 SITL in OFFBOARD mode ---
    #    Assumes you've already built PX4-Autopilot with:
    #       make px4_sitl_default gazebo-classic
    #    and that 'px4' is in your PATH (or edit the full path below).
    px4_sitl = ExecuteProcess(
        cmd=[
            "px4",  # or "/full/path/to/PX4-Autopilot/build/px4_sitl_default/bin/px4"
            "-s",
            os.path.expanduser(
                "~/PX4-Autopilot/ROMFS/px4_fmu_common"
            ),  # point to ROMFS
            "etc/init.d-posix/rcS",  # startup script for SITL
        ],
        output="screen",
    )

    # --- 2) launch Ignition Gazebo (via ros_gz_sim) ---
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": LaunchConfiguration("world")}.items(),
    )

    # --- 3) bridge the PX4 topics into ROS 2 ---
    #      We delay it a few seconds so the model is spawned first.
    bridge_args = [
        # IMU, odometry, etc coming out of the Gazebo PX4 plugin:
        "/X4/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
        "/X4/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry",
        # And your motor_speed topics (if you still need them):
        "/X4/command/motor_speed@actuator_msgs/msg/Actuators@gz.msgs.Actuators",
    ]
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=bridge_args,
    )
    delayed_bridge = TimerAction(period=3.0, actions=[bridge_node])

    # --- 4) launch MAVROS so you can send OFFBOARD setpoints ---
    #    This example assumes you have mavros (ROS 2 port) installed.
    #    fcu_url must match PX4's default SITL port (14540/14557).
    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        output="screen",
        parameters=[
            {
                "fcu_url": "udp://:14540@127.0.0.1:14557",
                "gcs_url": "",
                "system_id": 1,
                "component_id": 1,
            }
        ],
    )
    delayed_mavros = TimerAction(period=5.0, actions=[mavros_node])

    return LaunchDescription(
        [
            declared_world,
            set_gz_res,
            set_gz_model,
            log_model,
            # start PX4 SITL, then gazebo, then bridge, then MAVROS
            px4_sitl,
            gz_sim,
            delayed_bridge,
            delayed_mavros,
        ]
    )
