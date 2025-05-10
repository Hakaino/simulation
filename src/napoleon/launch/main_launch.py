#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    TimerAction,
    SetEnvironmentVariable,
    IncludeLaunchDescription,
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

    # set environment so Ignition can find model://X4
    set_gz_res = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", model_path)
    set_gz_model = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

    # debug logging
    log_model_path = LogInfo(msg=["[Launch] GAZEBO_MODEL_PATH = ", model_path])

    # launch Gazebo Sim (Harmonic) via ros_gz_sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": LaunchConfiguration("world")}.items(),
    )

    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/X4/command/motor_speed_0"
            "@actuator_msgs/msg/Actuators@gz.msgs.Actuators",
            "/X4/command/motor_speed_1"
            "@actuator_msgs/msg/Actuators@gz.msgs.Actuators",
            "/X4/command/motor_speed_2"
            "@actuator_msgs/msg/Actuators@gz.msgs.Actuators",
            "/X4/command/motor_speed_3"
            "@actuator_msgs/msg/Actuators@gz.msgs.Actuators",
            "/X4/command/motor_speed_4"
            "@actuator_msgs/msg/Actuators@gz.msgs.Actuators",
            "/X4/command/motor_speed_5"
            "@actuator_msgs/msg/Actuators@gz.msgs.Actuators",
            "/world/world_demo/model/X4/link/base_link/sensor/camera_front/image"
            "@sensor_msgs/msg/Image@gz.msgs.Image",
            "/world/world_demo/model/X4/link/base_link/sensor/camera_front/camera_info"
            "@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/world/world_demo/model/X4/link/base_link/sensor/imu_sensor/imu"
            "@sensor_msgs/msg/Imu@gz.msgs.IMU",
        ],
    )

    # delay the bridge so Ignition has time to spawn the model
    delayed_bridge = TimerAction(
        period=3.0,
        actions=[bridge_node],
    )

    return LaunchDescription(
        [
            declared_world,
            set_gz_res,
            set_gz_model,
            log_model_path,
            gz_sim,
            delayed_bridge,
        ]
    )
