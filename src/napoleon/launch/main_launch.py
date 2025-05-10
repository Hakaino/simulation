#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ament_index_python.packages import get_package_share_directory
import os

# Set environment variables for Gazebo resource paths
os.environ["GZ_SIM_RESOURCE_PATH"] = "/workspace/install/napoleon/share/napoleon/models"
os.environ["GAZEBO_MODEL_PATH"] = "/workspace/install/napoleon/share/napoleon/models"



def generate_launch_description():
    # Locate packages
    pkg_napoleon = get_package_share_directory("napoleon")

    # Get the path to the world file and model directory
    world_path = os.path.join(pkg_napoleon, "worlds", "tugbot_warehouse.sdf")
    model_path = os.path.join(pkg_napoleon, "models")

    # Declare the 'world' launch argument
    declared_world = DeclareLaunchArgument(
        "world",
        default_value=TextSubstitution(text=world_path),
        description="Path to the SDF world file",
    )

    # Print model path for debug
    log_model_path = LogInfo(msg=["[Launch] GAZEBO_MODEL_PATH = ", model_path])

    # Launch gz sim with the correct environment
    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", LaunchConfiguration("world")],
        output="screen",
        additional_env={
            "GAZEBO_MODEL_PATH": model_path
        }
    )

    # Bridge cmd_vel from ROS2 → Ignition
    ros_ign_bridge = ExecuteProcess(
        cmd=[
            "ros2", "run", "ros_ign_bridge", "parameter_bridge",
            "/model/tugbot/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist"
        ],
        output="screen"
    )

    # Delay bridge startup to avoid race conditions
    delayed_bridge = TimerAction(
        period=3.0,  # seconds
        actions=[ros_ign_bridge]
    )

    return LaunchDescription([
        declared_world,
        log_model_path,
        gz_sim,
        #delayed_bridge
    ])
