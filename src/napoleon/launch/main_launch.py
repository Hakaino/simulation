#!/usr/bin/env python3
"""
ROS 2 Jazzy Gazebo Launcher

This launch file starts Ignition Gazebo (ros_gz_sim) server and GUI,
loading a specified world SDF file.

Usage:
  ros2 launch napoleon main_launch.py world:=/path/to/your.world.sdf
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Locate packages
    pkg_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_napoleon = get_package_share_directory('napoleon')

    # Declare the 'world' launch argument
    declared_world = DeclareLaunchArgument(
        'world',
        default_value=TextSubstitution(text=os.path.join(pkg_napoleon, 'worlds', 'ground.world')),
        description='Path to the SDF world file'
    )

    # Launch Gazebo server
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gz_sim, 'launch', 'gz_server.launch.py')  # correct filename in ros_gz_sim ([github.com](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_sim/launch/gz_server.launch.py?utm_source=chatgpt.com))
        ),
        launch_arguments={
            'world_sdf_file': LaunchConfiguration('world')
        }.items()
    )

    # Launch Gazebo (server + GUI)
    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gz_sim, 'launch', 'gz_sim.launch.py')     # starts server and GUI ([gazebosim.org](https://gazebosim.org/docs/latest/ros2_launch_gazebo/?utm_source=chatgpt.com))
        ),
        launch_arguments={
            'gz_args': LaunchConfiguration('world')
        }.items()
    )

    return LaunchDescription([
        declared_world,
        gz_server,
        gz_gui,
    ])
