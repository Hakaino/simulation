#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Locate packages
    pkg_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_napoleon = get_package_share_directory("napoleon")

    # Declare the 'world' launch argument
    declared_world = DeclareLaunchArgument(
        "world",
        default_value=TextSubstitution(
            text=os.path.join(pkg_napoleon, "worlds", "ground.world")
        ),
        description="Path to the SDF world file",
    )

    # Launch Gazebo server
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_gz_sim, "launch", "gz_server.launch.py"
            )
        ),
        launch_arguments={"world_sdf_file": LaunchConfiguration("world")}.items(),
    )

    # Launch Gazebo (server + GUI)
    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_gz_sim, "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": LaunchConfiguration("world")}.items(),
    )

    return LaunchDescription(
        [
            declared_world,
            gz_server,
            gz_gui,
        ]
    )
