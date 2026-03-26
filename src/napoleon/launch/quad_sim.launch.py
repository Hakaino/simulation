#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    del args
    del kwargs

    world_choice = LaunchConfiguration("world").perform(context)
    gui_enabled = LaunchConfiguration("gui").perform(context).lower() == "true"
    demo_mode = LaunchConfiguration("demo").perform(context)

    package_share = get_package_share_directory("napoleon")
    model_path = os.path.join(package_share, "models")
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    existing_model_path = os.environ.get("GAZEBO_MODEL_PATH", "")

    world_map = {
        "warehouse": ("warehouse_world", os.path.join(package_share, "worlds", "warehouse.sdf")),
        "empty": ("empty_world", os.path.join(package_share, "worlds", "empty.sdf")),
    }
    if world_choice not in world_map:
        raise RuntimeError(f"Unsupported world '{world_choice}'. Expected one of: {', '.join(sorted(world_map))}.")

    world_name, world_path = world_map[world_choice]
    gz_args = f"-r -v 4 {'-s ' if not gui_enabled else ''}{world_path}"

    bridge_arguments = [
        f"/world/{world_name}/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock",
        "quadcopter/command/motor_speed@actuator_msgs/msg/Actuators@gz.msgs.Actuators",
        f"/world/{world_name}/model/quadcopter/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
        f"/world/{world_name}/dynamic_pose/info@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
    ]

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    parameter_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=bridge_arguments,
        remappings=[
            (f"/world/{world_name}/clock", "/clock"),
            ("quadcopter/command/motor_speed", "/quadcopter/internal/actuators"),
            (
                f"/world/{world_name}/model/quadcopter/link/base_link/sensor/imu_sensor/imu",
                "/quadcopter/state/imu",
            ),
            (f"/world/{world_name}/dynamic_pose/info", "/quadcopter/internal/dynamic_pose"),
        ],
        parameters=[{"use_sim_time": True}],
    )

    motor_command_gate = Node(
        package="napoleon",
        executable="motor_command_gate.py",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"command_topic": "/quadcopter/command/motor_speeds"},
            {"actuator_topic": "/quadcopter/internal/actuators"},
            {"max_motor_speed_rad_s": 900.0},
            {"command_timeout_sec": 0.2},
            {"publish_rate_hz": 50.0},
        ],
    )

    ground_truth_odometry = Node(
        package="napoleon",
        executable="ground_truth_odometry.py",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"pose_topic": "/quadcopter/internal/dynamic_pose"},
            {"odom_topic": "/quadcopter/state/odom"},
            {"world_frame": "world"},
            {"body_frame": "base_link"},
            {"model_name": "quadcopter"},
            {"link_name": "base_link"},
        ],
    )

    launch_actions = [
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            f"{model_path}:{existing_resource_path}" if existing_resource_path else model_path,
        ),
        SetEnvironmentVariable(
            "GAZEBO_MODEL_PATH",
            f"{model_path}:{existing_model_path}" if existing_model_path else model_path,
        ),
        gz_sim,
        parameter_bridge,
        motor_command_gate,
        ground_truth_odometry,
    ]

    if demo_mode == "takeoff":
        launch_actions.append(
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package="napoleon",
                        executable="takeoff_demo.py",
                        output="screen",
                        parameters=[
                            {"use_sim_time": True},
                            {"arm_service": "/quadcopter/arm"},
                            {"motor_command_topic": "/quadcopter/command/motor_speeds"},
                            {"publish_rate_hz": 50.0},
                            {"takeoff_speed_rad_s": 575.0},
                            {"spinup_speed_rad_s": 550.0},
                            {"spinup_duration_sec": 1.0},
                            {"hold_duration_sec": 1.0},
                            {"ramp_down_duration_sec": 1.0},
                        ],
                    )
                ],
            )
        )

    return launch_actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="warehouse",
                description="World to load: warehouse or empty.",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Launch Gazebo with its GUI when true, otherwise run headless.",
            ),
            DeclareLaunchArgument(
                "demo",
                default_value="none",
                description="Optional demo node to run: none or takeoff.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
