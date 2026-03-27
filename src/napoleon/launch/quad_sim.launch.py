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
    controller_enabled = LaunchConfiguration("controller").perform(context).lower() == "true"
    takeoff_altitude = float(LaunchConfiguration("takeoff_altitude").perform(context))

    package_share = get_package_share_directory("napoleon")
    model_path = os.path.join(package_share, "models")
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    existing_model_path = os.environ.get("GAZEBO_MODEL_PATH", "")

    world_map = {
        "outdoor": ("outdoor_world", os.path.join(package_share, "worlds", "outdoor.sdf")),
        "warehouse": ("warehouse_world", os.path.join(package_share, "worlds", "warehouse.sdf")),
        "empty": ("empty_world", os.path.join(package_share, "worlds", "empty.sdf")),
    }
    if world_choice not in world_map:
        raise RuntimeError(f"Unsupported world '{world_choice}'. Expected one of: {', '.join(sorted(world_map))}.")
    if controller_enabled and demo_mode != "none":
        raise RuntimeError("The closed-loop controller and the takeoff demo both publish rotor commands. Use only one.")

    world_name, world_path = world_map[world_choice]
    gz_args = f"-r -v 4 {'-s ' if not gui_enabled else ''}{world_path}"

    bridge_arguments = [
        f"/world/{world_name}/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock",
        "quadcopter/command/motor_speed@actuator_msgs/msg/Actuators@gz.msgs.Actuators",
        f"/world/{world_name}/model/quadcopter/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
        (
            f"/world/{world_name}/model/quadcopter/link/base_link/sensor/barometer/air_pressure"
            "@sensor_msgs/msg/FluidPressure@gz.msgs.FluidPressure"
        ),
        (
            f"/world/{world_name}/model/quadcopter/link/base_link/sensor/front_camera/image"
            "@sensor_msgs/msg/Image@gz.msgs.Image"
        ),
        (
            f"/world/{world_name}/model/quadcopter/link/base_link/sensor/front_camera/camera_info"
            "@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo"
        ),
        (
            f"/world/{world_name}/model/quadcopter/link/base_link/sensor/down_camera/image"
            "@sensor_msgs/msg/Image@gz.msgs.Image"
        ),
        (
            f"/world/{world_name}/model/quadcopter/link/base_link/sensor/down_camera/camera_info"
            "@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo"
        ),
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
                "/imu/data",
            ),
            (
                f"/world/{world_name}/model/quadcopter/link/base_link/sensor/barometer/air_pressure",
                "/baro/data",
            ),
            (
                f"/world/{world_name}/model/quadcopter/link/base_link/sensor/front_camera/image",
                "/camera/image_raw",
            ),
            (
                f"/world/{world_name}/model/quadcopter/link/base_link/sensor/front_camera/camera_info",
                "/camera/camera_info",
            ),
            (
                f"/world/{world_name}/model/quadcopter/link/base_link/sensor/down_camera/image",
                "/odom_camera/image_raw",
            ),
            (
                f"/world/{world_name}/model/quadcopter/link/base_link/sensor/down_camera/camera_info",
                "/odom_camera/camera_info",
            ),
            (f"/world/{world_name}/dynamic_pose/info", "/quadcopter/internal/dynamic_pose"),
        ],
        parameters=[{"use_sim_time": True}],
    )

    motor_command_gate = Node(
        package="napoleon",
        executable="motor_command_gate",
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

    visual_inertial_odometry = Node(
        package="napoleon",
        executable="visual_inertial_odometry",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"image_topic": "/odom_camera/image_raw"},
            {"camera_info_topic": "/odom_camera/camera_info"},
            {"imu_topic": "/imu/data"},
            {"pressure_topic": "/baro/data"},
            {"full_odom_topic": "/quadcopter/state/odom"},
            {"projected_odom_topic": "/odom"},
            {"world_frame": "odom"},
            {"body_frame": "base_link"},
            {"projected_body_frame": "base_footprint"},
            {"publish_tf": True},
            {"camera_mount_roll_rad": 0.0},
            {"camera_mount_pitch_rad": 1.57079632679},
            {"camera_mount_yaw_rad": 0.0},
        ],
    )

    ground_truth_odometry = Node(
        package="napoleon",
        executable="ground_truth_odometry",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"pose_topic": "/quadcopter/internal/dynamic_pose"},
            {"full_odom_topic": "/ground_truth/quadcopter/state/odom"},
            {"projected_odom_topic": "/ground_truth/odom"},
            {"world_frame": "odom"},
            {"body_frame": "base_link"},
            {"projected_body_frame": "base_footprint"},
            {"model_name": "quadcopter"},
            {"link_name": "base_link"},
            {"publish_tf": False},
        ],
    )

    imu_sensor_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "quadcopter/base_link/imu_sensor",
        ],
    )

    front_camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0.12",
            "--y",
            "0.0",
            "--z",
            "-0.015",
            "--roll",
            "0.0",
            "--pitch",
            "0.15",
            "--yaw",
            "0.0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "quadcopter/base_link/front_camera",
        ],
    )

    front_camera_optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--roll",
            "-1.57079632679",
            "--pitch",
            "0.0",
            "--yaw",
            "-1.57079632679",
            "--frame-id",
            "quadcopter/base_link/front_camera",
            "--child-frame-id",
            "quadcopter/base_link/front_camera_optical",
        ],
    )

    down_camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "-0.09",
            "--roll",
            "0.0",
            "--pitch",
            "1.57079632679",
            "--yaw",
            "0.0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "quadcopter/base_link/down_camera",
        ],
    )

    down_camera_optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--roll",
            "-1.57079632679",
            "--pitch",
            "0.0",
            "--yaw",
            "-1.57079632679",
            "--frame-id",
            "quadcopter/base_link/down_camera",
            "--child-frame-id",
            "quadcopter/base_link/down_camera_optical",
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
        visual_inertial_odometry,
        ground_truth_odometry,
        imu_sensor_tf,
        front_camera_tf,
        front_camera_optical_tf,
        down_camera_tf,
        down_camera_optical_tf,
    ]

    if controller_enabled:
        launch_actions.append(
            Node(
                package="napoleon",
                executable="flight_controller",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                    {"cmd_vel_topic": "/cmd_vel"},
                    {"odom_topic": "/quadcopter/state/odom"},
                    {"imu_topic": "/imu/data"},
                    {"motor_command_topic": "/quadcopter/command/motor_speeds"},
                    {"arm_service": "/quadcopter/arm"},
                    {"auto_arm": True},
                    {"takeoff_altitude_m": takeoff_altitude},
                ],
            )
        )

    if demo_mode == "takeoff":
        launch_actions.append(
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package="napoleon",
                        executable="takeoff_demo",
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
                default_value="outdoor",
                description="World to load: outdoor, warehouse, or empty.",
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
            DeclareLaunchArgument(
                "controller",
                default_value="true",
                description="Launch the closed-loop flight controller for hover and /cmd_vel tracking.",
            ),
            DeclareLaunchArgument(
                "takeoff_altitude",
                default_value="1.5",
                description="Target altitude in meters for the flight controller hover setpoint.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
