#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _prepend_path(path, existing):
    return f"{path}:{existing}" if existing else path


def _launch_setup(context, *args, **kwargs):
    del args
    del kwargs

    world_choice = LaunchConfiguration("world").perform(context)
    gui_enabled = LaunchConfiguration("gui").perform(context).lower() == "true"
    controller_enabled = LaunchConfiguration("controller").perform(context).lower() == "true"
    takeoff_altitude = float(LaunchConfiguration("takeoff_altitude").perform(context))

    if world_choice != "outdoor":
        raise RuntimeError(
            "The PX4-backed launch currently supports the textured outdoor world only. "
            "Use world:=outdoor."
        )

    package_share = get_package_share_directory("napoleon")
    mavros_share = get_package_share_directory("mavros")
    px4_root = os.environ.get("PX4_AUTOPILOT_PATH", "/workspace/src/PX4-Autopilot")
    if not os.path.isdir(px4_root):
        raise RuntimeError(
            "PX4_AUTOPILOT_PATH does not point to a PX4-Autopilot checkout. "
            f"Looked for: {px4_root}"
        )
    px4_models = os.path.join(px4_root, "Tools", "simulation", "gz", "models")
    px4_worlds = os.path.join(px4_root, "Tools", "simulation", "gz", "worlds")
    px4_plugins = os.path.join(px4_root, "build", "px4_sitl_default", "src", "modules", "simulation", "gz_plugins")
    px4_server_config = os.path.join(
        px4_root,
        "src",
        "modules",
        "simulation",
        "gz_bridge",
        "server.config",
    )
    napoleon_models = os.path.join(package_share, "models")
    world_path = os.path.join(package_share, "worlds", "outdoor_px4.sdf")
    world_name = "outdoor_world"
    px4_sim_model = "x500_mono_cam_down"
    vehicle_name = "x500_mono_cam_down_0"
    custom_airframe = os.path.join(
        px4_root,
        "ROMFS",
        "px4fmu_common",
        "init.d-posix",
        "airframes",
        "22000_gz_x500_mono_cam_down_vio",
    )
    px4_autostart = "22000" if os.path.isfile(custom_airframe) else "4014"

    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    existing_model_path = os.environ.get("GAZEBO_MODEL_PATH", "")
    existing_plugin_path = os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
    existing_server_config_path = os.environ.get("GZ_SIM_SERVER_CONFIG_PATH", "")
    resource_path = napoleon_models
    resource_path = _prepend_path(px4_models, resource_path)
    resource_path = _prepend_path(px4_worlds, resource_path)
    if existing_resource_path:
        resource_path = _prepend_path(resource_path, existing_resource_path)

    model_path = napoleon_models
    model_path = _prepend_path(px4_models, model_path)
    if existing_model_path:
        model_path = _prepend_path(model_path, existing_model_path)

    plugin_path = px4_plugins
    if existing_plugin_path:
        plugin_path = _prepend_path(plugin_path, existing_plugin_path)

    server_config_path = px4_server_config
    if existing_server_config_path:
        server_config_path = _prepend_path(server_config_path, existing_server_config_path)

    gz_args = f"-r -v 4 {'-s ' if not gui_enabled else ''}{world_path}"

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
    )

    bridge_arguments = [
        (
            f"/world/{world_name}/model/{vehicle_name}/link/base_link/sensor/imu_sensor/imu"
            "@sensor_msgs/msg/Imu@gz.msgs.IMU"
        ),
        (
            f"/world/{world_name}/model/{vehicle_name}/link/base_link/sensor/air_pressure_sensor/air_pressure"
            "@sensor_msgs/msg/FluidPressure@gz.msgs.FluidPressure"
        ),
        (
            f"/world/{world_name}/model/{vehicle_name}/link/camera_link/sensor/camera/image"
            "@sensor_msgs/msg/Image@gz.msgs.Image"
        ),
        (
            f"/world/{world_name}/model/{vehicle_name}/link/camera_link/sensor/camera/camera_info"
            "@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo"
        ),
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
            (
                f"/world/{world_name}/model/{vehicle_name}/link/base_link/sensor/imu_sensor/imu",
                "/imu/data",
            ),
            (
                f"/world/{world_name}/model/{vehicle_name}/link/base_link/sensor/air_pressure_sensor/air_pressure",
                "/baro/data",
            ),
            (
                f"/world/{world_name}/model/{vehicle_name}/link/camera_link/sensor/camera/image",
                "/odom_camera/image_raw",
            ),
            (
                f"/world/{world_name}/model/{vehicle_name}/link/camera_link/sensor/camera/camera_info",
                "/odom_camera/camera_info",
            ),
        ],
        parameters=[{"use_sim_time": True}],
    )

    px4_process = ExecuteProcess(
        cmd=[
            "/bin/bash",
            "-lc",
            (
                f"cd {px4_root} && "
                ". build/px4_sitl_default/rootfs/gz_env.sh && "
                f"PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART={px4_autostart} "
                f"PX4_GZ_MODEL_NAME={vehicle_name} PX4_GZ_WORLD={world_name} "
                "./build/px4_sitl_default/bin/px4"
            ),
        ],
        output="screen",
    )

    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        output="screen",
        parameters=[
            os.path.join(mavros_share, "launch", "px4_pluginlists.yaml"),
            os.path.join(mavros_share, "launch", "px4_config.yaml"),
            {
                "fcu_url": "udp://:14540@127.0.0.1:14557",
                "gcs_url": "",
                "tgt_system": 1,
                "tgt_component": 1,
                "use_sim_time": True,
            },
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
            {"full_odom_topic": "/mavros/odometry/out"},
            {"projected_odom_topic": ""},
            {"status_topic": "/mavros/companion_process/status"},
            {"world_frame": "odom"},
            {"body_frame": "base_link"},
            {"projected_body_frame": "base_footprint"},
            {"publish_tf": False},
            {"publish_rate_hz": 30.0},
            {"max_features": 160},
            {"min_features": 70},
            {"min_tracked_features": 35},
            {"lk_max_level": 2},
            {"camera_mount_roll_rad": 0.0},
            {"camera_mount_pitch_rad": 1.57079632679},
            {"camera_mount_yaw_rad": 0.0},
        ],
    )

    px4_odometry_bridge = Node(
        package="napoleon",
        executable="px4_odometry_bridge",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"source_topic": "/mavros/local_position/odom"},
            {"full_odom_topic": "/quadcopter/state/odom"},
            {"projected_odom_topic": "/odom"},
            {"world_frame": "odom"},
            {"body_frame": "base_link"},
            {"projected_body_frame": "base_footprint"},
            {"publish_tf": True},
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
            "0.10",
            "--roll",
            "0.0",
            "--pitch",
            "1.57079632679",
            "--yaw",
            "0.0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "odom_camera",
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
            "odom_camera",
            "--child-frame-id",
            "odom_camera_optical",
        ],
    )

    launch_actions = [
        SetEnvironmentVariable("PX4_GZ_MODELS", px4_models),
        SetEnvironmentVariable("PX4_GZ_WORLDS", px4_worlds),
        SetEnvironmentVariable("PX4_GZ_PLUGINS", px4_plugins),
        SetEnvironmentVariable("PX4_GZ_SERVER_CONFIG", px4_server_config),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path),
        SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", plugin_path),
        SetEnvironmentVariable("GZ_SIM_SERVER_CONFIG_PATH", server_config_path),
        gz_sim,
        clock_bridge,
        parameter_bridge,
        px4_process,
        mavros_node,
        visual_inertial_odometry,
        px4_odometry_bridge,
        down_camera_tf,
        down_camera_optical_tf,
    ]

    if controller_enabled:
        launch_actions.append(
            Node(
                package="napoleon",
                executable="px4_offboard_manager",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                    {"cmd_vel_topic": "/cmd_vel"},
                    {"state_topic": "/mavros/state"},
                    {"odom_topic": "/quadcopter/state/odom"},
                    {"setpoint_topic": "/mavros/setpoint_position/local"},
                    {"set_mode_service": "/mavros/set_mode"},
                    {"arm_service": "/mavros/cmd/arming"},
                    {"takeoff_altitude_m": takeoff_altitude},
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
                description="World to load. PX4 launch currently supports the outdoor world.",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Launch Gazebo with its GUI when true, otherwise run headless.",
            ),
            DeclareLaunchArgument(
                "controller",
                default_value="true",
                description="Launch the PX4 offboard bridge that converts /cmd_vel into PX4 setpoints.",
            ),
            DeclareLaunchArgument(
                "takeoff_altitude",
                default_value="1.5",
                description="Target local altitude in meters for the PX4 offboard takeoff setpoint.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
