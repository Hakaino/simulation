# Quadcopter Simulation

This repository now runs a PX4 SITL quadcopter in Gazebo Harmonic, with a textured outdoor flight range, an onboard down-facing camera, IMU and barometer sensing, a lightweight visual-inertial odometry node, and a ROS 2 bridge that lets a navigation stack drive the vehicle through `/cmd_vel`.

## What It Provides

- PX4 SITL for low-level rate, attitude, position, failsafe, and motor control
- MAVROS for ROS 2 integration
- A textured outdoor world tuned for downward visual odometry
- A downward camera, IMU, and barometer feeding the in-repo VIO node
- External-vision fusion into PX4 EKF2 as the main odometry source
- A PX4 offboard bridge that converts ROS `/cmd_vel` into PX4 local-position setpoints
- Nav2-friendly `/odom` plus `odom -> base_footprint -> base_link` TF

## Prerequisites

- Docker Engine with Compose support
- Linux with X11 if you want the Gazebo GUI

If you use the GUI, allow local container access to your X server before the first run:

```bash
xhost +local:root
```

## Quick Start

Build and run the default outdoor PX4 stack:

```bash
docker compose up --build
```

Run the same stack headless:

```bash
SIM_GUI=false docker compose up --build
```

Start the sim without the `/cmd_vel` offboard bridge so you can drive PX4 directly through MAVROS:

```bash
SIM_CONTROLLER=false docker compose up --build
```

Open a shell inside the image:

```bash
docker compose run --rm simulation bash
```

Inside the container, the main launch entrypoint is:

```bash
ros2 launch napoleon quad_sim.launch.py world:=outdoor gui:=true controller:=true takeoff_altitude:=1.5
```

## Runtime Architecture

The control stack is now split cleanly:

- `visual_inertial_odometry` publishes `nav_msgs/msg/Odometry` to `/mavros/odometry/out`
- PX4 EKF2 fuses that external vision stream and publishes local position through MAVROS
- `px4_odometry_bridge` republishes PX4 local position as `/quadcopter/state/odom` and `/odom`
- `px4_offboard_manager` converts `/cmd_vel` into PX4 local position setpoints on `/mavros/setpoint_position/local`

That means PX4 owns stabilization and motor control, while ROS owns perception, navigation, and high-level motion commands.

## Control Interface

With `controller:=true`, the offboard bridge automatically:

- streams warm-up setpoints
- switches PX4 to `OFFBOARD`
- arms the vehicle
- climbs to `takeoff_altitude`
- keeps altitude while integrating planar `/cmd_vel`

Command planar motion and yaw:

```bash
ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {z: 0.3}}"
```

If you disable the bridge with `controller:=false`, PX4 and MAVROS still start and you can command the vehicle through standard MAVROS topics and services instead.

## Key Topics

- `/cmd_vel`
  `geometry_msgs/msg/Twist`
  high-level planar velocity and yaw-rate command for the PX4 offboard bridge
- `/odom`
  `nav_msgs/msg/Odometry`
  projected PX4 local odometry for navigation, frame `odom`, child `base_footprint`
- `/quadcopter/state/odom`
  `nav_msgs/msg/Odometry`
  full PX4 local odometry, frame `odom`, child `base_link`
- `/mavros/odometry/out`
  `nav_msgs/msg/Odometry`
  raw VIO output sent to PX4 EKF2 as external vision
- `/mavros/local_position/odom`
  `nav_msgs/msg/Odometry`
  PX4 EKF2 local-position estimate from MAVROS
- `/mavros/state`
  `mavros_msgs/msg/State`
- `/imu/data`
  `sensor_msgs/msg/Imu`
- `/baro/data`
  `sensor_msgs/msg/FluidPressure`
- `/odom_camera/image_raw`
  `sensor_msgs/msg/Image`
- `/odom_camera/camera_info`
  `sensor_msgs/msg/CameraInfo`

## Launch Arguments

`quad_sim.launch.py` supports:

- `world:=outdoor`
- `gui:=true|false`
- `controller:=true|false`
- `takeoff_altitude:=1.5`

## Notes

- The default Docker workflow uses `SIM_WORLD=outdoor`, `SIM_GUI=true`, and `SIM_CONTROLLER=true`.
- The PX4-backed launch currently targets the textured outdoor world only.
- PX4 uses a custom SITL airframe in this repo that disables GPS and range fusion, enables EKF2 external-vision fusion, and treats vision as the main height reference.
- The in-repo VIO remains intentionally lightweight. It is now used as an external-vision source for PX4, which is much more robust than using it directly for motor control.
- The outdoor scene still downloads public Fuel grass tiles on first run.
- This setup is Nav2-drivable through `/cmd_vel`, but Nav2 is still a ground-navigation package. For truly aerial planning you may eventually want a planner that reasons in 3D.
