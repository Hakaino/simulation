# Quadcopter Simulation

This repository builds a ROS 2 Jazzy + Gazebo Harmonic quadcopter simulation with a local outdoor flight range, forward and downward RGB cameras, IMU and barometer sensing, a lightweight visual-inertial odometry path, a closed-loop flight controller, and raw rotor-speed access for lower-level experimentation.

## What It Provides

- A local 4-rotor X-configuration quadcopter model
- A realistic outdoor flight range, a warehouse world, and a minimal empty world
- A forward RGB camera for perception and a downward RGB camera for odometry
- An onboard IMU and barometer with basic noise models
- A rotor command gate with arming, clamping, and command timeout protection
- A lightweight downward-camera optical-flow odometry fused with IMU and barometer
- A cascaded hover controller that tracks `/cmd_vel` while holding altitude
- A short open-loop takeoff demo for raw motor-speed testing

## Prerequisites

- Docker Engine with Compose support
- Linux with X11 if you want the Gazebo GUI

If you use the GUI, allow local container access to your X server before the first run:

```bash
xhost +local:root
```

## Quick Start

Build and run the default outdoor scene with the closed-loop controller:

```bash
docker compose up --build
```

Run the same stack headless:

```bash
SIM_GUI=false docker compose up --build
```

Start the sim without the controller so you can publish your own motor commands:

```bash
SIM_CONTROLLER=false SIM_DEMO=none docker compose up --build
```

Open an interactive shell inside the image:

```bash
docker compose run --rm simulation bash
```

Inside the container, the main launch entrypoint is:

```bash
ros2 launch napoleon quad_sim.launch.py world:=outdoor gui:=true controller:=true demo:=none
```

## Control Interface

The default launch starts the native C++ `visual_inertial_odometry` and `flight_controller` nodes. The controller auto-arms the quadcopter, climbs to the configured hover altitude, and listens on `/cmd_vel`, while the odometry stack fuses downward-camera motion with the IMU and barometer.

If the visual flow estimate drops out, the odometry node now zeroes its lateral velocity output and raises lateral covariance instead of publishing drift. The controller treats that as an estimator-health fault and holds altitude / yaw while refusing planar motion until lateral odometry becomes trustworthy again.

The controller uses a cascaded control stack:

- body-frame velocity PI
- attitude-to-body-rate conversion
- body-rate PI
- rotor mixer

Command horizontal motion and yaw:

```bash
ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {z: 0.3}}"
```

Key runtime topics:

- `/cmd_vel`
  `geometry_msgs/msg/Twist`
  planar velocity and yaw-rate command for the closed-loop controller
- `/odom`
  `nav_msgs/msg/Odometry`
  planar visual-inertial odometry for navigation in the `odom` frame with child frame `base_footprint`
- `/quadcopter/state/odom`
  `nav_msgs/msg/Odometry`
  full 6DoF visual-inertial flight-state odometry used by the controller with child frame `base_link`
- `/imu/data`
  `sensor_msgs/msg/Imu`
- `/baro/data`
  `sensor_msgs/msg/FluidPressure`
- `/camera/image_raw`
  `sensor_msgs/msg/Image`
- `/camera/camera_info`
  `sensor_msgs/msg/CameraInfo`
- `/odom_camera/image_raw`
  `sensor_msgs/msg/Image`
- `/odom_camera/camera_info`
  `sensor_msgs/msg/CameraInfo`

If you want raw rotor-speed control instead, launch with `controller:=false`, then arm the vehicle:

```bash
ros2 service call /quadcopter/arm std_srvs/srv/SetBool "{data: true}"
```

Publish rotor speeds manually:

```bash
ros2 topic pub --rate 30 /quadcopter/command/motor_speeds std_msgs/msg/Float64MultiArray "{data: [565.0, 565.0, 565.0, 565.0]}"
```

Topic contract:

- `/quadcopter/command/motor_speeds`
  `std_msgs/msg/Float64MultiArray`
  rotor order: `front_left, front_right, rear_right, rear_left`
- `/quadcopter/arm`
  `std_srvs/srv/SetBool`

The motor gate clamps commands above the configured maximum, rejects invalid arrays, and forces all four motors to zero if commands go stale for more than 200 ms or the vehicle is disarmed.

## Launch Arguments

`quad_sim.launch.py` supports:

- `world:=outdoor|warehouse|empty`
- `gui:=true|false`
- `controller:=true|false`
- `takeoff_altitude:=1.5`
- `demo:=none|takeoff`

## Notes

- The default Docker workflow uses `SIM_WORLD=outdoor`, `SIM_GUI=true`, `SIM_CONTROLLER=true`, and `SIM_DEMO=none`.
- When `controller:=true`, the controller keeps the drone airborne at `takeoff_altitude` and a ROS navigation stack can command it through `/cmd_vel`.
- For Nav2-style configs, use `odom` as the global/local odom frame and `base_footprint` as `robot_base_frame`. The controller itself consumes `/quadcopter/state/odom`.
- The `outdoor` world is now the default and is designed as a drone test range with public Fuel `grasspatch` tiles plus local textured asphalt, buildings, trees, and a marked flight pad. The `warehouse` world remains useful for indoor testing, and both textured worlds give the downward camera more usable features than `empty`.
- The in-repo visual-inertial odometry is intentionally lightweight and should be treated as experimental. If you want a more realistic navigation-grade stack, use a mature VIO package such as OpenVINS, VINS-Fusion, or ORB-SLAM3, and add a downward range sensor or stereo pair.
- The built-in `takeoff` demo is intentionally simple and should only be used with `controller:=false`.
- The `outdoor` scene downloads public Fuel grass tiles on first run. The `warehouse` and `empty` scenes are fully local.
- The repo does not include PX4 or MAVROS in the default path.
- Nav2 integration still needs your choice of localization/costmap policy on top of this stack. The drone now exposes a `/cmd_vel` control path and a camera/IMU/baro odometry source that Nav2 can be wired against, but realistic autonomous flight will still benefit from a stronger estimator than the minimal one included here.
