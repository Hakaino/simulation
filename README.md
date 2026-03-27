# Quadcopter Simulation

This repository builds a ROS 2 Jazzy + Gazebo Harmonic quadcopter simulation with a forward camera, IMU, closed-loop flight controller, and raw rotor-speed access for lower-level experimentation.

## What It Provides

- A local 4-rotor X-configuration quadcopter model
- A warehouse world and a minimal empty world
- A forward-facing RGB camera and onboard IMU
- A rotor command gate with arming, clamping, and command timeout protection
- Ground-truth odometry with `odom -> base_link` TF for navigation stacks
- A closed-loop hover controller that tracks `/cmd_vel` while holding altitude
- A short open-loop takeoff demo for raw motor-speed testing

## Prerequisites

- Docker Engine with Compose support
- Linux with X11 if you want the Gazebo GUI

If you use the GUI, allow local container access to your X server before the first run:

```bash
xhost +local:root
```

## Quick Start

Build and run the default warehouse scene with the closed-loop controller:

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
ros2 launch napoleon quad_sim.launch.py world:=warehouse gui:=true controller:=true demo:=none
```

## Control Interface

The default launch starts `flight_controller.py`, which auto-arms the quadcopter, climbs to the configured hover altitude, and listens on `/cmd_vel`.

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
  ground-truth odometry in the `odom` frame
- `/imu/data`
  `sensor_msgs/msg/Imu`
- `/camera/image_raw`
  `sensor_msgs/msg/Image`
- `/camera/camera_info`
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

- `world:=warehouse|empty`
- `gui:=true|false`
- `controller:=true|false`
- `takeoff_altitude:=1.5`
- `demo:=none|takeoff`

## Notes

- The default Docker workflow uses `SIM_WORLD=warehouse`, `SIM_GUI=true`, `SIM_CONTROLLER=true`, and `SIM_DEMO=none`.
- When `controller:=true`, the controller keeps the drone airborne at `takeoff_altitude` and Nav2 can command it through `/cmd_vel`.
- The built-in `takeoff` demo is intentionally simple and should only be used with `controller:=false`.
- The warehouse scene is fully local; it does not download Fuel assets at runtime.
- The repo does not include PX4 or MAVROS in the default path.
- Nav2 integration still needs a localization and obstacle-source choice on top of this stack. The drone now exposes the standard flight-control interfaces that Nav2 expects to drive.
