# Quadcopter Simulation

This repository builds a ROS 2 Jazzy + Gazebo Harmonic quadcopter simulation that you control by publishing rotor angular velocities in `rad/s`.

## What It Provides

- A local 4-rotor X-configuration quadcopter model
- A warehouse world and a minimal empty world
- A rotor command gate with arming, clamping, and command timeout protection
- Ground-truth odometry and IMU topics for controller development
- A short open-loop takeoff demo that lifts off and settles back down

## Prerequisites

- Docker Engine with Compose support
- Linux with X11 if you want the Gazebo GUI

If you use the GUI, allow local container access to your X server before the first run:

```bash
xhost +local:root
```

## Quick Start

Build and run the default warehouse scene with the takeoff demo:

```bash
docker compose up --build
```

Run the same stack headless:

```bash
SIM_GUI=false docker compose up --build
```

Start the sim without the demo so you can publish your own motor commands:

```bash
SIM_DEMO=none docker compose up --build
```

Open an interactive shell inside the image:

```bash
docker compose run --rm simulation bash
```

Inside the container, the main launch entrypoint is:

```bash
ros2 launch napoleon quad_sim.launch.py world:=warehouse gui:=true demo:=none
```

## Control Interface

Arm the vehicle:

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
- `/quadcopter/state/odom`
  `nav_msgs/msg/Odometry`
- `/quadcopter/state/imu`
  `sensor_msgs/msg/Imu`

The motor gate clamps commands above the configured maximum, rejects invalid arrays, and forces all four motors to zero if commands go stale for more than 200 ms or the vehicle is disarmed.

## Launch Arguments

`quad_sim.launch.py` supports:

- `world:=warehouse|empty`
- `gui:=true|false`
- `demo:=none|takeoff`

## Notes

- The default Docker workflow uses `SIM_WORLD=warehouse`, `SIM_GUI=true`, and `SIM_DEMO=takeoff`.
- The built-in `takeoff` demo is intentionally simple: it sends the same rotor speed to all four motors long enough to show lift-off, then ramps back down.
- The warehouse scene is fully local; it does not download Fuel assets at runtime.
- The repo does not include PX4 or MAVROS in the default path. The interface is intentionally raw rotor-speed control so you can build your own flight logic on top.
