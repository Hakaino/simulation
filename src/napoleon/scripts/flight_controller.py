#!/usr/bin/env python3
import math
from typing import Optional, Tuple

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(value, upper))


def _rotate_vector_by_quat(
    vector: Tuple[float, float, float],
    quat: Tuple[float, float, float, float],
) -> Tuple[float, float, float]:
    vx, vy, vz = vector
    qx, qy, qz, qw = quat
    ix = qw * vx + qy * vz - qz * vy
    iy = qw * vy + qz * vx - qx * vz
    iz = qw * vz + qx * vy - qy * vx
    iw = -qx * vx - qy * vy - qz * vz
    return (
        ix * qw + iw * -qx + iy * -qz - iz * -qy,
        iy * qw + iw * -qy + iz * -qx - ix * -qz,
        iz * qw + iw * -qz + ix * -qy - iy * -qx,
    )


def _quat_to_euler(quat: Tuple[float, float, float, float]) -> Tuple[float, float, float]:
    x, y, z, w = quat

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class FlightController(Node):
    def __init__(self) -> None:
        super().__init__("flight_controller")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("motor_command_topic", "/quadcopter/command/motor_speeds")
        self.declare_parameter("arm_service", "/quadcopter/arm")
        self.declare_parameter("control_rate_hz", 100.0)
        self.declare_parameter("command_timeout_sec", 0.5)
        self.declare_parameter("auto_arm", True)
        self.declare_parameter("takeoff_altitude_m", 1.5)
        self.declare_parameter("max_altitude_m", 3.0)
        self.declare_parameter("max_horizontal_speed_mps", 1.5)
        self.declare_parameter("max_vertical_speed_mps", 0.5)
        self.declare_parameter("max_yaw_rate_rad_s", 1.0)
        self.declare_parameter("max_tilt_rad", 0.35)
        self.declare_parameter("mass_kg", 1.08)
        self.declare_parameter("gravity_mps2", 9.81)
        self.declare_parameter("arm_length_m", 0.18)
        self.declare_parameter("motor_constant", 8.54858e-06)
        self.declare_parameter("moment_constant", 0.016)
        self.declare_parameter("max_motor_speed_rad_s", 900.0)
        self.declare_parameter("altitude_kp", 3.8)
        self.declare_parameter("altitude_ki", 0.8)
        self.declare_parameter("altitude_kd", 2.4)
        self.declare_parameter("velocity_to_pitch_gain", 0.22)
        self.declare_parameter("velocity_to_roll_gain", 0.22)
        self.declare_parameter("attitude_kp", 7.5)
        self.declare_parameter("attitude_kd", 2.8)
        self.declare_parameter("yaw_rate_kp", 0.25)

        cmd_vel_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        imu_topic = self.get_parameter("imu_topic").get_parameter_value().string_value
        motor_command_topic = self.get_parameter("motor_command_topic").get_parameter_value().string_value
        arm_service = self.get_parameter("arm_service").get_parameter_value().string_value
        control_rate_hz = self.get_parameter("control_rate_hz").get_parameter_value().double_value
        timeout_sec = self.get_parameter("command_timeout_sec").get_parameter_value().double_value

        self.auto_arm = self.get_parameter("auto_arm").get_parameter_value().bool_value
        self.takeoff_altitude = self.get_parameter("takeoff_altitude_m").get_parameter_value().double_value
        self.max_altitude = self.get_parameter("max_altitude_m").get_parameter_value().double_value
        self.max_horizontal_speed = (
            self.get_parameter("max_horizontal_speed_mps").get_parameter_value().double_value
        )
        self.max_vertical_speed = self.get_parameter("max_vertical_speed_mps").get_parameter_value().double_value
        self.max_yaw_rate = self.get_parameter("max_yaw_rate_rad_s").get_parameter_value().double_value
        self.max_tilt = self.get_parameter("max_tilt_rad").get_parameter_value().double_value
        self.mass = self.get_parameter("mass_kg").get_parameter_value().double_value
        self.gravity = self.get_parameter("gravity_mps2").get_parameter_value().double_value
        self.arm_length = self.get_parameter("arm_length_m").get_parameter_value().double_value
        self.motor_constant = self.get_parameter("motor_constant").get_parameter_value().double_value
        self.moment_constant = self.get_parameter("moment_constant").get_parameter_value().double_value
        self.max_motor_speed = self.get_parameter("max_motor_speed_rad_s").get_parameter_value().double_value
        self.altitude_kp = self.get_parameter("altitude_kp").get_parameter_value().double_value
        self.altitude_ki = self.get_parameter("altitude_ki").get_parameter_value().double_value
        self.altitude_kd = self.get_parameter("altitude_kd").get_parameter_value().double_value
        self.velocity_to_pitch_gain = (
            self.get_parameter("velocity_to_pitch_gain").get_parameter_value().double_value
        )
        self.velocity_to_roll_gain = self.get_parameter("velocity_to_roll_gain").get_parameter_value().double_value
        self.attitude_kp = self.get_parameter("attitude_kp").get_parameter_value().double_value
        self.attitude_kd = self.get_parameter("attitude_kd").get_parameter_value().double_value
        self.yaw_rate_kp = self.get_parameter("yaw_rate_kp").get_parameter_value().double_value

        self.command_timeout = Duration(seconds=timeout_sec)
        self.command_publisher = self.create_publisher(Float64MultiArray, motor_command_topic, 10)
        self.cmd_vel_subscriber = self.create_subscription(Twist, cmd_vel_topic, self.cmd_vel_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, odom_topic, self.odom_callback, 20)
        self.imu_subscriber = self.create_subscription(Imu, imu_topic, self.imu_callback, 50)
        self.arm_client = self.create_client(SetBool, arm_service)
        self.arm_retry_timer = self.create_timer(1.0, self.ensure_armed)
        self.control_timer = self.create_timer(1.0 / max(control_rate_hz, 1.0), self.control_loop)

        self.last_command_time = None
        self.last_control_time = None
        self.last_arm_request_time = None
        self.arm_request_future = None
        self.arm_acknowledged = False

        self.commanded_velocity_x = 0.0
        self.commanded_velocity_y = 0.0
        self.commanded_velocity_z = 0.0
        self.commanded_yaw_rate = 0.0

        self.altitude_integral = 0.0
        self.target_altitude: Optional[float] = None
        self.current_altitude: Optional[float] = None
        self.orientation: Optional[Tuple[float, float, float, float]] = None
        self.body_velocity = (0.0, 0.0, 0.0)
        self.world_vertical_velocity = 0.0
        self.angular_velocity = (0.0, 0.0, 0.0)

        self.get_logger().info(
            "Flight controller ready. It will hold altitude and accept planar /cmd_vel commands for navigation."
        )

    def cmd_vel_callback(self, message: Twist) -> None:
        self.commanded_velocity_x = _clamp(
            message.linear.x,
            -self.max_horizontal_speed,
            self.max_horizontal_speed,
        )
        self.commanded_velocity_y = _clamp(
            message.linear.y,
            -self.max_horizontal_speed,
            self.max_horizontal_speed,
        )
        self.commanded_velocity_z = _clamp(
            message.linear.z,
            -self.max_vertical_speed,
            self.max_vertical_speed,
        )
        self.commanded_yaw_rate = _clamp(
            message.angular.z,
            -self.max_yaw_rate,
            self.max_yaw_rate,
        )
        self.last_command_time = self.get_clock().now()

    def odom_callback(self, message: Odometry) -> None:
        self.current_altitude = message.pose.pose.position.z
        self.orientation = (
            message.pose.pose.orientation.x,
            message.pose.pose.orientation.y,
            message.pose.pose.orientation.z,
            message.pose.pose.orientation.w,
        )
        self.body_velocity = (
            message.twist.twist.linear.x,
            message.twist.twist.linear.y,
            message.twist.twist.linear.z,
        )

        world_velocity = _rotate_vector_by_quat(self.body_velocity, self.orientation)
        self.world_vertical_velocity = world_velocity[2]

        if self.target_altitude is None:
            self.target_altitude = max(self.current_altitude, self.takeoff_altitude)

    def imu_callback(self, message: Imu) -> None:
        self.angular_velocity = (
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
        )

    def ensure_armed(self) -> None:
        if not self.auto_arm:
            return
        if not self.arm_client.service_is_ready():
            return

        now = self.get_clock().now()
        if self.arm_request_future is not None and not self.arm_request_future.done():
            return

        if self.arm_request_future is not None and self.arm_request_future.done():
            exception = self.arm_request_future.exception()
            if exception is not None:
                self.get_logger().warning(f"Arm request failed: {exception}")
                self.arm_request_future = None
                return

            response = self.arm_request_future.result()
            if response is not None and response.success:
                if not self.arm_acknowledged:
                    self.get_logger().info("Flight controller armed the quadcopter.")
                self.arm_acknowledged = True
                return
            if response is not None and response.message:
                self.get_logger().warning(f"Arm request rejected: {response.message}")

        if self.last_arm_request_time is not None:
            if (now - self.last_arm_request_time) < Duration(seconds=1.0):
                return

        request = SetBool.Request()
        request.data = True
        self.arm_request_future = self.arm_client.call_async(request)
        self.last_arm_request_time = now

    def command_is_stale(self) -> bool:
        if self.last_command_time is None:
            return True
        return (self.get_clock().now() - self.last_command_time) > self.command_timeout

    def control_loop(self) -> None:
        now = self.get_clock().now()
        if self.orientation is None or self.current_altitude is None or self.target_altitude is None:
            return

        dt = 0.01
        if self.last_control_time is not None:
            dt = max((now - self.last_control_time).nanoseconds * 1e-9, 1e-3)
        self.last_control_time = now

        if self.command_is_stale():
            commanded_velocity_x = 0.0
            commanded_velocity_y = 0.0
            commanded_velocity_z = 0.0
            commanded_yaw_rate = 0.0
        else:
            commanded_velocity_x = self.commanded_velocity_x
            commanded_velocity_y = self.commanded_velocity_y
            commanded_velocity_z = self.commanded_velocity_z
            commanded_yaw_rate = self.commanded_yaw_rate

        self.target_altitude += commanded_velocity_z * dt
        self.target_altitude = _clamp(self.target_altitude, 0.3, self.max_altitude)

        roll, pitch, _ = _quat_to_euler(self.orientation)
        body_z_axis_in_world = _rotate_vector_by_quat((0.0, 0.0, 1.0), self.orientation)

        altitude_error = self.target_altitude - self.current_altitude
        self.altitude_integral = _clamp(self.altitude_integral + altitude_error * dt, -1.5, 1.5)
        desired_vertical_acceleration = (
            self.altitude_kp * altitude_error
            + self.altitude_ki * self.altitude_integral
            - self.altitude_kd * self.world_vertical_velocity
        )
        desired_vertical_acceleration = _clamp(desired_vertical_acceleration, -4.0, 4.0)

        thrust = self.mass * (self.gravity + desired_vertical_acceleration)
        thrust /= max(body_z_axis_in_world[2], 0.35)
        thrust = max(0.0, thrust)

        desired_pitch = _clamp(
            self.velocity_to_pitch_gain * (commanded_velocity_x - self.body_velocity[0]),
            -self.max_tilt,
            self.max_tilt,
        )
        desired_roll = _clamp(
            -self.velocity_to_roll_gain * (commanded_velocity_y - self.body_velocity[1]),
            -self.max_tilt,
            self.max_tilt,
        )

        roll_torque = self.attitude_kp * (desired_roll - roll) - self.attitude_kd * self.angular_velocity[0]
        pitch_torque = self.attitude_kp * (desired_pitch - pitch) - self.attitude_kd * self.angular_velocity[1]
        yaw_torque = self.yaw_rate_kp * (commanded_yaw_rate - self.angular_velocity[2])

        collective = thrust / (4.0 * self.motor_constant)
        roll_term = roll_torque / (4.0 * self.motor_constant * self.arm_length)
        pitch_term = pitch_torque / (4.0 * self.motor_constant * self.arm_length)
        yaw_term = yaw_torque / (4.0 * self.motor_constant * self.moment_constant)

        squared_speeds = [
            collective + roll_term - pitch_term + yaw_term,
            collective - roll_term - pitch_term - yaw_term,
            collective - roll_term + pitch_term + yaw_term,
            collective + roll_term + pitch_term - yaw_term,
        ]

        max_squared_speed = self.max_motor_speed * self.max_motor_speed
        motor_speeds = [
            math.sqrt(_clamp(squared_speed, 0.0, max_squared_speed))
            for squared_speed in squared_speeds
        ]
        self.command_publisher.publish(Float64MultiArray(data=motor_speeds))


def main() -> None:
    rclpy.init()
    node = FlightController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
