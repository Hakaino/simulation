#!/usr/bin/env python3
import math
from typing import List

from actuator_msgs.msg import Actuators
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool


class MotorCommandGate(Node):
    def __init__(self) -> None:
        super().__init__("motor_command_gate")

        self.declare_parameter("command_topic", "/quadcopter/command/motor_speeds")
        self.declare_parameter("actuator_topic", "/quadcopter/internal/actuators")
        self.declare_parameter("max_motor_speed_rad_s", 900.0)
        self.declare_parameter("command_timeout_sec", 0.2)
        self.declare_parameter("publish_rate_hz", 50.0)

        command_topic = self.get_parameter("command_topic").get_parameter_value().string_value
        actuator_topic = self.get_parameter("actuator_topic").get_parameter_value().string_value
        self.max_motor_speed = (
            self.get_parameter("max_motor_speed_rad_s").get_parameter_value().double_value
        )
        timeout_sec = self.get_parameter("command_timeout_sec").get_parameter_value().double_value
        publish_rate_hz = self.get_parameter("publish_rate_hz").get_parameter_value().double_value

        self.command_timeout = Duration(seconds=timeout_sec)
        self.armed = False
        self.last_valid_command_time = None
        self.desired_speeds = [0.0, 0.0, 0.0, 0.0]

        self.command_subscriber = self.create_subscription(
            Float64MultiArray,
            command_topic,
            self.command_callback,
            10,
        )
        self.actuator_publisher = self.create_publisher(Actuators, actuator_topic, 10)
        self.arm_service = self.create_service(SetBool, "/quadcopter/arm", self.handle_arm_request)
        self.publish_timer = self.create_timer(1.0 / max(publish_rate_hz, 1.0), self.publish_actuators)

        self.get_logger().info(
            "Motor command gate ready. Publish Float64MultiArray[4] rotor speeds to "
            f"{command_topic} and arm via /quadcopter/arm."
        )

    def handle_arm_request(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        self.armed = request.data
        if not self.armed:
            self.desired_speeds = [0.0, 0.0, 0.0, 0.0]
            self.last_valid_command_time = None
        response.success = True
        response.message = "armed" if self.armed else "disarmed"
        self.get_logger().info(f"Vehicle {response.message}.")
        return response

    def command_callback(self, message: Float64MultiArray) -> None:
        values = list(message.data)
        if len(values) != 4:
            self.invalidate_command("Expected exactly 4 motor speeds.")
            return

        sanitized: List[float] = []
        for value in values:
            if not math.isfinite(value):
                self.invalidate_command("Received a non-finite motor speed.")
                return
            if value < 0.0:
                self.invalidate_command("Received a negative motor speed.")
                return
            sanitized.append(min(value, self.max_motor_speed))

        self.desired_speeds = sanitized
        self.last_valid_command_time = self.get_clock().now()

    def invalidate_command(self, reason: str) -> None:
        self.desired_speeds = [0.0, 0.0, 0.0, 0.0]
        self.last_valid_command_time = None
        self.get_logger().warning(reason)

    def publish_actuators(self) -> None:
        output = [0.0, 0.0, 0.0, 0.0]
        if self.armed and self.last_valid_command_time is not None:
            if (self.get_clock().now() - self.last_valid_command_time) <= self.command_timeout:
                output = list(self.desired_speeds)

        message = Actuators()
        if hasattr(message, "header"):
            message.header.stamp = self.get_clock().now().to_msg()
        if hasattr(message, "normalized"):
            message.normalized = []
        if hasattr(message, "position"):
            message.position = []
        if hasattr(message, "velocity"):
            message.velocity = output
        if hasattr(message, "effort"):
            message.effort = []

        self.actuator_publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = MotorCommandGate()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
