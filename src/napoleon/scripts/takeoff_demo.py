#!/usr/bin/env python3
from enum import Enum
from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool


class DemoState(Enum):
    WAITING_FOR_ARM_SERVICE = 1
    ARMING = 2
    SPINUP = 3
    HOLD = 4
    RAMP_DOWN = 5
    COMPLETE = 6


class TakeoffDemo(Node):
    def __init__(self) -> None:
        super().__init__("takeoff_demo")

        self.declare_parameter("arm_service", "/quadcopter/arm")
        self.declare_parameter("motor_command_topic", "/quadcopter/command/motor_speeds")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("spinup_speed_rad_s", 550.0)
        self.declare_parameter("takeoff_speed_rad_s", 575.0)
        self.declare_parameter("spinup_duration_sec", 1.0)
        self.declare_parameter("hold_duration_sec", 1.0)
        self.declare_parameter("ramp_down_duration_sec", 1.0)

        arm_service = self.get_parameter("arm_service").get_parameter_value().string_value
        command_topic = self.get_parameter("motor_command_topic").get_parameter_value().string_value
        publish_rate_hz = self.get_parameter("publish_rate_hz").get_parameter_value().double_value

        self.spinup_speed = self.get_parameter("spinup_speed_rad_s").get_parameter_value().double_value
        self.takeoff_speed = self.get_parameter("takeoff_speed_rad_s").get_parameter_value().double_value
        self.spinup_duration = self.get_parameter("spinup_duration_sec").get_parameter_value().double_value
        self.hold_duration = self.get_parameter("hold_duration_sec").get_parameter_value().double_value
        self.ramp_down_duration = (
            self.get_parameter("ramp_down_duration_sec").get_parameter_value().double_value
        )

        self.state = DemoState.WAITING_FOR_ARM_SERVICE
        self.state_start_time = None
        self.arm_request_future = None

        self.publisher = self.create_publisher(Float64MultiArray, command_topic, 10)
        self.arm_client = self.create_client(SetBool, arm_service)
        self.timer = self.create_timer(1.0 / max(publish_rate_hz, 1.0), self.tick)

    def tick(self) -> None:
        now = self.get_clock().now()
        if self.state_start_time is None:
            self.state_start_time = now

        if self.state == DemoState.WAITING_FOR_ARM_SERVICE:
            if self.arm_client.wait_for_service(timeout_sec=0.0):
                request = SetBool.Request()
                request.data = True
                self.arm_request_future = self.arm_client.call_async(request)
                self.state = DemoState.ARMING
                self.state_start_time = now
                self.get_logger().info("Arming quadcopter for takeoff demo.")
            return

        if self.state == DemoState.ARMING:
            if self.arm_request_future is not None and self.arm_request_future.done():
                response = self.arm_request_future.result()
                if response is None or not response.success:
                    self.get_logger().error("Failed to arm quadcopter for takeoff demo.")
                    self.state = DemoState.COMPLETE
                    return
                self.state = DemoState.SPINUP
                self.state_start_time = now
                self.get_logger().info("Starting motor ramp.")
            self.publish_speeds([0.0, 0.0, 0.0, 0.0])
            return

        elapsed = (now - self.state_start_time).nanoseconds * 1e-9
        if self.state == DemoState.SPINUP:
            progress = min(max(elapsed / max(self.spinup_duration, 1e-6), 0.0), 1.0)
            speed = self.spinup_speed + (self.takeoff_speed - self.spinup_speed) * progress
            self.publish_uniform_speed(speed)
            if progress >= 1.0:
                self.state = DemoState.HOLD
                self.state_start_time = now
                self.get_logger().info("Holding takeoff command.")
            return

        if self.state == DemoState.HOLD:
            self.publish_uniform_speed(self.takeoff_speed)
            if elapsed >= self.hold_duration:
                self.state = DemoState.RAMP_DOWN
                self.state_start_time = now
                self.get_logger().info("Ramping motors back down.")
            return

        if self.state == DemoState.RAMP_DOWN:
            progress = min(max(elapsed / max(self.ramp_down_duration, 1e-6), 0.0), 1.0)
            speed = self.takeoff_speed * (1.0 - progress)
            self.publish_uniform_speed(speed)
            if progress >= 1.0:
                self.state = DemoState.COMPLETE
                self.state_start_time = now
                self.disarm()
                self.get_logger().info("Takeoff demo complete.")
            return

        self.publish_uniform_speed(0.0)

    def publish_uniform_speed(self, speed: float) -> None:
        self.publish_speeds([speed, speed, speed, speed])

    def publish_speeds(self, speeds: List[float]) -> None:
        self.publisher.publish(Float64MultiArray(data=speeds))

    def disarm(self) -> None:
        if not self.arm_client.service_is_ready():
            return
        request = SetBool.Request()
        request.data = False
        self.arm_client.call_async(request)


def main() -> None:
    rclpy.init()
    node = TakeoffDemo()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
