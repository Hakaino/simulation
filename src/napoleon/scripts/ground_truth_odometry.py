#!/usr/bin/env python3
import math
from typing import Optional, Tuple

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


def _normalize_frame(frame_id: str) -> str:
    return frame_id.strip("/")


def _quat_conjugate(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    x, y, z, w = q
    return (-x, -y, -z, w)


def _quat_multiply(
    q1: Tuple[float, float, float, float],
    q2: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def _quat_normalize(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    norm = math.sqrt(sum(component * component for component in q))
    if norm == 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(component / norm for component in q)


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


class GroundTruthOdometry(Node):
    def __init__(self) -> None:
        super().__init__("ground_truth_odometry")

        self.declare_parameter("pose_topic", "/quadcopter/internal/dynamic_pose")
        self.declare_parameter("odom_topic", "/quadcopter/state/odom")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("body_frame", "base_link")
        self.declare_parameter("model_name", "quadcopter")
        self.declare_parameter("link_name", "base_link")

        pose_topic = self.get_parameter("pose_topic").get_parameter_value().string_value
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.world_frame = self.get_parameter("world_frame").get_parameter_value().string_value
        self.body_frame = self.get_parameter("body_frame").get_parameter_value().string_value
        self.model_name = self.get_parameter("model_name").get_parameter_value().string_value
        self.link_name = self.get_parameter("link_name").get_parameter_value().string_value

        self.previous_time: Optional[float] = None
        self.previous_position: Optional[Tuple[float, float, float]] = None
        self.previous_orientation: Optional[Tuple[float, float, float, float]] = None

        self.pose_subscriber = self.create_subscription(TFMessage, pose_topic, self.pose_callback, 50)
        self.odom_publisher = self.create_publisher(Odometry, odom_topic, 50)

    def select_transform(self, message: TFMessage):
        for transform in message.transforms:
            if self.matches_target(transform.child_frame_id):
                return transform

        # ros_gz_bridge currently drops pose names for Pose_V -> TFMessage in this setup.
        # Gazebo still keeps the base pose as the first transform, so fall back to that.
        if message.transforms and all(not transform.child_frame_id for transform in message.transforms):
            return message.transforms[0]

        return None

    def resolve_timestamp(self, transform) -> Tuple[float, object]:
        stamp = transform.header.stamp
        if stamp.sec != 0 or stamp.nanosec != 0:
            timestamp = float(stamp.sec) + float(stamp.nanosec) * 1e-9
            return timestamp, stamp

        now = self.get_clock().now().to_msg()
        timestamp = float(now.sec) + float(now.nanosec) * 1e-9
        return timestamp, now

    def matches_target(self, child_frame_id: str) -> bool:
        normalized = _normalize_frame(child_frame_id)
        expected = {
            self.model_name,
            f"{self.model_name}/{self.link_name}",
            f"{self.model_name}::{self.link_name}",
            f"model/{self.model_name}",
            f"model/{self.model_name}/link/{self.link_name}",
        }
        if normalized in expected:
            return True
        return normalized.endswith(f"/{self.link_name}") and self.model_name in normalized

    def pose_callback(self, message: TFMessage) -> None:
        transform = self.select_transform(message)
        if transform is None:
            return

        timestamp, stamp = self.resolve_timestamp(transform)
        position = (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        )
        orientation = _quat_normalize(
            (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            )
        )

        linear_velocity = (0.0, 0.0, 0.0)
        angular_velocity = (0.0, 0.0, 0.0)
        if self.previous_time is not None and timestamp > self.previous_time:
            dt = timestamp - self.previous_time
            delta_position = tuple(position[index] - self.previous_position[index] for index in range(3))
            world_linear_velocity = tuple(component / dt for component in delta_position)
            linear_velocity = _rotate_vector_by_quat(world_linear_velocity, _quat_conjugate(orientation))

            delta_orientation = _quat_multiply(_quat_conjugate(self.previous_orientation), orientation)
            delta_orientation = _quat_normalize(delta_orientation)
            sin_half_angle = math.sqrt(
                delta_orientation[0] ** 2 + delta_orientation[1] ** 2 + delta_orientation[2] ** 2
            )
            if sin_half_angle > 1e-6:
                axis = (
                    delta_orientation[0] / sin_half_angle,
                    delta_orientation[1] / sin_half_angle,
                    delta_orientation[2] / sin_half_angle,
                )
                angle = 2.0 * math.atan2(sin_half_angle, delta_orientation[3])
                angular_velocity = tuple(component * angle / dt for component in axis)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.world_frame
        odom.child_frame_id = self.body_frame
        odom.pose.pose.position.x = position[0]
        odom.pose.pose.position.y = position[1]
        odom.pose.pose.position.z = position[2]
        odom.pose.pose.orientation.x = orientation[0]
        odom.pose.pose.orientation.y = orientation[1]
        odom.pose.pose.orientation.z = orientation[2]
        odom.pose.pose.orientation.w = orientation[3]
        odom.twist.twist.linear.x = linear_velocity[0]
        odom.twist.twist.linear.y = linear_velocity[1]
        odom.twist.twist.linear.z = linear_velocity[2]
        odom.twist.twist.angular.x = angular_velocity[0]
        odom.twist.twist.angular.y = angular_velocity[1]
        odom.twist.twist.angular.z = angular_velocity[2]
        self.odom_publisher.publish(odom)

        self.previous_time = timestamp
        self.previous_position = position
        self.previous_orientation = orientation


def main() -> None:
    rclpy.init()
    node = GroundTruthOdometry()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
