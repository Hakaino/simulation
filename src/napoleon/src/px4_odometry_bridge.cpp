#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace {

constexpr double kPi = 3.14159265358979323846;
using Quaternion = std::array<double, 4>;

double wrapAngle(double angle) {
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

Quaternion quatFromYaw(double yaw) {
  return {0.0, 0.0, std::sin(yaw * 0.5), std::cos(yaw * 0.5)};
}

Quaternion quatConjugate(const Quaternion & quat) {
  return {-quat[0], -quat[1], -quat[2], quat[3]};
}

Quaternion quatMultiply(const Quaternion & lhs, const Quaternion & rhs) {
  return {
      lhs[3] * rhs[0] + lhs[0] * rhs[3] + lhs[1] * rhs[2] - lhs[2] * rhs[1],
      lhs[3] * rhs[1] - lhs[0] * rhs[2] + lhs[1] * rhs[3] + lhs[2] * rhs[0],
      lhs[3] * rhs[2] + lhs[0] * rhs[1] - lhs[1] * rhs[0] + lhs[2] * rhs[3],
      lhs[3] * rhs[3] - lhs[0] * rhs[0] - lhs[1] * rhs[1] - lhs[2] * rhs[2],
  };
}

Quaternion quatNormalize(const Quaternion & quat) {
  const auto norm = std::sqrt(quat[0] * quat[0] + quat[1] * quat[1] +
                              quat[2] * quat[2] + quat[3] * quat[3]);
  if (norm <= 1e-9) {
    return {0.0, 0.0, 0.0, 1.0};
  }

  return {quat[0] / norm, quat[1] / norm, quat[2] / norm, quat[3] / norm};
}

double yawFromQuaternion(const Quaternion & quat) {
  const auto siny_cosp = 2.0 * (quat[3] * quat[2] + quat[0] * quat[1]);
  const auto cosy_cosp = 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]);
  return wrapAngle(std::atan2(siny_cosp, cosy_cosp));
}

}  // namespace

class Px4OdometryBridge : public rclcpp::Node {
public:
  Px4OdometryBridge() : Node("px4_odometry_bridge") {
    const auto source_topic = declare_parameter<std::string>(
        "source_topic", "/mavros/local_position/odom");
    full_odom_topic_ = declare_parameter<std::string>(
        "full_odom_topic", "/quadcopter/state/odom");
    projected_odom_topic_ =
        declare_parameter<std::string>("projected_odom_topic", "/odom");
    world_frame_ = declare_parameter<std::string>("world_frame", "odom");
    body_frame_ = declare_parameter<std::string>("body_frame", "base_link");
    projected_body_frame_ =
        declare_parameter<std::string>("projected_body_frame", "base_footprint");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);

    source_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        source_topic, rclcpp::SensorDataQoS(),
        std::bind(&Px4OdometryBridge::odomCallback, this, std::placeholders::_1));
    full_odom_publisher_ =
        create_publisher<nav_msgs::msg::Odometry>(full_odom_topic_, rclcpp::QoS(10));
    projected_odom_publisher_ =
        create_publisher<nav_msgs::msg::Odometry>(projected_odom_topic_, rclcpp::QoS(10));

    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr message) {
    const auto orientation = quatNormalize(
        Quaternion{message->pose.pose.orientation.x, message->pose.pose.orientation.y,
                   message->pose.pose.orientation.z, message->pose.pose.orientation.w});
    const auto yaw = yawFromQuaternion(orientation);
    const auto yaw_orientation = quatFromYaw(yaw);
    const auto relative_orientation =
        quatNormalize(quatMultiply(quatConjugate(yaw_orientation), orientation));

    nav_msgs::msg::Odometry full_odom = *message;
    full_odom.header.frame_id = world_frame_;
    full_odom.child_frame_id = body_frame_;
    full_odom_publisher_->publish(full_odom);

    nav_msgs::msg::Odometry projected_odom;
    projected_odom.header.stamp = message->header.stamp;
    projected_odom.header.frame_id = world_frame_;
    projected_odom.child_frame_id = projected_body_frame_;
    projected_odom.pose.pose.position.x = message->pose.pose.position.x;
    projected_odom.pose.pose.position.y = message->pose.pose.position.y;
    projected_odom.pose.pose.position.z = 0.0;
    projected_odom.pose.pose.orientation.x = yaw_orientation[0];
    projected_odom.pose.pose.orientation.y = yaw_orientation[1];
    projected_odom.pose.pose.orientation.z = yaw_orientation[2];
    projected_odom.pose.pose.orientation.w = yaw_orientation[3];
    projected_odom.twist.twist.linear.x = message->twist.twist.linear.x;
    projected_odom.twist.twist.linear.y = message->twist.twist.linear.y;
    projected_odom.twist.twist.linear.z = 0.0;
    projected_odom.twist.twist.angular.x = 0.0;
    projected_odom.twist.twist.angular.y = 0.0;
    projected_odom.twist.twist.angular.z = message->twist.twist.angular.z;
    projected_odom.pose.covariance = message->pose.covariance;
    projected_odom.twist.covariance = message->twist.covariance;
    projected_odom.pose.covariance[14] = 9999.0;
    projected_odom.pose.covariance[21] = 9999.0;
    projected_odom.pose.covariance[28] = 9999.0;
    projected_odom.twist.covariance[14] = 9999.0;
    projected_odom.twist.covariance[21] = 9999.0;
    projected_odom.twist.covariance[28] = 9999.0;
    projected_odom_publisher_->publish(projected_odom);

    if (tf_broadcaster_ == nullptr) {
      return;
    }

    geometry_msgs::msg::TransformStamped odom_to_projected;
    odom_to_projected.header.stamp = message->header.stamp;
    odom_to_projected.header.frame_id = world_frame_;
    odom_to_projected.child_frame_id = projected_body_frame_;
    odom_to_projected.transform.translation.x = message->pose.pose.position.x;
    odom_to_projected.transform.translation.y = message->pose.pose.position.y;
    odom_to_projected.transform.translation.z = 0.0;
    odom_to_projected.transform.rotation.x = yaw_orientation[0];
    odom_to_projected.transform.rotation.y = yaw_orientation[1];
    odom_to_projected.transform.rotation.z = yaw_orientation[2];
    odom_to_projected.transform.rotation.w = yaw_orientation[3];
    tf_broadcaster_->sendTransform(odom_to_projected);

    geometry_msgs::msg::TransformStamped projected_to_body;
    projected_to_body.header.stamp = message->header.stamp;
    projected_to_body.header.frame_id = projected_body_frame_;
    projected_to_body.child_frame_id = body_frame_;
    projected_to_body.transform.translation.x = 0.0;
    projected_to_body.transform.translation.y = 0.0;
    projected_to_body.transform.translation.z = message->pose.pose.position.z;
    projected_to_body.transform.rotation.x = relative_orientation[0];
    projected_to_body.transform.rotation.y = relative_orientation[1];
    projected_to_body.transform.rotation.z = relative_orientation[2];
    projected_to_body.transform.rotation.w = relative_orientation[3];
    tf_broadcaster_->sendTransform(projected_to_body);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr source_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr full_odom_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr projected_odom_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string full_odom_topic_;
  std::string projected_odom_topic_;
  std::string world_frame_;
  std::string body_frame_;
  std::string projected_body_frame_;
  bool publish_tf_{true};
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4OdometryBridge>());
  rclcpp::shutdown();
  return 0;
}
