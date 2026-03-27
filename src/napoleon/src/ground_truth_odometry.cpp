#include <algorithm>
#include <array>
#include <cstdint>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace {

using Vector3 = std::array<double, 3>;
using Quaternion = std::array<double, 4>;
constexpr double kPi = 3.14159265358979323846;

std::string normalizeFrame(const std::string & frame_id) {
  std::size_t start = 0;
  while (start < frame_id.size() && frame_id[start] == '/') {
    ++start;
  }

  std::size_t end = frame_id.size();
  while (end > start && frame_id[end - 1] == '/') {
    --end;
  }

  return frame_id.substr(start, end - start);
}

double wrapAngle(double angle) {
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

Quaternion quatConjugate(const Quaternion & q) {
  return {-q[0], -q[1], -q[2], q[3]};
}

Quaternion quatMultiply(const Quaternion & q1, const Quaternion & q2) {
  return {
      q1[3] * q2[0] + q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1],
      q1[3] * q2[1] - q1[0] * q2[2] + q1[1] * q2[3] + q1[2] * q2[0],
      q1[3] * q2[2] + q1[0] * q2[1] - q1[1] * q2[0] + q1[2] * q2[3],
      q1[3] * q2[3] - q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2],
  };
}

Quaternion quatNormalize(const Quaternion & q) {
  const auto norm =
      std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (norm == 0.0) {
    return {0.0, 0.0, 0.0, 1.0};
  }

  return {q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm};
}

Vector3 quatToEuler(const Quaternion & quat) {
  const auto sinr_cosp = 2.0 * (quat[3] * quat[0] + quat[1] * quat[2]);
  const auto cosr_cosp = 1.0 - 2.0 * (quat[0] * quat[0] + quat[1] * quat[1]);
  const auto roll = std::atan2(sinr_cosp, cosr_cosp);

  const auto sinp = 2.0 * (quat[3] * quat[1] - quat[2] * quat[0]);
  const auto pitch =
      std::abs(sinp) >= 1.0 ? std::copysign(kPi / 2.0, sinp) : std::asin(sinp);

  const auto siny_cosp = 2.0 * (quat[3] * quat[2] + quat[0] * quat[1]);
  const auto cosy_cosp = 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]);
  const auto yaw = std::atan2(siny_cosp, cosy_cosp);

  return {roll, pitch, yaw};
}

Quaternion quatFromYaw(double yaw) {
  return {0.0, 0.0, std::sin(yaw * 0.5), std::cos(yaw * 0.5)};
}

Vector3 rotateVectorByQuat(const Vector3 & vector, const Quaternion & quat) {
  const auto ix = quat[3] * vector[0] + quat[1] * vector[2] - quat[2] * vector[1];
  const auto iy = quat[3] * vector[1] + quat[2] * vector[0] - quat[0] * vector[2];
  const auto iz = quat[3] * vector[2] + quat[0] * vector[1] - quat[1] * vector[0];
  const auto iw = -quat[0] * vector[0] - quat[1] * vector[1] - quat[2] * vector[2];

  return {
      ix * quat[3] + iw * -quat[0] + iy * -quat[2] - iz * -quat[1],
      iy * quat[3] + iw * -quat[1] + iz * -quat[0] - ix * -quat[2],
      iz * quat[3] + iw * -quat[2] + ix * -quat[1] - iy * -quat[0],
  };
}

} // namespace

class GroundTruthOdometry : public rclcpp::Node {
public:
  GroundTruthOdometry() : Node("ground_truth_odometry") {
    const auto pose_topic =
        declare_parameter<std::string>("pose_topic", "/quadcopter/internal/dynamic_pose");
    full_odom_topic_ =
        declare_parameter<std::string>("full_odom_topic", "/quadcopter/state/odom");
    projected_odom_topic_ =
        declare_parameter<std::string>("projected_odom_topic", "/odom");
    world_frame_ = declare_parameter<std::string>("world_frame", "odom");
    body_frame_ = declare_parameter<std::string>("body_frame", "base_link");
    projected_body_frame_ =
        declare_parameter<std::string>("projected_body_frame", "base_footprint");
    model_name_ = declare_parameter<std::string>("model_name", "quadcopter");
    link_name_ = declare_parameter<std::string>("link_name", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);

    pose_subscriber_ = create_subscription<tf2_msgs::msg::TFMessage>(
        pose_topic, rclcpp::QoS(1),
        std::bind(&GroundTruthOdometry::poseCallback, this, std::placeholders::_1));
    full_odom_publisher_ =
        create_publisher<nav_msgs::msg::Odometry>(full_odom_topic_, rclcpp::QoS(10));
    projected_odom_publisher_ =
        create_publisher<nav_msgs::msg::Odometry>(projected_odom_topic_, rclcpp::QoS(10));
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
  }

private:
  const geometry_msgs::msg::TransformStamped *
  selectTransform(const tf2_msgs::msg::TFMessage::SharedPtr & message) const {
    for (const auto & transform : message->transforms) {
      if (matchesTarget(transform.child_frame_id)) {
        return &transform;
      }
    }

    const auto all_empty =
        !message->transforms.empty() &&
        std::all_of(message->transforms.begin(), message->transforms.end(),
                    [](const auto & transform) { return transform.child_frame_id.empty(); });
    if (all_empty) {
      return &message->transforms.front();
    }

    return nullptr;
  }

  bool matchesTarget(const std::string & child_frame_id) const {
    const auto normalized = normalizeFrame(child_frame_id);
    const std::unordered_set<std::string> expected = {
        model_name_,
        model_name_ + "/" + link_name_,
        model_name_ + "::" + link_name_,
        "model/" + model_name_,
        "model/" + model_name_ + "/link/" + link_name_,
    };

    if (expected.count(normalized) != 0U) {
      return true;
    }

    return normalized.size() >= link_name_.size() &&
           normalized.find(model_name_) != std::string::npos &&
           normalized.rfind("/" + link_name_) == normalized.size() - link_name_.size() - 1U;
  }

  std::pair<rclcpp::Time, builtin_interfaces::msg::Time>
  resolveTimestamp(const geometry_msgs::msg::TransformStamped & transform) const {
    if (transform.header.stamp.sec != 0 || transform.header.stamp.nanosec != 0) {
      return {rclcpp::Time(transform.header.stamp), transform.header.stamp};
    }

    const auto now_stamp = now();
    const auto total_nanoseconds = now_stamp.nanoseconds();
    builtin_interfaces::msg::Time stamp_message;
    stamp_message.sec = static_cast<int32_t>(total_nanoseconds / 1000000000LL);
    stamp_message.nanosec =
        static_cast<uint32_t>(total_nanoseconds % 1000000000LL);
    return {now_stamp, stamp_message};
  }

  void fillCovariance(nav_msgs::msg::Odometry & odom) const {
    odom.pose.covariance[0] = 0.02;
    odom.pose.covariance[7] = 0.02;
    odom.pose.covariance[14] = 0.04;
    odom.pose.covariance[21] = 0.01;
    odom.pose.covariance[28] = 0.01;
    odom.pose.covariance[35] = 0.02;
    odom.twist.covariance[0] = 0.05;
    odom.twist.covariance[7] = 0.05;
    odom.twist.covariance[14] = 0.08;
    odom.twist.covariance[21] = 0.02;
    odom.twist.covariance[28] = 0.02;
    odom.twist.covariance[35] = 0.04;
  }

  void publishFullOdometry(const builtin_interfaces::msg::Time & stamp, const Vector3 & position,
                           const Quaternion & orientation, const Vector3 & linear_velocity,
                           const Vector3 & angular_velocity) {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = world_frame_;
    odom.child_frame_id = body_frame_;
    odom.pose.pose.position.x = position[0];
    odom.pose.pose.position.y = position[1];
    odom.pose.pose.position.z = position[2];
    odom.pose.pose.orientation.x = orientation[0];
    odom.pose.pose.orientation.y = orientation[1];
    odom.pose.pose.orientation.z = orientation[2];
    odom.pose.pose.orientation.w = orientation[3];
    odom.twist.twist.linear.x = linear_velocity[0];
    odom.twist.twist.linear.y = linear_velocity[1];
    odom.twist.twist.linear.z = linear_velocity[2];
    odom.twist.twist.angular.x = angular_velocity[0];
    odom.twist.twist.angular.y = angular_velocity[1];
    odom.twist.twist.angular.z = angular_velocity[2];
    fillCovariance(odom);
    full_odom_publisher_->publish(odom);
  }

  void publishProjectedOdometry(const builtin_interfaces::msg::Time & stamp,
                                const Vector3 & position, double yaw,
                                const Vector3 & planar_velocity, double yaw_rate) {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = world_frame_;
    odom.child_frame_id = projected_body_frame_;
    odom.pose.pose.position.x = position[0];
    odom.pose.pose.position.y = position[1];
    odom.pose.pose.position.z = 0.0;
    const auto yaw_orientation = quatFromYaw(yaw);
    odom.pose.pose.orientation.x = yaw_orientation[0];
    odom.pose.pose.orientation.y = yaw_orientation[1];
    odom.pose.pose.orientation.z = yaw_orientation[2];
    odom.pose.pose.orientation.w = yaw_orientation[3];
    odom.twist.twist.linear.x = planar_velocity[0];
    odom.twist.twist.linear.y = planar_velocity[1];
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = yaw_rate;
    fillCovariance(odom);
    projected_odom_publisher_->publish(odom);
  }

  void publishTransforms(const builtin_interfaces::msg::Time & stamp,
                         const Vector3 & position, const Quaternion & orientation,
                         double yaw) {
    if (tf_broadcaster_ == nullptr) {
      return;
    }

    const auto yaw_orientation = quatFromYaw(yaw);

    geometry_msgs::msg::TransformStamped odom_to_projected;
    odom_to_projected.header.stamp = stamp;
    odom_to_projected.header.frame_id = world_frame_;
    odom_to_projected.child_frame_id = projected_body_frame_;
    odom_to_projected.transform.translation.x = position[0];
    odom_to_projected.transform.translation.y = position[1];
    odom_to_projected.transform.translation.z = 0.0;
    odom_to_projected.transform.rotation.x = yaw_orientation[0];
    odom_to_projected.transform.rotation.y = yaw_orientation[1];
    odom_to_projected.transform.rotation.z = yaw_orientation[2];
    odom_to_projected.transform.rotation.w = yaw_orientation[3];
    tf_broadcaster_->sendTransform(odom_to_projected);

    geometry_msgs::msg::TransformStamped projected_to_body;
    projected_to_body.header.stamp = stamp;
    projected_to_body.header.frame_id = projected_body_frame_;
    projected_to_body.child_frame_id = body_frame_;
    projected_to_body.transform.translation.x = 0.0;
    projected_to_body.transform.translation.y = 0.0;
    projected_to_body.transform.translation.z = position[2];
    const auto relative_orientation =
        quatNormalize(quatMultiply(quatConjugate(yaw_orientation), orientation));
    projected_to_body.transform.rotation.x = relative_orientation[0];
    projected_to_body.transform.rotation.y = relative_orientation[1];
    projected_to_body.transform.rotation.z = relative_orientation[2];
    projected_to_body.transform.rotation.w = relative_orientation[3];
    tf_broadcaster_->sendTransform(projected_to_body);
  }

  void poseCallback(const tf2_msgs::msg::TFMessage::SharedPtr message) {
    const auto * transform = selectTransform(message);
    if (transform == nullptr) {
      return;
    }

    const auto [timestamp, stamp] = resolveTimestamp(*transform);
    const Vector3 position = {transform->transform.translation.x, transform->transform.translation.y,
                              transform->transform.translation.z};
    const Quaternion orientation = quatNormalize(
        {transform->transform.rotation.x, transform->transform.rotation.y,
         transform->transform.rotation.z, transform->transform.rotation.w});
    const auto euler = quatToEuler(orientation);
    const auto yaw = euler[2];

    Vector3 world_linear_velocity = {0.0, 0.0, 0.0};
    Vector3 body_linear_velocity = {0.0, 0.0, 0.0};
    Vector3 body_angular_velocity = {0.0, 0.0, 0.0};
    double yaw_rate = 0.0;

    if (previous_time_.has_value() && timestamp > *previous_time_) {
      const auto dt = (timestamp - *previous_time_).seconds();
      const Vector3 delta_position = {position[0] - (*previous_position_)[0],
                                      position[1] - (*previous_position_)[1],
                                      position[2] - (*previous_position_)[2]};
      world_linear_velocity = {delta_position[0] / dt, delta_position[1] / dt,
                               delta_position[2] / dt};
      body_linear_velocity =
          rotateVectorByQuat(world_linear_velocity, quatConjugate(orientation));

      auto delta_orientation =
          quatMultiply(quatConjugate(*previous_orientation_), orientation);
      delta_orientation = quatNormalize(delta_orientation);
      const auto sin_half_angle = std::sqrt(delta_orientation[0] * delta_orientation[0] +
                                            delta_orientation[1] * delta_orientation[1] +
                                            delta_orientation[2] * delta_orientation[2]);
      if (sin_half_angle > 1e-6) {
        const Vector3 axis = {delta_orientation[0] / sin_half_angle,
                              delta_orientation[1] / sin_half_angle,
                              delta_orientation[2] / sin_half_angle};
        const auto angle = 2.0 * std::atan2(sin_half_angle, delta_orientation[3]);
        body_angular_velocity = {axis[0] * angle / dt, axis[1] * angle / dt,
                                 axis[2] * angle / dt};
      }

      if (previous_yaw_.has_value()) {
        yaw_rate = wrapAngle(yaw - *previous_yaw_) / dt;
      }
    }

    const auto yaw_orientation = quatFromYaw(yaw);
    auto planar_velocity = rotateVectorByQuat(world_linear_velocity, quatConjugate(yaw_orientation));
    planar_velocity[2] = 0.0;

    publishFullOdometry(stamp, position, orientation, body_linear_velocity, body_angular_velocity);
    publishProjectedOdometry(stamp, position, yaw, planar_velocity, yaw_rate);
    publishTransforms(stamp, position, orientation, yaw);

    previous_time_ = timestamp;
    previous_position_ = position;
    previous_orientation_ = orientation;
    previous_yaw_ = yaw;
  }

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr pose_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr full_odom_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr projected_odom_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string full_odom_topic_;
  std::string projected_odom_topic_;
  std::string world_frame_;
  std::string body_frame_;
  std::string projected_body_frame_;
  std::string model_name_;
  std::string link_name_;
  bool publish_tf_{true};

  std::optional<rclcpp::Time> previous_time_;
  std::optional<Vector3> previous_position_;
  std::optional<Quaternion> previous_orientation_;
  std::optional<double> previous_yaw_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundTruthOdometry>());
  rclcpp::shutdown();
  return 0;
}
