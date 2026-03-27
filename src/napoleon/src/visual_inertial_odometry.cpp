#include <algorithm>
#include <array>
#include <cstdint>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace {

using Vector3 = std::array<double, 3>;
using Quaternion = std::array<double, 4>;
constexpr double kPi = 3.14159265358979323846;

double clamp(double value, double lower, double upper) {
  return std::clamp(value, lower, upper);
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

double blend(double current, double measurement, double weight) {
  return (1.0 - weight) * current + weight * measurement;
}

double vectorNorm(const Vector3 & vector) {
  return std::sqrt(vector[0] * vector[0] + vector[1] * vector[1] +
                   vector[2] * vector[2]);
}

Vector3 addVectors(const Vector3 & lhs, const Vector3 & rhs) {
  return {lhs[0] + rhs[0], lhs[1] + rhs[1], lhs[2] + rhs[2]};
}

Vector3 scaleVector(const Vector3 & vector, double scale) {
  return {vector[0] * scale, vector[1] * scale, vector[2] * scale};
}

Quaternion quatNormalize(const Quaternion & quat) {
  const auto norm = std::sqrt(quat[0] * quat[0] + quat[1] * quat[1] +
                              quat[2] * quat[2] + quat[3] * quat[3]);
  if (norm <= 1e-9) {
    return {0.0, 0.0, 0.0, 1.0};
  }

  return {quat[0] / norm, quat[1] / norm, quat[2] / norm, quat[3] / norm};
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

Quaternion quatFromEuler(double roll, double pitch, double yaw) {
  const auto cr = std::cos(roll * 0.5);
  const auto sr = std::sin(roll * 0.5);
  const auto cp = std::cos(pitch * 0.5);
  const auto sp = std::sin(pitch * 0.5);
  const auto cy = std::cos(yaw * 0.5);
  const auto sy = std::sin(yaw * 0.5);

  return quatNormalize({
      sr * cp * cy - cr * sp * sy,
      cr * sp * cy + sr * cp * sy,
      cr * cp * sy - sr * sp * cy,
      cr * cp * cy + sr * sp * sy,
  });
}

Quaternion quatFromYaw(double yaw) {
  return {0.0, 0.0, std::sin(yaw * 0.5), std::cos(yaw * 0.5)};
}

Quaternion quatFromAngularVelocity(const Vector3 & angular_velocity_body, double dt) {
  const auto angle = vectorNorm(angular_velocity_body) * dt;
  if (angle <= 1e-9) {
    return {0.0, 0.0, 0.0, 1.0};
  }

  const auto axis = scaleVector(angular_velocity_body, 1.0 / vectorNorm(angular_velocity_body));
  const auto half_angle = angle * 0.5;
  const auto sin_half_angle = std::sin(half_angle);
  return quatNormalize({
      axis[0] * sin_half_angle,
      axis[1] * sin_half_angle,
      axis[2] * sin_half_angle,
      std::cos(half_angle),
  });
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

double median(std::vector<double> samples) {
  if (samples.empty()) {
    return 0.0;
  }

  const auto middle = samples.begin() + static_cast<std::ptrdiff_t>(samples.size() / 2U);
  std::nth_element(samples.begin(), middle, samples.end());
  auto result = *middle;
  if ((samples.size() % 2U) == 0U) {
    const auto lower_middle =
        std::max_element(samples.begin(), middle);
    result = 0.5 * (result + *lower_middle);
  }
  return result;
}

builtin_interfaces::msg::Time toBuiltinTime(const rclcpp::Time & stamp) {
  const auto total_nanoseconds = stamp.nanoseconds();
  builtin_interfaces::msg::Time message;
  message.sec = static_cast<int32_t>(total_nanoseconds / 1000000000LL);
  message.nanosec = static_cast<uint32_t>(total_nanoseconds % 1000000000LL);
  return message;
}

} // namespace

class VisualInertialOdometry : public rclcpp::Node {
public:
  VisualInertialOdometry() : Node("visual_inertial_odometry") {
    const auto image_topic =
        declare_parameter<std::string>("image_topic", "/odom_camera/image_raw");
    const auto camera_info_topic =
        declare_parameter<std::string>("camera_info_topic", "/odom_camera/camera_info");
    const auto imu_topic = declare_parameter<std::string>("imu_topic", "/imu/data");
    const auto pressure_topic =
        declare_parameter<std::string>("pressure_topic", "/baro/data");
    full_odom_topic_ =
        declare_parameter<std::string>("full_odom_topic", "/quadcopter/state/odom");
    projected_odom_topic_ =
        declare_parameter<std::string>("projected_odom_topic", "/odom");
    world_frame_ = declare_parameter<std::string>("world_frame", "odom");
    body_frame_ = declare_parameter<std::string>("body_frame", "base_link");
    projected_body_frame_ =
        declare_parameter<std::string>("projected_body_frame", "base_footprint");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);

    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 100.0);
    max_imu_dt_ = declare_parameter<double>("max_imu_dt_sec", 0.03);
    accel_min_norm_ = declare_parameter<double>("accel_min_norm_mps2", 7.0);
    accel_max_norm_ = declare_parameter<double>("accel_max_norm_mps2", 12.5);
    accel_attitude_weight_ =
        declare_parameter<double>("accel_attitude_weight", 0.03);
    baro_altitude_weight_ =
        declare_parameter<double>("baro_altitude_weight", 0.35);
    baro_velocity_weight_ =
        declare_parameter<double>("baro_velocity_weight", 0.35);
    flow_velocity_weight_ =
        declare_parameter<double>("flow_velocity_weight", 0.55);
    flow_position_weight_ =
        declare_parameter<double>("flow_position_weight", 0.30);
    horizontal_velocity_damping_hz_ =
        declare_parameter<double>("horizontal_velocity_damping_hz", 0.20);
    stale_flow_damping_hz_ =
        declare_parameter<double>("stale_flow_damping_hz", 2.5);
    flow_timeout_sec_ = declare_parameter<double>("flow_timeout_sec", 0.25);
    min_flow_altitude_m_ =
        declare_parameter<double>("min_flow_altitude_m", 0.35);
    max_flow_altitude_m_ =
        declare_parameter<double>("max_flow_altitude_m", 4.0);
    max_tilt_for_flow_rad_ =
        declare_parameter<double>("max_tilt_for_flow_rad", 0.45);
    max_vertical_speed_for_flow_mps_ =
        declare_parameter<double>("max_vertical_speed_for_flow_mps", 0.45);
    max_planar_speed_mps_ =
        declare_parameter<double>("max_planar_speed_mps", 3.0);
    max_features_ = declare_parameter<int>("max_features", 260);
    min_features_ = declare_parameter<int>("min_features", 110);
    min_tracked_features_ = declare_parameter<int>("min_tracked_features", 60);
    quality_level_ = declare_parameter<double>("feature_quality_level", 0.01);
    min_feature_distance_px_ =
        declare_parameter<double>("min_feature_distance_px", 12.0);
    lk_window_size_px_ = declare_parameter<int>("lk_window_size_px", 21);
    lk_max_level_ = declare_parameter<int>("lk_max_level", 3);
    affine_ransac_threshold_px_ =
        declare_parameter<double>("affine_ransac_threshold_px", 2.5);
    camera_mount_roll_rad_ =
        declare_parameter<double>("camera_mount_roll_rad", 0.0);
    camera_mount_pitch_rad_ =
        declare_parameter<double>("camera_mount_pitch_rad", kPi / 2.0);
    camera_mount_yaw_rad_ =
        declare_parameter<double>("camera_mount_yaw_rad", 0.0);
    camera_optical_roll_rad_ =
        declare_parameter<double>("camera_optical_roll_rad", -kPi / 2.0);
    camera_optical_pitch_rad_ =
        declare_parameter<double>("camera_optical_pitch_rad", 0.0);
    camera_optical_yaw_rad_ =
        declare_parameter<double>("camera_optical_yaw_rad", -kPi / 2.0);

    body_from_optical_ = quatMultiply(
        quatFromEuler(camera_mount_roll_rad_, camera_mount_pitch_rad_,
                      camera_mount_yaw_rad_),
        quatFromEuler(camera_optical_roll_rad_, camera_optical_pitch_rad_,
                      camera_optical_yaw_rad_));

    const auto sensor_qos = rclcpp::SensorDataQoS();
    image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic, sensor_qos,
        std::bind(&VisualInertialOdometry::imageCallback, this, std::placeholders::_1));
    camera_info_subscriber_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic, sensor_qos,
        std::bind(&VisualInertialOdometry::cameraInfoCallback, this,
                  std::placeholders::_1));
    imu_subscriber_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, sensor_qos,
        std::bind(&VisualInertialOdometry::imuCallback, this, std::placeholders::_1));
    pressure_subscriber_ = create_subscription<sensor_msgs::msg::FluidPressure>(
        pressure_topic, sensor_qos,
        std::bind(&VisualInertialOdometry::pressureCallback, this,
                  std::placeholders::_1));

    full_odom_publisher_ =
        create_publisher<nav_msgs::msg::Odometry>(full_odom_topic_, rclcpp::QoS(10));
    projected_odom_publisher_ =
        create_publisher<nav_msgs::msg::Odometry>(projected_odom_topic_, rclcpp::QoS(10));
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    publish_timer_ = rclcpp::create_timer(
        this, get_clock(),
        rclcpp::Duration::from_seconds(1.0 / std::max(publish_rate_hz_, 1.0)),
        std::bind(&VisualInertialOdometry::publishState, this));

    RCLCPP_INFO(
        get_logger(),
        "Visual-inertial odometry ready. It uses downward optical flow, IMU attitude, and "
        "barometric altitude as the primary odometry source.");
  }

private:
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr message) {
    std::lock_guard<std::mutex> lock(mutex_);
    fx_ = message->k[0];
    fy_ = message->k[4];
    cx_ = message->k[2];
    cy_ = message->k[5];
    camera_info_received_ = fx_ > 1.0 && fy_ > 1.0;
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr message) {
    const auto stamp = resolveStamp(message->header.stamp);
    std::lock_guard<std::mutex> lock(mutex_);

    angular_velocity_body_ = {message->angular_velocity.x, message->angular_velocity.y,
                              message->angular_velocity.z};

    const Vector3 linear_acceleration = {message->linear_acceleration.x,
                                         message->linear_acceleration.y,
                                         message->linear_acceleration.z};

    if (!imu_initialized_) {
      orientation_world_body_ = orientationFromAcceleration(linear_acceleration);
      last_imu_time_ = stamp;
      last_state_time_ = stamp;
      imu_initialized_ = true;
      return;
    }

    auto dt = (stamp - *last_imu_time_).seconds();
    if (dt <= 0.0) {
      last_state_time_ = stamp;
      return;
    }

    dt = std::min(dt, max_imu_dt_);

    const auto horizontal_decay = std::exp(-horizontal_velocity_damping_hz_ * dt);
    velocity_world_[0] *= horizontal_decay;
    velocity_world_[1] *= horizontal_decay;
    if (previous_image_time_.has_value() &&
        (stamp - *previous_image_time_).seconds() > flow_timeout_sec_) {
      const auto stale_decay = std::exp(-stale_flow_damping_hz_ * dt);
      velocity_world_[0] *= stale_decay;
      velocity_world_[1] *= stale_decay;
    }

    position_world_ =
        addVectors(position_world_, scaleVector(velocity_world_, dt));

    auto integrated_orientation = quatNormalize(quatMultiply(
        orientation_world_body_, quatFromAngularVelocity(angular_velocity_body_, dt)));
    auto euler = quatToEuler(integrated_orientation);

    const auto acceleration_norm = vectorNorm(linear_acceleration);
    if (acceleration_norm >= accel_min_norm_ && acceleration_norm <= accel_max_norm_) {
      const auto accel_roll =
          std::atan2(linear_acceleration[1], linear_acceleration[2]);
      const auto accel_pitch = std::atan2(
          -linear_acceleration[0],
          std::sqrt(linear_acceleration[1] * linear_acceleration[1] +
                    linear_acceleration[2] * linear_acceleration[2]));
      const auto weight = clamp(accel_attitude_weight_, 0.0, 1.0);
      euler[0] = wrapAngle(blend(euler[0], accel_roll, weight));
      euler[1] = wrapAngle(blend(euler[1], accel_pitch, weight));
    }

    orientation_world_body_ = quatFromEuler(euler[0], euler[1], euler[2]);
    last_imu_time_ = stamp;
    last_state_time_ = stamp;
  }

  void pressureCallback(const sensor_msgs::msg::FluidPressure::SharedPtr message) {
    if (message->fluid_pressure <= 1.0) {
      return;
    }

    const auto stamp = resolveStamp(message->header.stamp);
    std::lock_guard<std::mutex> lock(mutex_);

    if (!baro_reference_pressure_.has_value()) {
      baro_reference_pressure_ = message->fluid_pressure;
      filtered_altitude_ = 0.0;
      previous_baro_altitude_ = 0.0;
      previous_baro_time_ = stamp;
      position_world_[2] = 0.0;
      velocity_world_[2] = 0.0;
      last_state_time_ = stamp;
      return;
    }

    const auto relative_altitude =
        44330.0 * (1.0 - std::pow(message->fluid_pressure / *baro_reference_pressure_, 0.190295));
    if (!filtered_altitude_.has_value()) {
      filtered_altitude_ = relative_altitude;
    } else {
      filtered_altitude_ =
          blend(*filtered_altitude_, relative_altitude, baro_altitude_weight_);
    }

    position_world_[2] = *filtered_altitude_;

    if (previous_baro_time_.has_value() && previous_baro_altitude_.has_value()) {
      const auto dt = (stamp - *previous_baro_time_).seconds();
      if (dt > 1e-3) {
        const auto measured_vertical_velocity =
            (*filtered_altitude_ - *previous_baro_altitude_) / dt;
        velocity_world_[2] =
            blend(velocity_world_[2], measured_vertical_velocity, baro_velocity_weight_);
      }
    }

    previous_baro_altitude_ = *filtered_altitude_;
    previous_baro_time_ = stamp;
    last_state_time_ = stamp;
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr message) {
    cv::Mat gray_image;
    if (!toGrayImage(*message, gray_image)) {
      return;
    }

    const auto stamp = resolveStamp(message->header.stamp);
    std::lock_guard<std::mutex> lock(mutex_);
    if (!camera_info_received_ || !imu_initialized_ || !filtered_altitude_.has_value()) {
      return;
    }

    if (!have_previous_frame_ || previous_gray_.empty() ||
        previous_gray_.size() != gray_image.size()) {
      resetVisualTracking(gray_image, stamp);
      return;
    }

    const auto frame_dt = (stamp - *previous_image_time_).seconds();
    if (frame_dt <= 1e-3) {
      resetVisualTracking(gray_image, stamp);
      return;
    }

    const auto euler = quatToEuler(orientation_world_body_);
    if (*filtered_altitude_ < min_flow_altitude_m_ ||
        *filtered_altitude_ > max_flow_altitude_m_ ||
        std::hypot(euler[0], euler[1]) > max_tilt_for_flow_rad_ ||
        std::abs(velocity_world_[2]) > max_vertical_speed_for_flow_mps_) {
      resetVisualTracking(gray_image, stamp);
      return;
    }

    if (static_cast<int>(previous_points_.size()) < min_features_) {
      previous_points_ = detectFeatures(previous_gray_);
    }
    if (static_cast<int>(previous_points_.size()) < min_tracked_features_) {
      resetVisualTracking(gray_image, stamp);
      return;
    }

    std::vector<cv::Point2f> tracked_points;
    std::vector<unsigned char> status;
    std::vector<float> errors;
    cv::calcOpticalFlowPyrLK(
        previous_gray_, gray_image, previous_points_, tracked_points, status, errors,
        cv::Size(lk_window_size_px_, lk_window_size_px_), lk_max_level_);

    std::vector<cv::Point2f> previous_valid_points;
    std::vector<cv::Point2f> current_valid_points;
    previous_valid_points.reserve(previous_points_.size());
    current_valid_points.reserve(previous_points_.size());

    for (std::size_t index = 0; index < previous_points_.size(); ++index) {
      if (index >= status.size() || status[index] == 0U) {
        continue;
      }

      const auto & current_point = tracked_points[index];
      if (current_point.x < 2.0F || current_point.y < 2.0F ||
          current_point.x >= static_cast<float>(gray_image.cols - 2) ||
          current_point.y >= static_cast<float>(gray_image.rows - 2)) {
        continue;
      }

      previous_valid_points.push_back(previous_points_[index]);
      current_valid_points.push_back(current_point);
    }

    if (static_cast<int>(current_valid_points.size()) < min_tracked_features_) {
      warnNoFlowYet();
      resetVisualTracking(gray_image, stamp);
      return;
    }

    cv::Mat inlier_mask;
    const auto affine_transform =
        cv::estimateAffinePartial2D(previous_valid_points, current_valid_points,
                                    inlier_mask, cv::RANSAC,
                                    affine_ransac_threshold_px_);
    if (affine_transform.empty() || inlier_mask.empty()) {
      warnNoFlowYet();
      resetVisualTracking(gray_image, stamp);
      return;
    }

    const auto range_to_ground = opticalRangeToGround();
    if (range_to_ground <= min_flow_altitude_m_) {
      resetVisualTracking(gray_image, stamp);
      return;
    }

    const auto angular_velocity_optical =
        rotateVectorByQuat(angular_velocity_body_, quatConjugate(body_from_optical_));
    std::vector<double> velocity_x_samples;
    std::vector<double> velocity_y_samples;
    velocity_x_samples.reserve(previous_valid_points.size());
    velocity_y_samples.reserve(previous_valid_points.size());

    auto current_body_velocity =
        rotateVectorByQuat(velocity_world_, quatConjugate(orientation_world_body_));
    current_body_velocity[0] = 0.0;
    current_body_velocity[1] = 0.0;
    const auto current_optical_velocity =
        rotateVectorByQuat(current_body_velocity, quatConjugate(body_from_optical_));
    const auto vertical_optical_velocity = current_optical_velocity[2];

    auto inlier_count = 0;
    for (int index = 0; index < inlier_mask.rows; ++index) {
      if (inlier_mask.at<unsigned char>(index) == 0U) {
        continue;
      }

      ++inlier_count;
      const auto & previous_point =
          previous_valid_points[static_cast<std::size_t>(index)];
      const auto & current_point =
          current_valid_points[static_cast<std::size_t>(index)];

      const auto x = (static_cast<double>(previous_point.x) - cx_) / fx_;
      const auto y = (static_cast<double>(previous_point.y) - cy_) / fy_;
      const auto observed_u =
          (static_cast<double>(current_point.x) - previous_point.x) / frame_dt;
      const auto observed_v =
          (static_cast<double>(current_point.y) - previous_point.y) / frame_dt;

      const auto rotational_u =
          fx_ * (x * y * angular_velocity_optical[0] -
                 (1.0 + x * x) * angular_velocity_optical[1] +
                 y * angular_velocity_optical[2]);
      const auto rotational_v =
          fy_ * ((1.0 + y * y) * angular_velocity_optical[0] -
                 x * y * angular_velocity_optical[1] -
                 x * angular_velocity_optical[2]);

      auto residual_u = observed_u - rotational_u;
      auto residual_v = observed_v - rotational_v;
      residual_u -= fx_ * x * vertical_optical_velocity / range_to_ground;
      residual_v -= fy_ * y * vertical_optical_velocity / range_to_ground;

      const auto optical_velocity_x = residual_u * range_to_ground / fx_;
      const auto optical_velocity_y = residual_v * range_to_ground / fy_;
      if (!std::isfinite(optical_velocity_x) || !std::isfinite(optical_velocity_y)) {
        continue;
      }

      if (std::hypot(optical_velocity_x, optical_velocity_y) > max_planar_speed_mps_ * 1.5) {
        continue;
      }

      velocity_x_samples.push_back(optical_velocity_x);
      velocity_y_samples.push_back(optical_velocity_y);
    }

    if (inlier_count < min_tracked_features_ / 2 ||
        velocity_x_samples.size() < static_cast<std::size_t>(min_tracked_features_ / 2)) {
      warnNoFlowYet();
      resetVisualTracking(gray_image, stamp);
      return;
    }

    const auto optical_velocity =
        Vector3{median(velocity_x_samples), median(velocity_y_samples), 0.0};
    auto body_velocity_estimate =
        rotateVectorByQuat(optical_velocity, body_from_optical_);
    body_velocity_estimate[0] *= -1.0;
    body_velocity_estimate[1] *= -1.0;
    body_velocity_estimate[0] =
        clamp(body_velocity_estimate[0], -max_planar_speed_mps_, max_planar_speed_mps_);
    body_velocity_estimate[1] =
        clamp(body_velocity_estimate[1], -max_planar_speed_mps_, max_planar_speed_mps_);
    body_velocity_estimate[2] = 0.0;

    const auto world_velocity_estimate =
        rotateVectorByQuat(body_velocity_estimate, orientation_world_body_);
    velocity_world_[0] =
        blend(velocity_world_[0], world_velocity_estimate[0], flow_velocity_weight_);
    velocity_world_[1] =
        blend(velocity_world_[1], world_velocity_estimate[1], flow_velocity_weight_);

    Vector3 position_correction = previous_flow_position_world_;
    position_correction[0] += world_velocity_estimate[0] * frame_dt;
    position_correction[1] += world_velocity_estimate[1] * frame_dt;
    position_world_[0] =
        blend(position_world_[0], position_correction[0], flow_position_weight_);
    position_world_[1] =
        blend(position_world_[1], position_correction[1], flow_position_weight_);

    previous_gray_ = gray_image;
    previous_points_.clear();
    previous_points_.reserve(current_valid_points.size());
    for (int index = 0; index < inlier_mask.rows; ++index) {
      if (inlier_mask.at<unsigned char>(index) == 0U) {
        continue;
      }
      previous_points_.push_back(
          current_valid_points[static_cast<std::size_t>(index)]);
    }
    if (static_cast<int>(previous_points_.size()) < min_features_) {
      previous_points_ = detectFeatures(previous_gray_);
    }

    previous_image_time_ = stamp;
    previous_flow_position_world_ = position_world_;
    last_state_time_ = stamp;
    have_previous_frame_ = true;
  }

  void publishState() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!imu_initialized_) {
      return;
    }

    const auto stamp = toBuiltinTime(last_state_time_.value_or(now()));
    const auto orientation = orientation_world_body_;
    const auto body_velocity =
        rotateVectorByQuat(velocity_world_, quatConjugate(orientation));
    const auto euler = quatToEuler(orientation);
    const auto yaw_orientation = quatFromYaw(euler[2]);
    auto planar_velocity =
        rotateVectorByQuat(velocity_world_, quatConjugate(yaw_orientation));
    planar_velocity[2] = 0.0;

    publishFullOdometry(stamp, position_world_, orientation, body_velocity,
                        angular_velocity_body_);
    publishProjectedOdometry(stamp, position_world_, euler[2], planar_velocity,
                             angular_velocity_body_[2]);
    publishTransforms(stamp, position_world_, orientation, euler[2]);
  }

  void publishFullOdometry(const builtin_interfaces::msg::Time & stamp,
                           const Vector3 & position,
                           const Quaternion & orientation,
                           const Vector3 & body_velocity,
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
    odom.twist.twist.linear.x = body_velocity[0];
    odom.twist.twist.linear.y = body_velocity[1];
    odom.twist.twist.linear.z = body_velocity[2];
    odom.twist.twist.angular.x = angular_velocity[0];
    odom.twist.twist.angular.y = angular_velocity[1];
    odom.twist.twist.angular.z = angular_velocity[2];
    fillFullCovariance(odom);
    full_odom_publisher_->publish(odom);
  }

  void publishProjectedOdometry(const builtin_interfaces::msg::Time & stamp,
                                const Vector3 & position,
                                double yaw,
                                const Vector3 & planar_velocity,
                                double yaw_rate) {
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
    fillProjectedCovariance(odom);
    projected_odom_publisher_->publish(odom);
  }

  void publishTransforms(const builtin_interfaces::msg::Time & stamp,
                         const Vector3 & position,
                         const Quaternion & orientation,
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

  void fillFullCovariance(nav_msgs::msg::Odometry & odom) const {
    odom.pose.covariance[0] = 0.06;
    odom.pose.covariance[7] = 0.06;
    odom.pose.covariance[14] = 0.10;
    odom.pose.covariance[21] = 0.03;
    odom.pose.covariance[28] = 0.03;
    odom.pose.covariance[35] = 0.10;
    odom.twist.covariance[0] = 0.12;
    odom.twist.covariance[7] = 0.12;
    odom.twist.covariance[14] = 0.18;
    odom.twist.covariance[21] = 0.04;
    odom.twist.covariance[28] = 0.04;
    odom.twist.covariance[35] = 0.10;
  }

  void fillProjectedCovariance(nav_msgs::msg::Odometry & odom) const {
    odom.pose.covariance[0] = 0.05;
    odom.pose.covariance[7] = 0.05;
    odom.pose.covariance[14] = 9999.0;
    odom.pose.covariance[21] = 9999.0;
    odom.pose.covariance[28] = 9999.0;
    odom.pose.covariance[35] = 0.08;
    odom.twist.covariance[0] = 0.10;
    odom.twist.covariance[7] = 0.10;
    odom.twist.covariance[14] = 9999.0;
    odom.twist.covariance[21] = 9999.0;
    odom.twist.covariance[28] = 9999.0;
    odom.twist.covariance[35] = 0.08;
  }

  Quaternion orientationFromAcceleration(const Vector3 & acceleration) const {
    const auto roll = std::atan2(acceleration[1], acceleration[2]);
    const auto pitch = std::atan2(
        -acceleration[0],
        std::sqrt(acceleration[1] * acceleration[1] +
                  acceleration[2] * acceleration[2]));
    return quatFromEuler(roll, pitch, 0.0);
  }

  void warnNoFlowYet() {
    if (have_warned_no_flow_) {
      return;
    }

    RCLCPP_WARN(
        get_logger(),
        "Downward visual odometry does not have enough valid ground texture yet. "
        "Use the warehouse world or add floor texture for reliable flight odometry.");
    have_warned_no_flow_ = true;
  }

  double opticalRangeToGround() const {
    if (!filtered_altitude_.has_value() || *filtered_altitude_ <= 0.0) {
      return 0.0;
    }

    const auto optical_axis_body =
        rotateVectorByQuat(Vector3{0.0, 0.0, 1.0}, body_from_optical_);
    const auto optical_axis_world =
        rotateVectorByQuat(optical_axis_body, orientation_world_body_);
    if (optical_axis_world[2] >= -0.15) {
      return 0.0;
    }

    return *filtered_altitude_ / -optical_axis_world[2];
  }

  void resetVisualTracking(const cv::Mat & gray_image, const rclcpp::Time & stamp) {
    previous_gray_ = gray_image;
    previous_points_ = detectFeatures(previous_gray_);
    previous_image_time_ = stamp;
    previous_flow_position_world_ = position_world_;
    have_previous_frame_ = true;
  }

  std::vector<cv::Point2f> detectFeatures(const cv::Mat & gray_image) const {
    std::vector<cv::Point2f> features;
    cv::goodFeaturesToTrack(gray_image, features, max_features_, quality_level_,
                            min_feature_distance_px_);
    return features;
  }

  bool toGrayImage(const sensor_msgs::msg::Image & image_message, cv::Mat & gray_image) {
    const auto height = static_cast<int>(image_message.height);
    const auto width = static_cast<int>(image_message.width);
    if (height <= 0 || width <= 0) {
      return false;
    }

    if (image_message.encoding == sensor_msgs::image_encodings::MONO8) {
      gray_image =
          cv::Mat(height, width, CV_8UC1, const_cast<unsigned char *>(image_message.data.data()),
                  static_cast<std::size_t>(image_message.step))
              .clone();
      return true;
    }

    if (image_message.encoding == sensor_msgs::image_encodings::RGB8) {
      const auto rgb_image =
          cv::Mat(height, width, CV_8UC3, const_cast<unsigned char *>(image_message.data.data()),
                  static_cast<std::size_t>(image_message.step));
      cv::cvtColor(rgb_image, gray_image, cv::COLOR_RGB2GRAY);
      return true;
    }

    if (image_message.encoding == sensor_msgs::image_encodings::BGR8) {
      const auto bgr_image =
          cv::Mat(height, width, CV_8UC3, const_cast<unsigned char *>(image_message.data.data()),
                  static_cast<std::size_t>(image_message.step));
      cv::cvtColor(bgr_image, gray_image, cv::COLOR_BGR2GRAY);
      return true;
    }

    if (!have_warned_unknown_encoding_) {
      RCLCPP_WARN(
          get_logger(),
          "Unsupported camera encoding '%s'; visual odometry skipped for this frame.",
          image_message.encoding.c_str());
      have_warned_unknown_encoding_ = true;
    }
    return false;
  }

  rclcpp::Time resolveStamp(const builtin_interfaces::msg::Time & stamp_message) const {
    if (stamp_message.sec != 0 || stamp_message.nanosec != 0) {
      return rclcpp::Time(stamp_message);
    }

    return now();
  }

  std::mutex mutex_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr full_odom_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr projected_odom_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::string full_odom_topic_;
  std::string projected_odom_topic_;
  std::string world_frame_;
  std::string body_frame_;
  std::string projected_body_frame_;
  bool publish_tf_{true};

  double publish_rate_hz_{100.0};
  double max_imu_dt_{0.03};
  double accel_min_norm_{7.0};
  double accel_max_norm_{12.5};
  double accel_attitude_weight_{0.03};
  double baro_altitude_weight_{0.35};
  double baro_velocity_weight_{0.35};
  double flow_velocity_weight_{0.55};
  double flow_position_weight_{0.30};
  double horizontal_velocity_damping_hz_{0.20};
  double stale_flow_damping_hz_{2.5};
  double flow_timeout_sec_{0.25};
  double min_flow_altitude_m_{0.35};
  double max_flow_altitude_m_{4.0};
  double max_tilt_for_flow_rad_{0.45};
  double max_vertical_speed_for_flow_mps_{0.45};
  double max_planar_speed_mps_{3.0};
  int max_features_{260};
  int min_features_{110};
  int min_tracked_features_{60};
  double quality_level_{0.01};
  double min_feature_distance_px_{12.0};
  int lk_window_size_px_{21};
  int lk_max_level_{3};
  double affine_ransac_threshold_px_{2.5};
  double camera_mount_roll_rad_{0.0};
  double camera_mount_pitch_rad_{kPi / 2.0};
  double camera_mount_yaw_rad_{0.0};
  double camera_optical_roll_rad_{-kPi / 2.0};
  double camera_optical_pitch_rad_{0.0};
  double camera_optical_yaw_rad_{-kPi / 2.0};
  Quaternion body_from_optical_{0.0, 0.0, 0.0, 1.0};

  bool camera_info_received_{false};
  bool imu_initialized_{false};
  bool have_previous_frame_{false};
  bool have_warned_unknown_encoding_{false};
  bool have_warned_no_flow_{false};

  double fx_{0.0};
  double fy_{0.0};
  double cx_{0.0};
  double cy_{0.0};

  cv::Mat previous_gray_;
  std::vector<cv::Point2f> previous_points_;

  std::optional<rclcpp::Time> previous_image_time_;
  std::optional<rclcpp::Time> last_imu_time_;
  std::optional<rclcpp::Time> last_state_time_;
  std::optional<rclcpp::Time> previous_baro_time_;
  std::optional<double> baro_reference_pressure_;
  std::optional<double> filtered_altitude_;
  std::optional<double> previous_baro_altitude_;

  Vector3 position_world_{0.0, 0.0, 0.0};
  Vector3 velocity_world_{0.0, 0.0, 0.0};
  Quaternion orientation_world_body_{0.0, 0.0, 0.0, 1.0};
  Vector3 angular_velocity_body_{0.0, 0.0, 0.0};
  Vector3 previous_flow_position_world_{0.0, 0.0, 0.0};
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisualInertialOdometry>());
  rclcpp::shutdown();
  return 0;
}
