#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>

namespace {

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

std::array<double, 4> quatFromYaw(double yaw) {
  return {0.0, 0.0, std::sin(yaw * 0.5), std::cos(yaw * 0.5)};
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & quaternion) {
  const auto siny_cosp =
      2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const auto cosy_cosp =
      1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace

class Px4OffboardManager : public rclcpp::Node {
public:
  Px4OffboardManager() : Node("px4_offboard_manager") {
    const auto cmd_vel_topic =
        declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    const auto state_topic =
        declare_parameter<std::string>("state_topic", "/mavros/state");
    const auto odom_topic = declare_parameter<std::string>(
        "odom_topic", "/mavros/local_position/odom");
    const auto setpoint_topic = declare_parameter<std::string>(
        "setpoint_topic", "/mavros/setpoint_position/local");
    const auto set_mode_service = declare_parameter<std::string>(
        "set_mode_service", "/mavros/set_mode");
    const auto arm_service = declare_parameter<std::string>(
        "arm_service", "/mavros/cmd/arming");

    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 30.0);
    command_timeout_sec_ =
        declare_parameter<double>("command_timeout_sec", 0.5);
    takeoff_altitude_m_ =
        declare_parameter<double>("takeoff_altitude_m", 1.5);
    max_horizontal_speed_mps_ =
        declare_parameter<double>("max_horizontal_speed_mps", 1.5);
    max_yaw_rate_rad_s_ =
        declare_parameter<double>("max_yaw_rate_rad_s", 1.0);
    max_setpoint_lead_m_ =
        declare_parameter<double>("max_setpoint_lead_m", 2.0);
    planar_motion_altitude_margin_m_ =
        declare_parameter<double>("planar_motion_altitude_margin_m", 0.2);
    offboard_setpoint_warmup_count_ =
        declare_parameter<int>("offboard_setpoint_warmup_count", 20);

    state_subscription_ = create_subscription<mavros_msgs::msg::State>(
        state_topic, rclcpp::QoS(10),
        std::bind(&Px4OffboardManager::stateCallback, this, std::placeholders::_1));
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::QoS(10),
        std::bind(&Px4OffboardManager::odomCallback, this, std::placeholders::_1));
    cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic, rclcpp::QoS(5),
        std::bind(&Px4OffboardManager::cmdVelCallback, this, std::placeholders::_1));

    setpoint_publisher_ =
        create_publisher<geometry_msgs::msg::PoseStamped>(setpoint_topic, rclcpp::QoS(10));
    set_mode_client_ = create_client<mavros_msgs::srv::SetMode>(set_mode_service);
    arm_client_ = create_client<mavros_msgs::srv::CommandBool>(arm_service);

    control_timer_ = rclcpp::create_timer(
        this, get_clock(),
        rclcpp::Duration::from_seconds(1.0 / std::max(1.0, publish_rate_hz_)),
        std::bind(&Px4OffboardManager::controlLoop, this));

    RCLCPP_INFO(
        get_logger(),
        "PX4 offboard manager ready. It will stream position setpoints from /cmd_vel "
        "so PX4 handles stabilization, takeoff, and hold.");
  }

private:
  struct Pose2p5D {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double yaw{0.0};
  };

  void stateCallback(const mavros_msgs::msg::State::SharedPtr message) {
    const auto was_connected = state_connected_;
    const auto was_armed = state_armed_;
    const auto previous_mode = state_mode_;

    state_connected_ = message->connected;
    state_armed_ = message->armed;
    state_mode_ = message->mode;

    if (!was_connected && state_connected_) {
      RCLCPP_INFO(get_logger(), "MAVROS connected to PX4.");
    }
    if (!was_armed && state_armed_) {
      RCLCPP_INFO(get_logger(), "PX4 is armed.");
    }
    if (previous_mode != "OFFBOARD" && state_mode_ == "OFFBOARD") {
      RCLCPP_INFO(get_logger(), "PX4 accepted OFFBOARD mode.");
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr message) {
    current_pose_ = Pose2p5D{
        message->pose.pose.position.x,
        message->pose.pose.position.y,
        message->pose.pose.position.z,
        yawFromQuaternion(message->pose.pose.orientation),
    };

    if (!target_initialized_) {
      target_x_ = current_pose_->x;
      target_y_ = current_pose_->y;
      target_z_ = std::max(current_pose_->z, takeoff_altitude_m_);
      target_yaw_ = current_pose_->yaw;
      target_initialized_ = true;

      RCLCPP_INFO(
          get_logger(),
          "Initialized PX4 local-position target at x=%.2f y=%.2f z=%.2f yaw=%.2f rad.",
          target_x_, target_y_, target_z_, target_yaw_);
    }
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr message) {
    command_velocity_x_ = clamp(
        message->linear.x, -max_horizontal_speed_mps_, max_horizontal_speed_mps_);
    command_velocity_y_ = clamp(
        message->linear.y, -max_horizontal_speed_mps_, max_horizontal_speed_mps_);
    command_yaw_rate_ = clamp(
        message->angular.z, -max_yaw_rate_rad_s_, max_yaw_rate_rad_s_);
    last_command_time_ = now();
    have_last_command_time_ = true;
  }

  void controlLoop() {
    if (!target_initialized_ || !current_pose_.has_value()) {
      return;
    }

    const auto now_time = now();
    double dt = 0.0;
    if (!last_control_time_.has_value()) {
      last_control_time_ = now_time;
    } else {
      dt = clamp((now_time - *last_control_time_).seconds(), 0.0, 0.2);
      last_control_time_ = now_time;
    }

    const auto command_is_fresh =
        have_last_command_time_ &&
        (now_time - last_command_time_).seconds() <= command_timeout_sec_;
    const auto planar_motion_enabled =
        current_pose_->z >= (target_z_ - planar_motion_altitude_margin_m_);

    if (planar_motion_enabled && !have_announced_planar_motion_) {
      RCLCPP_INFO(
          get_logger(),
          "PX4 reached takeoff altitude. /cmd_vel now moves the vehicle over the ground.");
      have_announced_planar_motion_ = true;
    }

    if (planar_motion_enabled && dt > 0.0) {
      const auto body_vx = command_is_fresh ? command_velocity_x_ : 0.0;
      const auto body_vy = command_is_fresh ? command_velocity_y_ : 0.0;
      const auto yaw_rate = command_is_fresh ? command_yaw_rate_ : 0.0;

      const auto yaw_cos = std::cos(current_pose_->yaw);
      const auto yaw_sin = std::sin(current_pose_->yaw);
      const auto world_vx = yaw_cos * body_vx - yaw_sin * body_vy;
      const auto world_vy = yaw_sin * body_vx + yaw_cos * body_vy;

      target_x_ += world_vx * dt;
      target_y_ += world_vy * dt;
      target_yaw_ = wrapAngle(target_yaw_ + yaw_rate * dt);

      clampTargetLead();
    }

    publishSetpoint(now_time);
    requestOffboardAndArm(now_time);
  }

  void clampTargetLead() {
    if (!current_pose_.has_value()) {
      return;
    }

    const auto dx = target_x_ - current_pose_->x;
    const auto dy = target_y_ - current_pose_->y;
    const auto planar_distance = std::hypot(dx, dy);
    if (planar_distance <= max_setpoint_lead_m_ || planar_distance <= 1e-6) {
      return;
    }

    const auto scale = max_setpoint_lead_m_ / planar_distance;
    target_x_ = current_pose_->x + dx * scale;
    target_y_ = current_pose_->y + dy * scale;
  }

  void publishSetpoint(const rclcpp::Time & stamp) {
    geometry_msgs::msg::PoseStamped setpoint;
    setpoint.header.stamp = stamp;
    setpoint.header.frame_id = "odom";
    setpoint.pose.position.x = target_x_;
    setpoint.pose.position.y = target_y_;
    setpoint.pose.position.z = target_z_;

    const auto yaw_quaternion = quatFromYaw(target_yaw_);
    setpoint.pose.orientation.x = yaw_quaternion[0];
    setpoint.pose.orientation.y = yaw_quaternion[1];
    setpoint.pose.orientation.z = yaw_quaternion[2];
    setpoint.pose.orientation.w = yaw_quaternion[3];

    setpoint_publisher_->publish(setpoint);
    ++setpoint_stream_count_;
  }

  void requestOffboardAndArm(const rclcpp::Time & stamp) {
    if (!state_connected_ || setpoint_stream_count_ < offboard_setpoint_warmup_count_) {
      return;
    }

    if (state_mode_ != "OFFBOARD") {
      if (set_mode_request_pending_ || !set_mode_client_->service_is_ready()) {
        return;
      }
      if (last_mode_request_time_.has_value() &&
          (stamp - *last_mode_request_time_).seconds() < 1.0) {
        return;
      }

      auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
      request->base_mode = 0;
      request->custom_mode = "OFFBOARD";

      set_mode_request_pending_ = true;
      last_mode_request_time_ = stamp;
      set_mode_client_->async_send_request(
          request,
          [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
            set_mode_request_pending_ = false;
            try {
              const auto response = future.get();
              if (response && response->mode_sent) {
                return;
              }
              RCLCPP_WARN(get_logger(), "PX4 OFFBOARD mode request was rejected.");
            } catch (const std::exception & error) {
              RCLCPP_WARN(
                  get_logger(), "PX4 OFFBOARD mode request failed: %s", error.what());
            }
          });
      return;
    }

    if (state_armed_ || arm_request_pending_ || !arm_client_->service_is_ready()) {
      return;
    }
    if (last_arm_request_time_.has_value() &&
        (stamp - *last_arm_request_time_).seconds() < 1.0) {
      return;
    }

    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = true;

    arm_request_pending_ = true;
    last_arm_request_time_ = stamp;
    arm_client_->async_send_request(
        request,
        [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
          arm_request_pending_ = false;
          try {
            const auto response = future.get();
            if (response && response->success) {
              return;
            }
            RCLCPP_WARN(get_logger(), "PX4 arming request was rejected.");
          } catch (const std::exception & error) {
            RCLCPP_WARN(get_logger(), "PX4 arming request failed: %s", error.what());
          }
        });
  }

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_publisher_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  double publish_rate_hz_{30.0};
  double command_timeout_sec_{0.5};
  double takeoff_altitude_m_{1.5};
  double max_horizontal_speed_mps_{1.5};
  double max_yaw_rate_rad_s_{1.0};
  double max_setpoint_lead_m_{2.0};
  double planar_motion_altitude_margin_m_{0.2};
  int offboard_setpoint_warmup_count_{20};

  bool state_connected_{false};
  bool state_armed_{false};
  std::string state_mode_;
  std::optional<Pose2p5D> current_pose_;
  std::optional<rclcpp::Time> last_control_time_;
  std::optional<rclcpp::Time> last_mode_request_time_;
  std::optional<rclcpp::Time> last_arm_request_time_;

  double target_x_{0.0};
  double target_y_{0.0};
  double target_z_{0.0};
  double target_yaw_{0.0};
  bool target_initialized_{false};

  double command_velocity_x_{0.0};
  double command_velocity_y_{0.0};
  double command_yaw_rate_{0.0};
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  bool have_last_command_time_{false};

  int setpoint_stream_count_{0};
  bool set_mode_request_pending_{false};
  bool arm_request_pending_{false};
  bool have_announced_planar_motion_{false};
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4OffboardManager>());
  rclcpp::shutdown();
  return 0;
}
