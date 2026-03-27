#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace {

using Vector3 = std::array<double, 3>;
using Quaternion = std::array<double, 4>;
constexpr double kPi = 3.14159265358979323846;

double clamp(double value, double lower, double upper) {
  return std::clamp(value, lower, upper);
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

} // namespace

class FlightController : public rclcpp::Node {
public:
  FlightController() : Node("flight_controller") {
    const auto cmd_vel_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/odom");
    const auto imu_topic = declare_parameter<std::string>("imu_topic", "/imu/data");
    const auto motor_command_topic =
        declare_parameter<std::string>("motor_command_topic",
                                       "/quadcopter/command/motor_speeds");
    const auto arm_service =
        declare_parameter<std::string>("arm_service", "/quadcopter/arm");
    const auto control_rate_hz = declare_parameter<double>("control_rate_hz", 100.0);
    const auto timeout_sec = declare_parameter<double>("command_timeout_sec", 0.5);

    auto_arm_ = declare_parameter<bool>("auto_arm", true);
    takeoff_altitude_ = declare_parameter<double>("takeoff_altitude_m", 1.5);
    max_altitude_ = declare_parameter<double>("max_altitude_m", 3.0);
    max_horizontal_speed_ = declare_parameter<double>("max_horizontal_speed_mps", 1.5);
    max_vertical_speed_ = declare_parameter<double>("max_vertical_speed_mps", 0.5);
    max_yaw_rate_ = declare_parameter<double>("max_yaw_rate_rad_s", 1.0);
    max_tilt_ = declare_parameter<double>("max_tilt_rad", 0.35);
    mass_ = declare_parameter<double>("mass_kg", 1.08);
    gravity_ = declare_parameter<double>("gravity_mps2", 9.81);
    arm_length_ = declare_parameter<double>("arm_length_m", 0.18);
    motor_constant_ = declare_parameter<double>("motor_constant", 8.54858e-06);
    moment_constant_ = declare_parameter<double>("moment_constant", 0.016);
    max_motor_speed_ = declare_parameter<double>("max_motor_speed_rad_s", 900.0);
    altitude_kp_ = declare_parameter<double>("altitude_kp", 3.8);
    altitude_ki_ = declare_parameter<double>("altitude_ki", 0.8);
    altitude_kd_ = declare_parameter<double>("altitude_kd", 2.4);
    velocity_to_pitch_gain_ = declare_parameter<double>("velocity_to_pitch_gain", 0.22);
    velocity_to_roll_gain_ = declare_parameter<double>("velocity_to_roll_gain", 0.22);
    attitude_kp_ = declare_parameter<double>("attitude_kp", 7.5);
    attitude_kd_ = declare_parameter<double>("attitude_kd", 2.8);
    yaw_rate_kp_ = declare_parameter<double>("yaw_rate_kp", 0.25);

    command_timeout_ = rclcpp::Duration::from_seconds(timeout_sec);

    command_publisher_ =
        create_publisher<std_msgs::msg::Float64MultiArray>(motor_command_topic, rclcpp::QoS(1));
    cmd_vel_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic, rclcpp::QoS(5),
        std::bind(&FlightController::cmdVelCallback, this, std::placeholders::_1));
    odom_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::QoS(1),
        std::bind(&FlightController::odomCallback, this, std::placeholders::_1));
    imu_subscriber_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, rclcpp::QoS(1),
        std::bind(&FlightController::imuCallback, this, std::placeholders::_1));
    arm_client_ = create_client<std_srvs::srv::SetBool>(arm_service);
    arm_retry_timer_ = rclcpp::create_timer(
        this, get_clock(), rclcpp::Duration::from_seconds(1.0),
        std::bind(&FlightController::ensureArmed, this));
    control_timer_ = rclcpp::create_timer(
        this, get_clock(),
        rclcpp::Duration::from_seconds(1.0 / std::max(control_rate_hz, 1.0)),
        std::bind(&FlightController::controlLoop, this));

    RCLCPP_INFO(get_logger(),
                "Flight controller ready. It will hold altitude and accept planar /cmd_vel "
                "commands for navigation.");
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr message) {
    commanded_velocity_x_ =
        clamp(message->linear.x, -max_horizontal_speed_, max_horizontal_speed_);
    commanded_velocity_y_ =
        clamp(message->linear.y, -max_horizontal_speed_, max_horizontal_speed_);
    commanded_velocity_z_ =
        clamp(message->linear.z, -max_vertical_speed_, max_vertical_speed_);
    commanded_yaw_rate_ = clamp(message->angular.z, -max_yaw_rate_, max_yaw_rate_);
    last_command_time_ = now();
    has_last_command_time_ = true;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr message) {
    current_altitude_ = message->pose.pose.position.z;
    orientation_ = Quaternion{message->pose.pose.orientation.x, message->pose.pose.orientation.y,
                              message->pose.pose.orientation.z, message->pose.pose.orientation.w};
    body_velocity_ = Vector3{message->twist.twist.linear.x, message->twist.twist.linear.y,
                             message->twist.twist.linear.z};

    const auto world_velocity = rotateVectorByQuat(body_velocity_, *orientation_);
    world_vertical_velocity_ = world_velocity[2];

    if (!target_altitude_.has_value()) {
      target_altitude_ = std::max(*current_altitude_, takeoff_altitude_);
    }
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr message) {
    angular_velocity_ = Vector3{message->angular_velocity.x, message->angular_velocity.y,
                                message->angular_velocity.z};
  }

  void ensureArmed() {
    if (!auto_arm_ || arm_acknowledged_ || arm_request_pending_ || !arm_client_->service_is_ready()) {
      return;
    }

    const auto now_time = now();
    if (has_last_arm_request_time_ &&
        (now_time - last_arm_request_time_) < rclcpp::Duration::from_seconds(1.0)) {
      return;
    }

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;
    arm_request_pending_ = true;
    last_arm_request_time_ = now_time;
    has_last_arm_request_time_ = true;

    arm_client_->async_send_request(
        request,
        [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
          arm_request_pending_ = false;
          try {
            const auto response = future.get();
            if (response && response->success) {
              if (!arm_acknowledged_) {
                RCLCPP_INFO(get_logger(), "Flight controller armed the quadcopter.");
              }
              arm_acknowledged_ = true;
              return;
            }

            if (response && !response->message.empty()) {
              RCLCPP_WARN(get_logger(), "Arm request rejected: %s",
                          response->message.c_str());
            }
          } catch (const std::exception & ex) {
            RCLCPP_WARN(get_logger(), "Arm request failed: %s", ex.what());
          }
        });
  }

  bool commandIsStale() const {
    if (!has_last_command_time_) {
      return true;
    }
    return (now() - last_command_time_) > command_timeout_;
  }

  void controlLoop() {
    if (!orientation_.has_value() || !current_altitude_.has_value() ||
        !target_altitude_.has_value()) {
      return;
    }

    const auto now_time = now();
    double dt = 0.01;
    if (has_last_control_time_) {
      dt = std::max((now_time - last_control_time_).seconds(), 1e-3);
    }
    last_control_time_ = now_time;
    has_last_control_time_ = true;

    double commanded_velocity_x = 0.0;
    double commanded_velocity_y = 0.0;
    double commanded_velocity_z = 0.0;
    double commanded_yaw_rate = 0.0;
    if (!commandIsStale()) {
      commanded_velocity_x = commanded_velocity_x_;
      commanded_velocity_y = commanded_velocity_y_;
      commanded_velocity_z = commanded_velocity_z_;
      commanded_yaw_rate = commanded_yaw_rate_;
    }

    target_altitude_ = clamp(*target_altitude_ + commanded_velocity_z * dt, 0.3, max_altitude_);

    const auto euler = quatToEuler(*orientation_);
    const auto body_z_axis_in_world =
        rotateVectorByQuat(Vector3{0.0, 0.0, 1.0}, *orientation_);

    const auto altitude_error = *target_altitude_ - *current_altitude_;
    altitude_integral_ = clamp(altitude_integral_ + altitude_error * dt, -1.5, 1.5);
    auto desired_vertical_acceleration =
        altitude_kp_ * altitude_error + altitude_ki_ * altitude_integral_ -
        altitude_kd_ * world_vertical_velocity_;
    desired_vertical_acceleration = clamp(desired_vertical_acceleration, -4.0, 4.0);

    auto thrust = mass_ * (gravity_ + desired_vertical_acceleration);
    thrust /= std::max(body_z_axis_in_world[2], 0.35);
    thrust = std::max(0.0, thrust);

    const auto desired_pitch =
        clamp(velocity_to_pitch_gain_ * (commanded_velocity_x - body_velocity_[0]),
              -max_tilt_, max_tilt_);
    const auto desired_roll =
        clamp(-velocity_to_roll_gain_ * (commanded_velocity_y - body_velocity_[1]),
              -max_tilt_, max_tilt_);

    const auto roll_torque =
        attitude_kp_ * (desired_roll - euler[0]) - attitude_kd_ * angular_velocity_[0];
    const auto pitch_torque =
        attitude_kp_ * (desired_pitch - euler[1]) - attitude_kd_ * angular_velocity_[1];
    const auto yaw_torque =
        yaw_rate_kp_ * (commanded_yaw_rate - angular_velocity_[2]);

    const auto collective = thrust / (4.0 * motor_constant_);
    const auto roll_term = roll_torque / (4.0 * motor_constant_ * arm_length_);
    const auto pitch_term = pitch_torque / (4.0 * motor_constant_ * arm_length_);
    const auto yaw_term = yaw_torque / (4.0 * motor_constant_ * moment_constant_);

    const std::array<double, 4> squared_speeds = {
        collective + roll_term - pitch_term + yaw_term,
        collective - roll_term - pitch_term - yaw_term,
        collective - roll_term + pitch_term + yaw_term,
        collective + roll_term + pitch_term - yaw_term,
    };

    const auto max_squared_speed = max_motor_speed_ * max_motor_speed_;
    std::vector<double> motor_speeds;
    motor_speeds.reserve(squared_speeds.size());
    for (const auto squared_speed : squared_speeds) {
      motor_speeds.push_back(std::sqrt(clamp(squared_speed, 0.0, max_squared_speed)));
    }

    std_msgs::msg::Float64MultiArray message;
    message.data = std::move(motor_speeds);
    command_publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr arm_client_;
  rclcpp::TimerBase::SharedPtr arm_retry_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  bool auto_arm_{true};
  double takeoff_altitude_{1.5};
  double max_altitude_{3.0};
  double max_horizontal_speed_{1.5};
  double max_vertical_speed_{0.5};
  double max_yaw_rate_{1.0};
  double max_tilt_{0.35};
  double mass_{1.08};
  double gravity_{9.81};
  double arm_length_{0.18};
  double motor_constant_{8.54858e-06};
  double moment_constant_{0.016};
  double max_motor_speed_{900.0};
  double altitude_kp_{3.8};
  double altitude_ki_{0.8};
  double altitude_kd_{2.4};
  double velocity_to_pitch_gain_{0.22};
  double velocity_to_roll_gain_{0.22};
  double attitude_kp_{7.5};
  double attitude_kd_{2.8};
  double yaw_rate_kp_{0.25};

  rclcpp::Duration command_timeout_{0, 0};
  bool has_last_command_time_{false};
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  bool has_last_control_time_{false};
  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};
  bool has_last_arm_request_time_{false};
  rclcpp::Time last_arm_request_time_{0, 0, RCL_ROS_TIME};
  bool arm_request_pending_{false};
  bool arm_acknowledged_{false};

  double commanded_velocity_x_{0.0};
  double commanded_velocity_y_{0.0};
  double commanded_velocity_z_{0.0};
  double commanded_yaw_rate_{0.0};
  double altitude_integral_{0.0};

  std::optional<double> target_altitude_;
  std::optional<double> current_altitude_;
  std::optional<Quaternion> orientation_;
  Vector3 body_velocity_{0.0, 0.0, 0.0};
  double world_vertical_velocity_{0.0};
  Vector3 angular_velocity_{0.0, 0.0, 0.0};
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlightController>());
  rclcpp::shutdown();
  return 0;
}
