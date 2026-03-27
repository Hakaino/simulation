#include <actuator_msgs/msg/actuators.hpp>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>

class MotorCommandGate : public rclcpp::Node {
public:
  MotorCommandGate() : Node("motor_command_gate") {
    const auto command_topic =
        declare_parameter<std::string>("command_topic", "/quadcopter/command/motor_speeds");
    const auto actuator_topic =
        declare_parameter<std::string>("actuator_topic", "/quadcopter/internal/actuators");
    max_motor_speed_ = declare_parameter<double>("max_motor_speed_rad_s", 900.0);
    const auto timeout_sec = declare_parameter<double>("command_timeout_sec", 0.2);
    const auto publish_rate_hz = declare_parameter<double>("publish_rate_hz", 50.0);

    command_timeout_ = rclcpp::Duration::from_seconds(timeout_sec);

    command_subscriber_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        command_topic, rclcpp::QoS(1),
        std::bind(&MotorCommandGate::commandCallback, this, std::placeholders::_1));
    actuator_publisher_ =
        create_publisher<actuator_msgs::msg::Actuators>(actuator_topic, rclcpp::QoS(1));
    arm_service_ = create_service<std_srvs::srv::SetBool>(
        "/quadcopter/arm",
        std::bind(&MotorCommandGate::handleArmRequest, this, std::placeholders::_1,
                  std::placeholders::_2, std::placeholders::_3));
    publish_timer_ = rclcpp::create_timer(
        this, get_clock(),
        rclcpp::Duration::from_seconds(1.0 / std::max(publish_rate_hz, 1.0)),
        std::bind(&MotorCommandGate::publishActuators, this));

    RCLCPP_INFO(get_logger(),
                "Motor command gate ready. Publish Float64MultiArray[4] rotor speeds to %s and "
                "arm via /quadcopter/arm.",
                command_topic.c_str());
  }

private:
  void handleArmRequest(const std::shared_ptr<rmw_request_id_t>,
                        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    armed_ = request->data;
    if (!armed_) {
      desired_speeds_.fill(0.0);
      has_last_valid_command_time_ = false;
    }

    response->success = true;
    response->message = armed_ ? "armed" : "disarmed";
    RCLCPP_INFO(get_logger(), "Vehicle %s.", response->message.c_str());
  }

  void commandCallback(const std_msgs::msg::Float64MultiArray::SharedPtr message) {
    if (message->data.size() != desired_speeds_.size()) {
      invalidateCommand("Expected exactly 4 motor speeds.");
      return;
    }

    std::array<double, 4> sanitized{};
    for (std::size_t index = 0; index < message->data.size(); ++index) {
      const auto value = message->data[index];
      if (!std::isfinite(value)) {
        invalidateCommand("Received a non-finite motor speed.");
        return;
      }
      if (value < 0.0) {
        invalidateCommand("Received a negative motor speed.");
        return;
      }
      sanitized[index] = std::min(value, max_motor_speed_);
    }

    desired_speeds_ = sanitized;
    last_valid_command_time_ = now();
    has_last_valid_command_time_ = true;
  }

  void invalidateCommand(const std::string & reason) {
    desired_speeds_.fill(0.0);
    has_last_valid_command_time_ = false;
    RCLCPP_WARN(get_logger(), "%s", reason.c_str());
  }

  void publishActuators() {
    std::array<double, 4> output{};
    if (armed_ && has_last_valid_command_time_) {
      if ((now() - last_valid_command_time_) <= command_timeout_) {
        output = desired_speeds_;
      }
    }

    actuator_msgs::msg::Actuators message;
    message.header.stamp = now();
    message.normalized.clear();
    message.position.clear();
    message.velocity.assign(output.begin(), output.end());
    actuator_publisher_->publish(message);
  }

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_subscriber_;
  rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr actuator_publisher_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arm_service_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  double max_motor_speed_{900.0};
  rclcpp::Duration command_timeout_{0, 0};
  bool armed_{false};
  bool has_last_valid_command_time_{false};
  rclcpp::Time last_valid_command_time_{0, 0, RCL_ROS_TIME};
  std::array<double, 4> desired_speeds_{};
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorCommandGate>());
  rclcpp::shutdown();
  return 0;
}
