#include <algorithm>
#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>

class TakeoffDemo : public rclcpp::Node {
public:
  TakeoffDemo() : Node("takeoff_demo") {
    const auto arm_service =
        declare_parameter<std::string>("arm_service", "/quadcopter/arm");
    const auto motor_command_topic =
        declare_parameter<std::string>("motor_command_topic",
                                       "/quadcopter/command/motor_speeds");
    const auto publish_rate_hz = declare_parameter<double>("publish_rate_hz", 50.0);

    spinup_speed_ = declare_parameter<double>("spinup_speed_rad_s", 550.0);
    takeoff_speed_ = declare_parameter<double>("takeoff_speed_rad_s", 575.0);
    spinup_duration_ = declare_parameter<double>("spinup_duration_sec", 1.0);
    hold_duration_ = declare_parameter<double>("hold_duration_sec", 1.0);
    ramp_down_duration_ = declare_parameter<double>("ramp_down_duration_sec", 1.0);

    publisher_ =
        create_publisher<std_msgs::msg::Float64MultiArray>(motor_command_topic, 10);
    arm_client_ = create_client<std_srvs::srv::SetBool>(arm_service);
    timer_ = rclcpp::create_timer(
        this, get_clock(),
        rclcpp::Duration::from_seconds(1.0 / std::max(publish_rate_hz, 1.0)),
        std::bind(&TakeoffDemo::tick, this));
  }

private:
  enum class DemoState {
    WaitingForArmService,
    Arming,
    Spinup,
    Hold,
    RampDown,
    Complete,
  };

  void tick() {
    const auto now_time = now();
    if (!has_state_start_time_) {
      state_start_time_ = now_time;
      has_state_start_time_ = true;
    }

    if (state_ == DemoState::WaitingForArmService) {
      if (arm_client_->service_is_ready()) {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;
        arm_request_pending_ = true;
        arm_response_received_ = false;
        arm_client_->async_send_request(
            request,
            [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
              arm_request_pending_ = false;
              arm_response_received_ = true;
              try {
                const auto response = future.get();
                arm_response_success_ = response && response->success;
              } catch (const std::exception &) {
                arm_response_success_ = false;
              }
            });
        state_ = DemoState::Arming;
        state_start_time_ = now_time;
        RCLCPP_INFO(get_logger(), "Arming quadcopter for takeoff demo.");
      }
      return;
    }

    if (state_ == DemoState::Arming) {
      if (arm_response_received_) {
        if (!arm_response_success_) {
          RCLCPP_ERROR(get_logger(), "Failed to arm quadcopter for takeoff demo.");
          state_ = DemoState::Complete;
          return;
        }

        state_ = DemoState::Spinup;
        state_start_time_ = now_time;
        RCLCPP_INFO(get_logger(), "Starting motor ramp.");
      }
      publishSpeeds({0.0, 0.0, 0.0, 0.0});
      return;
    }

    const auto elapsed = (now_time - state_start_time_).seconds();
    if (state_ == DemoState::Spinup) {
      const auto progress = std::clamp(elapsed / std::max(spinup_duration_, 1e-6), 0.0, 1.0);
      const auto speed = spinup_speed_ + (takeoff_speed_ - spinup_speed_) * progress;
      publishUniformSpeed(speed);
      if (progress >= 1.0) {
        state_ = DemoState::Hold;
        state_start_time_ = now_time;
        RCLCPP_INFO(get_logger(), "Holding takeoff command.");
      }
      return;
    }

    if (state_ == DemoState::Hold) {
      publishUniformSpeed(takeoff_speed_);
      if (elapsed >= hold_duration_) {
        state_ = DemoState::RampDown;
        state_start_time_ = now_time;
        RCLCPP_INFO(get_logger(), "Ramping motors back down.");
      }
      return;
    }

    if (state_ == DemoState::RampDown) {
      const auto progress =
          std::clamp(elapsed / std::max(ramp_down_duration_, 1e-6), 0.0, 1.0);
      const auto speed = takeoff_speed_ * (1.0 - progress);
      publishUniformSpeed(speed);
      if (progress >= 1.0) {
        state_ = DemoState::Complete;
        state_start_time_ = now_time;
        disarm();
        RCLCPP_INFO(get_logger(), "Takeoff demo complete.");
      }
      return;
    }

    publishUniformSpeed(0.0);
  }

  void publishUniformSpeed(double speed) {
    publishSpeeds({speed, speed, speed, speed});
  }

  void publishSpeeds(const std::array<double, 4> & speeds) {
    std_msgs::msg::Float64MultiArray message;
    message.data.assign(speeds.begin(), speeds.end());
    publisher_->publish(message);
  }

  void disarm() {
    if (!arm_client_->service_is_ready()) {
      return;
    }

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = false;
    arm_client_->async_send_request(request);
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr arm_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  DemoState state_{DemoState::WaitingForArmService};
  bool has_state_start_time_{false};
  rclcpp::Time state_start_time_{0, 0, RCL_ROS_TIME};
  bool arm_request_pending_{false};
  bool arm_response_received_{false};
  bool arm_response_success_{false};

  double spinup_speed_{550.0};
  double takeoff_speed_{575.0};
  double spinup_duration_{1.0};
  double hold_duration_{1.0};
  double ramp_down_duration_{1.0};
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TakeoffDemo>());
  rclcpp::shutdown();
  return 0;
}
