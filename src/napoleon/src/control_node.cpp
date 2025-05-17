#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"

using namespace std::chrono_literals;

class OffboardNode : public rclcpp::Node
{
public:
  OffboardNode()
  : Node("offboard_cpp")
  {
    // Subscribers
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "mavros/state", 10,
      std::bind(&OffboardNode::state_cb, this, std::placeholders::_1));

    // Publishers
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "mavros/setpoint_position/local", 10);

    // Clients
    arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>(
      "mavros/cmd/arming");
    mode_client_ = this->create_client<mavros_msgs::srv::SetMode>(
      "mavros/set_mode");

    // Timer at 20 Hz to send setpoints
    timer_ = this->create_wall_timer(50ms,
      std::bind(&OffboardNode::timer_cb, this));
  }

private:
  void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
  {
    current_state_ = *msg;
  }

  void timer_cb()
  {
    // Wait until connected
    if (!current_state_.connected) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Waiting for FCU connection...");
      return;
    }

    // Send a few setpoints before switching modes
    if (setpoint_count_ < 100) {
      publish_pose();
      setpoint_count_++;
      return;
    }

    // Once we're streaming, try to switch to OFFBOARD + arm once
    if (!offboard_set_) {
      auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
      req->custom_mode = "OFFBOARD";
      mode_client_->async_send_request(req);
      offboard_set_ = true;
      RCLCPP_INFO(this->get_logger(), "Offboard mode requested");
    } else if (!armed_) {
      auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
      req->value = true;
      arming_client_->async_send_request(req);
      armed_ = true;
      RCLCPP_INFO(this->get_logger(), "Arming requested");
    }

    // Publish the hover setpoint
    publish_pose();
  }

  void publish_pose()
  {
    geometry_msgs::msg::PoseStamped p;
    p.header.stamp = this->now();
    p.pose.position.x = 0.0;
    p.pose.position.y = 0.0;
    p.pose.position.z = 2.0;
    pose_pub_->publish(p);
  }

  // State
  mavros_msgs::msg::State current_state_;
  bool offboard_set_ = false;
  bool armed_        = false;
  int  setpoint_count_ = 0;

  // ROS interfaces
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr   mode_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OffboardNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
