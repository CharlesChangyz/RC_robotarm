#ifndef DMBOT_SERIAL_DEBUGGER_NODE_HPP_
#define DMBOT_SERIAL_DEBUGGER_NODE_HPP_

#include <array>
#include <atomic>
#include <thread>
#include <utility>
#include <vector>

#include "arm_msgs/msg/robot_command.hpp"
#include "rclcpp/rclcpp.hpp"

class RobotCommandPublisher : public rclcpp::Node
{
public:
  RobotCommandPublisher();
  ~RobotCommandPublisher() override;

private:
  std::vector<arm_msgs::msg::RobotCommand> create_commands();
  static float clamp(float value, float min_value, float max_value);
  void apply_q_limits_to_commands();
  void timer_callback();
  void keyboard_listener();

  rclcpp::Publisher<arm_msgs::msg::RobotCommand>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::thread keyboard_thread_;
  std::atomic<bool> running_{false};

  int current_command_idx_{0};
  float swing_phase_{0.0F};

  std::array<std::pair<float, float>, 6> motor_gains_{};
  std::vector<arm_msgs::msg::RobotCommand> commands_;
  std::array<std::pair<float, float>, 4> joint_q_limits_{};
};

#endif  // DMBOT_SERIAL_DEBUGGER_NODE_HPP_
