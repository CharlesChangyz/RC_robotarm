#include "dmbot_serial/debugger_node.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

RobotCommandPublisher::RobotCommandPublisher()
: Node("robot_command_publisher")
{
  publisher_ = this->create_publisher<arm_msgs::msg::RobotCommand>("robot_command", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&RobotCommandPublisher::timer_callback, this));

  motor_gains_ = {
    std::pair<float, float>{0.001F, 0.005F}, {5.0F, 1.0F}, {5.0F, 1.0F},
    {5.0F, 1.0F}, {0.0F, 0.0F}, {0.0F, 0.0F}};

  commands_ = create_commands();
  joint_q_limits_ = {{{-1.865F, 2.365F}, {-1.85F, 0.0F}, {-2.13861F, 0.0F}, {-0.5313F, 1.2963F}}};
  apply_q_limits_to_commands();

  running_.store(true);

  RCLCPP_INFO(this->get_logger(), "Robot Command Publisher 已启动");
  RCLCPP_INFO(this->get_logger(), "按数字键 1-9/0 切换不同的发布消息 (0对应命令10)");
  RCLCPP_INFO(this->get_logger(), "当前命令: 1 (is_enable=true)");

  keyboard_thread_ = std::thread(&RobotCommandPublisher::keyboard_listener, this);
}

RobotCommandPublisher::~RobotCommandPublisher()
{
  running_.store(false);
  if (keyboard_thread_.joinable()) {
    keyboard_thread_.join();
  }
}

std::vector<arm_msgs::msg::RobotCommand> RobotCommandPublisher::create_commands()
{
  std::vector<arm_msgs::msg::RobotCommand> commands;
  commands.reserve(10);

  arm_msgs::msg::RobotCommand cmd1;
  cmd1.is_enable = true;
  cmd1.motor_command.resize(6);
  for (int i = 0; i < 6; ++i) {
    auto & motor_cmd = cmd1.motor_command[i];
    motor_cmd.q = 0.0F;
    motor_cmd.dq = 0.0F;
    motor_cmd.tau = 0.0F;
    motor_cmd.kp = 0.0F;
    motor_cmd.kd = 0.0F;
  }
  commands.push_back(cmd1);

  arm_msgs::msg::RobotCommand cmd2;
  cmd2.is_enable = true;
  cmd2.motor_command.resize(6);
  for (int i = 0; i < 6; ++i) {
    auto & motor_cmd = cmd2.motor_command[i];
    motor_cmd.q = (i == 1) ? -0.1F : 0.0F;
    motor_cmd.dq = 0.0F;
    motor_cmd.tau = 0.0F;
    motor_cmd.kp = motor_gains_[i].first;
    motor_cmd.kd = motor_gains_[i].second;
  }
  commands.push_back(cmd2);

  arm_msgs::msg::RobotCommand cmd3;
  cmd3.is_enable = true;
  cmd3.motor_command.resize(6);
  for (int i = 0; i < 6; ++i) {
    auto & motor_cmd = cmd3.motor_command[i];
    motor_cmd.q = (i == 2) ? -0.1F : 0.0F;
    motor_cmd.dq = 0.0F;
    motor_cmd.tau = 0.0F;
    motor_cmd.kp = motor_gains_[i].first;
    motor_cmd.kd = motor_gains_[i].second;
  }
  commands.push_back(cmd3);

  arm_msgs::msg::RobotCommand cmd4;
  cmd4.is_enable = true;
  cmd4.motor_command.resize(6);
  for (int i = 0; i < 6; ++i) {
    auto & motor_cmd = cmd4.motor_command[i];
    motor_cmd.q = (i == 3) ? 0.1F : 0.0F;
    motor_cmd.dq = 0.0F;
    motor_cmd.tau = 0.0F;
    motor_cmd.kp = motor_gains_[i].first;
    motor_cmd.kd = motor_gains_[i].second;
  }
  commands.push_back(cmd4);

  arm_msgs::msg::RobotCommand cmd5;
  cmd5.is_enable = true;
  cmd5.motor_command.resize(6);
  for (int i = 0; i < 6; ++i) {
    auto & motor_cmd = cmd5.motor_command[i];
    motor_cmd.q = 0.0F;
    motor_cmd.dq = 0.0F;
    motor_cmd.tau = 0.0F;
    motor_cmd.kp = motor_gains_[i].first;
    motor_cmd.kd = motor_gains_[i].second;
  }
  commands.push_back(cmd5);

  arm_msgs::msg::RobotCommand cmd6;
  cmd6.is_enable = false;
  cmd6.motor_command.resize(6);
  for (int i = 0; i < 6; ++i) {
    auto & motor_cmd = cmd6.motor_command[i];
    motor_cmd.q = 0.0F;
    motor_cmd.dq = 0.0F;
    motor_cmd.tau = 0.0F;
    motor_cmd.kp = 0.0F;
    motor_cmd.kd = 0.0F;
  }
  commands.push_back(cmd6);

  arm_msgs::msg::RobotCommand cmd7;
  cmd7.is_enable = true;
  cmd7.motor_command.resize(6);
  const std::array<float, 6> q7{{1.2F, -1.2F, -1.4F, 0.9F, 0.0F, 0.0F}};
  for (int i = 0; i < 6; ++i) {
    auto & motor_cmd = cmd7.motor_command[i];
    motor_cmd.q = q7[i];
    motor_cmd.dq = 0.0F;
    motor_cmd.tau = 0.0F;
    motor_cmd.kp = motor_gains_[i].first;
    motor_cmd.kd = motor_gains_[i].second;
  }
  commands.push_back(cmd7);

  arm_msgs::msg::RobotCommand cmd8;
  cmd8.is_enable = true;
  cmd8.motor_command.resize(6);
  const std::array<float, 6> q8{{-1.2F, -0.4F, -0.6F, -0.3F, 0.0F, 0.0F}};
  for (int i = 0; i < 6; ++i) {
    auto & motor_cmd = cmd8.motor_command[i];
    motor_cmd.q = q8[i];
    motor_cmd.dq = 0.0F;
    motor_cmd.tau = 0.0F;
    motor_cmd.kp = motor_gains_[i].first;
    motor_cmd.kd = motor_gains_[i].second;
  }
  commands.push_back(cmd8);

  arm_msgs::msg::RobotCommand cmd9;
  cmd9.is_enable = true;
  cmd9.motor_command.resize(6);
  const std::array<float, 6> q9{{0.8F, -1.6F, -1.9F, 1.1F, 0.0F, 0.0F}};
  for (int i = 0; i < 6; ++i) {
    auto & motor_cmd = cmd9.motor_command[i];
    motor_cmd.q = q9[i];
    motor_cmd.dq = 0.0F;
    motor_cmd.tau = 0.0F;
    motor_cmd.kp = motor_gains_[i].first;
    motor_cmd.kd = motor_gains_[i].second;
  }
  commands.push_back(cmd9);

  arm_msgs::msg::RobotCommand cmd10;
  cmd10.is_enable = true;
  cmd10.motor_command.resize(6);
  const std::array<float, 6> q10{{0.2F, -0.3F, -0.5F, -0.5F, 0.0F, 0.0F}};
  for (int i = 0; i < 6; ++i) {
    auto & motor_cmd = cmd10.motor_command[i];
    motor_cmd.q = q10[i];
    motor_cmd.dq = 0.0F;
    motor_cmd.tau = 0.0F;
    motor_cmd.kp = motor_gains_[i].first;
    motor_cmd.kd = motor_gains_[i].second;
  }
  commands.push_back(cmd10);

  return commands;
}

float RobotCommandPublisher::clamp(float value, float min_value, float max_value)
{
  return std::max(min_value, std::min(value, max_value));
}

void RobotCommandPublisher::apply_q_limits_to_commands()
{
  for (auto & cmd : commands_) {
    const std::size_t count = std::min<std::size_t>(4, cmd.motor_command.size());
    for (std::size_t i = 0; i < count; ++i) {
      cmd.motor_command[i].q = clamp(cmd.motor_command[i].q, joint_q_limits_[i].first, joint_q_limits_[i].second);
    }
  }
}

void RobotCommandPublisher::timer_callback()
{
  if (current_command_idx_ >= static_cast<int>(commands_.size())) {
    return;
  }

  auto msg = commands_[current_command_idx_];
  if (current_command_idx_ == 4) {
    const float swing = 0.1F * std::sin(swing_phase_);
    swing_phase_ += 0.15F;
    const std::array<float, 6> base_q{{0.0F, -0.2F, -0.2F, 0.2F, 0.0F, 0.0F}};
    for (int i = 0; i < 6; ++i) {
      msg.motor_command[i].q = base_q[i] + swing;
      msg.motor_command[i].dq = 0.0F;
    }
    const std::size_t count = std::min<std::size_t>(4, msg.motor_command.size());
    for (std::size_t i = 0; i < count; ++i) {
      msg.motor_command[i].q =
        clamp(msg.motor_command[i].q, joint_q_limits_[i].first, joint_q_limits_[i].second);
    }
  }

  publisher_->publish(msg);
  const char * enable_status = msg.is_enable ? "启用" : "禁用";
  RCLCPP_INFO(this->get_logger(), "发布命令 %d: %s", current_command_idx_ + 1, enable_status);
}

void RobotCommandPublisher::keyboard_listener()
{
  termios old_settings{};
  termios new_settings{};
  if (tcgetattr(STDIN_FILENO, &old_settings) != 0) {
    RCLCPP_WARN(this->get_logger(), "无法读取终端属性，键盘监听未启动");
    return;
  }

  new_settings = old_settings;
  new_settings.c_lflag &= static_cast<unsigned long>(~(ICANON | ECHO));
  new_settings.c_cc[VMIN] = 0;
  new_settings.c_cc[VTIME] = 1;
  tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);

  while (running_.load() && rclcpp::ok()) {
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);
    timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000;

    const int ret = select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &timeout);
    if (ret > 0 && FD_ISSET(STDIN_FILENO, &readfds)) {
      char key = 0;
      if (read(STDIN_FILENO, &key, 1) <= 0) {
        continue;
      }
      if (key >= '1' && key <= '9') {
        current_command_idx_ = static_cast<int>(key - '1');
        const char * status = commands_[current_command_idx_].is_enable ? "启用" : "禁用";
        RCLCPP_INFO(this->get_logger(), "切换到命令 %c: %s", key, status);
      } else if (key == '0') {
        current_command_idx_ = 9;
        const char * status = commands_[current_command_idx_].is_enable ? "启用" : "禁用";
        RCLCPP_INFO(this->get_logger(), "切换到命令 10: %s", status);
      } else if (key == 'q') {
        RCLCPP_INFO(this->get_logger(), "退出程序");
        running_.store(false);
        break;
      }
    }
  }

  tcsetattr(STDIN_FILENO, TCSANOW, &old_settings);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotCommandPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
