#ifndef DMBOT_SERIAL_USB2CANFD_DM_NODE_HPP_
#define DMBOT_SERIAL_USB2CANFD_DM_NODE_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "arm_msgs/msg/robot_command.hpp"
#include "dmbot_serial/protocol/damiao.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32.hpp"


class Usb2canfdDMNode : public rclcpp::Node
{
public:
  Usb2canfdDMNode();
  ~Usb2canfdDMNode() override;

private:
  void command_callback(const arm_msgs::msg::RobotCommand::SharedPtr msg);
  void publish_joint_state();
  void publish_ee_distance();
  void final_joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void final_pd_gain_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void final_torque_ff_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void send_commands();

  damiao::Control_Mode control_mode_{damiao::MIT_MODE};
  std::vector<damiao::DmActData> init_data_;
  std::shared_ptr<damiao::Motor_Control> motor_control_;
  std::vector<uint16_t> motor_ids_;
  std::unordered_map<uint16_t, std::size_t> motor_id_to_index_;
  std::size_t motor_count_{0};

  // Command arrays
  std::vector<float> q_arr_;
  std::vector<float> dq_arr_;
  std::vector<float> tau_arr_;
  std::vector<float> kp_arr_;
  std::vector<float> kd_arr_;
  std::mutex command_mutex_;

  rclcpp::Subscription<arm_msgs::msg::RobotCommand>::SharedPtr command_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ee_distance_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr ee_distance_timer_;
  rclcpp::TimerBase::SharedPtr command_timer_;

  // New subscribers for debug topics
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr final_joint_command_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr final_pd_gain_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr final_torque_ff_sub_;
};

#endif  // DMBOT_SERIAL_USB2CANFD_DM_NODE_HPP_
