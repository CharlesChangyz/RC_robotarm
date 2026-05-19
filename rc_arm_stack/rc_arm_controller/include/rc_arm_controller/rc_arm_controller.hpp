#ifndef RC_ARM_CONTROLLER__RC_ARM_CONTROLLER_HPP_
#define RC_ARM_CONTROLLER__RC_ARM_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace rc_arm_controller
{

class RcArmController : public controller_interface::ControllerInterface
{
public:
  RcArmController();

  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  struct TrajectoryPoint
  {
    double time_from_start{0.0};
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> acceleration;
    std::vector<double> effort;
  };

  void topic_trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

  bool normalize_reference_point(
    const trajectory_msgs::msg::JointTrajectory & msg,
    TrajectoryPoint & normalized_point,
    std::string & error) const;
  std::vector<size_t> build_joint_permutation(
    const std::vector<std::string> & incoming_joint_names,
    std::string & error) const;
  void set_hold_command_from_current_state();
  void set_command_from_point(const TrajectoryPoint & point);

  std::vector<std::string> joint_names_;
  double reference_timeout_{0.1};
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr topic_subscription_;

  mutable std::mutex reference_mutex_;
  TrajectoryPoint active_reference_;
  bool has_active_reference_{false};
  rclcpp::Time last_reference_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace rc_arm_controller

#endif  // RC_ARM_CONTROLLER__RC_ARM_CONTROLLER_HPP_
