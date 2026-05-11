#ifndef RC_ARM_CONTROLLER__RC_ARM_CONTROLLER_HPP_
#define RC_ARM_CONTROLLER__RC_ARM_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
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
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

  struct TrajectoryPoint
  {
    double time_from_start{0.0};
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> acceleration;
    std::vector<double> effort;
  };

  struct ActiveTrajectory
  {
    std::vector<TrajectoryPoint> points;
    rclcpp::Time start_time;
    std::shared_ptr<GoalHandle> goal_handle;
    bool from_topic{false};
  };

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
  void topic_trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

  bool normalize_trajectory(
    const trajectory_msgs::msg::JointTrajectory & msg,
    std::vector<TrajectoryPoint> & normalized_points,
    std::string & error) const;
  TrajectoryPoint sample_trajectory(
    const std::vector<TrajectoryPoint> & points,
    double elapsed_sec,
    bool & finished) const;
  std::vector<size_t> build_joint_permutation(
    const std::vector<std::string> & incoming_joint_names,
    std::string & error) const;
  void set_hold_command_from_current_state();
  void set_command_from_point(const TrajectoryPoint & point);
  void publish_feedback(
    const rclcpp::Time & time,
    const TrajectoryPoint & desired,
    const std::shared_ptr<GoalHandle> & goal_handle);
  void finish_goal(
    const std::shared_ptr<GoalHandle> & goal_handle,
    int32_t error_code,
    const std::string & error_string,
    bool canceled = false);

  std::vector<std::string> joint_names_;
  bool allow_topic_commands_{true};
  double feedback_publish_rate_{20.0};

  std::shared_ptr<rclcpp_action::Server<FollowJointTrajectory>> action_server_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr topic_subscription_;

  mutable std::mutex trajectory_mutex_;
  std::shared_ptr<ActiveTrajectory> active_trajectory_;
  rclcpp::Time last_feedback_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace rc_arm_controller

#endif  // RC_ARM_CONTROLLER__RC_ARM_CONTROLLER_HPP_
