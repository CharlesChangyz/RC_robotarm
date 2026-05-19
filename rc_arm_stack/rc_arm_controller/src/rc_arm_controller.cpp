#include "rc_arm_controller/rc_arm_controller.hpp"

#include <algorithm>
#include <functional>
#include <limits>
#include <string>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace rc_arm_controller
{

RcArmController::RcArmController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn RcArmController::on_init()
{
  auto_declare<std::vector<std::string>>("joints", {});
  auto_declare<double>("reference_timeout", 0.1);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration RcArmController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint_name : joint_names_) {
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_ACCELERATION);
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return config;
}

controller_interface::InterfaceConfiguration RcArmController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint_name : joint_names_) {
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
    config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return config;
}

controller_interface::CallbackReturn RcArmController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  reference_timeout_ = std::max(0.0, get_node()->get_parameter("reference_timeout").as_double());

  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "parameter 'joints' cannot be empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  topic_subscription_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "~/joint_trajectory",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&RcArmController::topic_trajectory_callback, this, std::placeholders::_1));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RcArmController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (command_interfaces_.size() != joint_names_.size() * 4U) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "expected %zu command interfaces, got %zu",
      joint_names_.size() * 4U,
      command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (state_interfaces_.size() != joint_names_.size() * 3U) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "expected %zu state interfaces, got %zu",
      joint_names_.size() * 3U,
      state_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  {
    std::lock_guard<std::mutex> lock(reference_mutex_);
    has_active_reference_ = false;
  }
  set_hold_command_from_current_state();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RcArmController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  {
    std::lock_guard<std::mutex> lock(reference_mutex_);
    has_active_reference_ = false;
  }
  set_hold_command_from_current_state();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type RcArmController::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & /*period*/)
{
  TrajectoryPoint reference_point;
  bool has_reference = false;
  bool stale_reference = false;
  {
    std::lock_guard<std::mutex> lock(reference_mutex_);
    has_reference = has_active_reference_;
    if (has_reference) {
      reference_point = active_reference_;
      stale_reference =
        reference_timeout_ > 0.0 && (time - last_reference_time_).seconds() > reference_timeout_;
      if (stale_reference) {
        has_active_reference_ = false;
      }
    }
  }

  if (!has_reference || stale_reference) {
    set_hold_command_from_current_state();
    return controller_interface::return_type::OK;
  }

  set_command_from_point(reference_point);
  return controller_interface::return_type::OK;
}

void RcArmController::topic_trajectory_callback(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  TrajectoryPoint normalized_point;
  std::string error;
  if (!normalize_reference_point(*msg, normalized_point, error)) {
    RCLCPP_WARN(get_node()->get_logger(), "ignoring streaming trajectory: %s", error.c_str());
    return;
  }

  std::lock_guard<std::mutex> lock(reference_mutex_);
  active_reference_ = std::move(normalized_point);
  has_active_reference_ = true;
  last_reference_time_ = get_node()->now();
}

bool RcArmController::normalize_reference_point(
  const trajectory_msgs::msg::JointTrajectory & msg,
  TrajectoryPoint & normalized_point,
  std::string & error) const
{
  if (msg.points.size() != 1U) {
    error = "streaming controller expects exactly one trajectory point";
    return false;
  }

  std::vector<size_t> permutation = build_joint_permutation(msg.joint_names, error);
  if (permutation.empty()) {
    return false;
  }

  const auto & point = msg.points.front();
  if (point.positions.size() != msg.joint_names.size()) {
    error = "positions size does not match joint count";
    return false;
  }
  if (!point.velocities.empty() && point.velocities.size() != msg.joint_names.size()) {
    error = "velocities size does not match joint count";
    return false;
  }
  if (!point.accelerations.empty() && point.accelerations.size() != msg.joint_names.size()) {
    error = "accelerations size does not match joint count";
    return false;
  }
  if (!point.effort.empty() && point.effort.size() != msg.joint_names.size()) {
    error = "effort size does not match joint count";
    return false;
  }

  normalized_point.time_from_start =
    static_cast<double>(point.time_from_start.sec) +
    static_cast<double>(point.time_from_start.nanosec) * 1e-9;
  normalized_point.position.resize(joint_names_.size(), 0.0);
  normalized_point.velocity.resize(joint_names_.size(), 0.0);
  normalized_point.acceleration.resize(joint_names_.size(), 0.0);
  normalized_point.effort.resize(joint_names_.size(), 0.0);

  for (size_t controller_index = 0; controller_index < permutation.size(); ++controller_index) {
    const size_t source_index = permutation[controller_index];
    normalized_point.position[controller_index] = point.positions[source_index];
    if (!point.velocities.empty()) {
      normalized_point.velocity[controller_index] = point.velocities[source_index];
    }
    if (!point.accelerations.empty()) {
      normalized_point.acceleration[controller_index] = point.accelerations[source_index];
    }
    if (!point.effort.empty()) {
      normalized_point.effort[controller_index] = point.effort[source_index];
    }
  }

  return true;
}

std::vector<size_t> RcArmController::build_joint_permutation(
  const std::vector<std::string> & incoming_joint_names,
  std::string & error) const
{
  std::vector<size_t> permutation;
  const auto & source_names = incoming_joint_names.empty() ? joint_names_ : incoming_joint_names;

  if (source_names.size() != joint_names_.size()) {
    error = "joint count mismatch";
    return {};
  }

  permutation.resize(joint_names_.size(), std::numeric_limits<size_t>::max());
  for (size_t controller_index = 0; controller_index < joint_names_.size(); ++controller_index) {
    const auto it = std::find(source_names.begin(), source_names.end(), joint_names_[controller_index]);
    if (it == source_names.end()) {
      error = "missing joint '" + joint_names_[controller_index] + "'";
      return {};
    }
    permutation[controller_index] = static_cast<size_t>(std::distance(source_names.begin(), it));
  }

  return permutation;
}

void RcArmController::set_hold_command_from_current_state()
{
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const double position = state_interfaces_[3 * i].get_value();
    command_interfaces_[4 * i].set_value(position);
    command_interfaces_[4 * i + 1].set_value(0.0);
    command_interfaces_[4 * i + 2].set_value(0.0);
    command_interfaces_[4 * i + 3].set_value(0.0);
  }
}

void RcArmController::set_command_from_point(const TrajectoryPoint & point)
{
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces_[4 * i].set_value(point.position[i]);
    command_interfaces_[4 * i + 1].set_value(point.velocity[i]);
    command_interfaces_[4 * i + 2].set_value(point.acceleration[i]);
    command_interfaces_[4 * i + 3].set_value(point.effort[i]);
  }
}

}  // namespace rc_arm_controller

PLUGINLIB_EXPORT_CLASS(rc_arm_controller::RcArmController, controller_interface::ControllerInterface)
