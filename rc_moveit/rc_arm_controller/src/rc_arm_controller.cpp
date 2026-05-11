#include "rc_arm_controller/rc_arm_controller.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <limits.h>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <utility>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace rc_arm_controller
{

namespace
{

int64_t wall_time_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

bool ensure_directory_exists(const std::string & path)
{
  if (path.empty()) {
    return false;
  }

  std::string current;
  if (!path.empty() && path[0] == '/') {
    current = "/";
  }

  std::stringstream stream(path);
  std::string segment;
  while (std::getline(stream, segment, '/')) {
    if (segment.empty()) {
      continue;
    }
    if (!current.empty() && current != "/") {
      current += "/";
    }
    current += segment;
    if (::mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) {
      return false;
    }
  }
  return true;
}

std::string default_timing_dir()
{
  const std::string source_path = __FILE__;
  const std::string marker = "/rc_moveit/";
  const auto marker_pos = source_path.rfind(marker);
  if (marker_pos != std::string::npos) {
    return source_path.substr(0, marker_pos) + "/logs/rc_arm_timing";
  }

  char cwd[PATH_MAX];
  if (::getcwd(cwd, sizeof(cwd)) != nullptr) {
    return std::string(cwd) + "/logs/rc_arm_timing";
  }
  return "/tmp/rc_arm_timing";
}

std::string resolve_timing_log_path()
{
  const char * configured = std::getenv("RC_ARM_TIMING_DIR");
  std::string dir = (configured != nullptr && configured[0] != '\0') ?
    std::string(configured) : default_timing_dir();
  ensure_directory_exists(dir);
  return dir + "/controller.events.jsonl";
}

std::string json_escape(const std::string & text)
{
  std::ostringstream out;
  for (const char ch : text) {
    switch (ch) {
      case '\\':
        out << "\\\\";
        break;
      case '"':
        out << "\\\"";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        out << ch;
        break;
    }
  }
  return out.str();
}

}  // namespace

RcArmController::RcArmController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn RcArmController::on_init()
{
  auto_declare<std::vector<std::string>>("joints", {});
  auto_declare<bool>("allow_topic_commands", false);
  auto_declare<double>("feedback_publish_rate", 20.0);
  timing_log_path_ = resolve_timing_log_path();
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
  allow_topic_commands_ = get_node()->get_parameter("allow_topic_commands").as_bool();
  feedback_publish_rate_ = get_node()->get_parameter("feedback_publish_rate").as_double();

  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "parameter 'joints' cannot be empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
    get_node(),
    "~/follow_joint_trajectory",
    std::bind(&RcArmController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&RcArmController::handle_cancel, this, std::placeholders::_1),
    std::bind(&RcArmController::handle_accepted, this, std::placeholders::_1));

  timing_trace_subscription_ = get_node()->create_subscription<std_msgs::msg::String>(
    "/rc_arm_2/timing_trace_id",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&RcArmController::timing_trace_callback, this, std::placeholders::_1));

  if (allow_topic_commands_) {
    topic_subscription_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "~/joint_trajectory",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&RcArmController::topic_trajectory_callback, this, std::placeholders::_1));
  } else {
    topic_subscription_.reset();
  }

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
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    active_trajectory_.reset();
    pending_trace_id_.clear();
  }
  set_hold_command_from_current_state();
  last_feedback_time_ = get_node()->now();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RcArmController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::shared_ptr<GoalHandle> goal_handle;
  std::string trace_id;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (active_trajectory_) {
      goal_handle = active_trajectory_->goal_handle;
      trace_id = active_trajectory_->trace_id;
      active_trajectory_.reset();
    }
  }
  if (goal_handle) {
    log_timing_event("controller_goal_deactivated", trace_id, "failed", "controller deactivated");
    finish_goal(
      goal_handle,
      FollowJointTrajectory::Result::INVALID_GOAL,
      "controller deactivated",
      true);
  }
  set_hold_command_from_current_state();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type RcArmController::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & /*period*/)
{
  std::shared_ptr<ActiveTrajectory> trajectory;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    trajectory = active_trajectory_;
  }

  if (!trajectory) {
    return controller_interface::return_type::OK;
  }

  if (trajectory->goal_handle && trajectory->goal_handle->is_canceling()) {
    set_hold_command_from_current_state();
    log_timing_event("controller_goal_canceled", trajectory->trace_id, "failed", "goal canceled");
    finish_goal(
      trajectory->goal_handle,
      FollowJointTrajectory::Result::SUCCESSFUL,
      "goal canceled",
      true);
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (active_trajectory_ == trajectory) {
      active_trajectory_.reset();
    }
    return controller_interface::return_type::OK;
  }

  const double elapsed_sec = std::max(0.0, (time - trajectory->start_time).seconds());
  bool finished = false;
  const auto sampled = sample_trajectory(trajectory->points, elapsed_sec, finished);
  set_command_from_point(sampled);

  if (trajectory->goal_handle) {
    const double min_feedback_period =
      feedback_publish_rate_ > 0.0 ? (1.0 / feedback_publish_rate_) : 0.0;
    if (min_feedback_period <= 0.0 || (time - last_feedback_time_).seconds() >= min_feedback_period) {
      publish_feedback(time, sampled, trajectory->goal_handle);
      last_feedback_time_ = time;
    }
  }

  if (finished) {
    if (trajectory->goal_handle) {
      log_timing_event("controller_goal_finished", trajectory->trace_id, "ok");
      finish_goal(
        trajectory->goal_handle,
        FollowJointTrajectory::Result::SUCCESSFUL,
        "");
    }
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (active_trajectory_ == trajectory) {
      active_trajectory_.reset();
    }
  }

  return controller_interface::return_type::OK;
}

rclcpp_action::GoalResponse RcArmController::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const FollowJointTrajectory::Goal> goal)
{
  std::vector<TrajectoryPoint> normalized_points;
  std::string error;
  if (!goal) {
    std::string trace_id;
    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      trace_id = pending_trace_id_;
      pending_trace_id_.clear();
    }
    log_timing_event("controller_goal_rejected", trace_id, "failed", "null goal");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!normalize_trajectory(goal->trajectory, normalized_points, error)) {
    std::string trace_id;
    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      trace_id = pending_trace_id_;
      pending_trace_id_.clear();
    }
    log_timing_event("controller_goal_rejected", trace_id, "failed", error);
    RCLCPP_WARN(get_node()->get_logger(), "rejecting FollowJointTrajectory goal: %s", error.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RcArmController::handle_cancel(
  const std::shared_ptr<GoalHandle> /*goal_handle*/)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void RcArmController::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  std::vector<TrajectoryPoint> normalized_points;
  std::string error;
  if (!normalize_trajectory(goal_handle->get_goal()->trajectory, normalized_points, error)) {
    std::string trace_id;
    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      trace_id = pending_trace_id_;
      pending_trace_id_.clear();
    }
    log_timing_event("controller_goal_invalid", trace_id, "failed", error);
    finish_goal(goal_handle, FollowJointTrajectory::Result::INVALID_GOAL, error);
    return;
  }

  auto next_trajectory = std::make_shared<ActiveTrajectory>();
  next_trajectory->points = std::move(normalized_points);
  next_trajectory->start_time = get_node()->now();
  next_trajectory->goal_handle = goal_handle;
  next_trajectory->from_topic = false;
  next_trajectory->trace_id = std::to_string(next_trajectory->start_time.nanoseconds());

  std::shared_ptr<ActiveTrajectory> previous_trajectory;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (active_trajectory_) {
      previous_trajectory = active_trajectory_;
    }
    if (!pending_trace_id_.empty()) {
      next_trajectory->trace_id = pending_trace_id_;
      pending_trace_id_.clear();
    }
    active_trajectory_ = next_trajectory;
  }
  log_timing_event("controller_goal_accepted", next_trajectory->trace_id, "ok");

  if (previous_trajectory && previous_trajectory->goal_handle != goal_handle) {
    log_timing_event(
      "controller_goal_preempted",
      previous_trajectory->trace_id,
      "failed",
      "preempted by newer goal");
    finish_goal(
      previous_trajectory->goal_handle,
      FollowJointTrajectory::Result::INVALID_GOAL,
      "preempted by newer goal");
  }
}

void RcArmController::topic_trajectory_callback(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  std::vector<TrajectoryPoint> normalized_points;
  std::string error;
  if (!normalize_trajectory(*msg, normalized_points, error)) {
    RCLCPP_WARN(get_node()->get_logger(), "ignoring topic trajectory: %s", error.c_str());
    return;
  }

  auto next_trajectory = std::make_shared<ActiveTrajectory>();
  next_trajectory->points = std::move(normalized_points);
  next_trajectory->start_time = get_node()->now();
  next_trajectory->from_topic = true;
  next_trajectory->trace_id = std::to_string(next_trajectory->start_time.nanoseconds());

  std::lock_guard<std::mutex> lock(trajectory_mutex_);
  pending_trace_id_.clear();
  active_trajectory_ = next_trajectory;
}

void RcArmController::timing_trace_callback(const std_msgs::msg::String::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  const std::string trace_id = msg->data;
  if (trace_id.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(trajectory_mutex_);
  pending_trace_id_ = trace_id;
}

bool RcArmController::normalize_trajectory(
  const trajectory_msgs::msg::JointTrajectory & msg,
  std::vector<TrajectoryPoint> & normalized_points,
  std::string & error) const
{
  if (msg.points.empty()) {
    error = "trajectory contains no points";
    return false;
  }

  std::vector<size_t> permutation = build_joint_permutation(msg.joint_names, error);
  if (permutation.empty()) {
    return false;
  }

  normalized_points.clear();
  normalized_points.reserve(msg.points.size());

  double previous_time = -1.0;
  for (const auto & point : msg.points) {
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

    const double point_time =
      static_cast<double>(point.time_from_start.sec) +
      static_cast<double>(point.time_from_start.nanosec) * 1e-9;
    if (point_time < previous_time) {
      error = "time_from_start must be monotonic";
      return false;
    }
    previous_time = point_time;

    TrajectoryPoint normalized;
    normalized.time_from_start = point_time;
    normalized.position.resize(joint_names_.size(), 0.0);
    normalized.velocity.resize(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());
    normalized.acceleration.resize(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());
    normalized.effort.resize(joint_names_.size(), 0.0);

    for (size_t controller_index = 0; controller_index < permutation.size(); ++controller_index) {
      const size_t source_index = permutation[controller_index];
      normalized.position[controller_index] = point.positions[source_index];
      if (!point.velocities.empty()) {
        normalized.velocity[controller_index] = point.velocities[source_index];
      }
      if (!point.accelerations.empty()) {
        normalized.acceleration[controller_index] = point.accelerations[source_index];
      }
      if (!point.effort.empty()) {
        normalized.effort[controller_index] = point.effort[source_index];
      }
    }

    normalized_points.push_back(std::move(normalized));
  }

  return true;
}

RcArmController::TrajectoryPoint RcArmController::sample_trajectory(
  const std::vector<TrajectoryPoint> & points,
  double elapsed_sec,
  bool & finished) const
{
  finished = false;
  if (points.empty()) {
    return {};
  }

  if (elapsed_sec <= points.front().time_from_start) {
    return points.front();
  }

  const auto & last_point = points.back();
  if (elapsed_sec >= last_point.time_from_start) {
    finished = true;
    return last_point;
  }

  for (size_t i = 0; i + 1 < points.size(); ++i) {
    const auto & start = points[i];
    const auto & stop = points[i + 1];
    if (elapsed_sec > stop.time_from_start) {
      continue;
    }

    const double dt = stop.time_from_start - start.time_from_start;
    const double alpha = dt > 1e-9 ? (elapsed_sec - start.time_from_start) / dt : 1.0;

    TrajectoryPoint sampled;
    sampled.time_from_start = elapsed_sec;
    sampled.position.resize(joint_names_.size(), 0.0);
    sampled.velocity.resize(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());
    sampled.acceleration.resize(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());
    sampled.effort.resize(joint_names_.size(), 0.0);

    for (size_t joint_index = 0; joint_index < joint_names_.size(); ++joint_index) {
      sampled.position[joint_index] =
        start.position[joint_index] + alpha * (stop.position[joint_index] - start.position[joint_index]);
      if (std::isfinite(start.velocity[joint_index]) && std::isfinite(stop.velocity[joint_index])) {
        sampled.velocity[joint_index] =
          start.velocity[joint_index] + alpha * (stop.velocity[joint_index] - start.velocity[joint_index]);
      }
      if (std::isfinite(start.acceleration[joint_index]) && std::isfinite(stop.acceleration[joint_index])) {
        sampled.acceleration[joint_index] =
          start.acceleration[joint_index] +
          alpha * (stop.acceleration[joint_index] - start.acceleration[joint_index]);
      }
      sampled.effort[joint_index] =
        start.effort[joint_index] + alpha * (stop.effort[joint_index] - start.effort[joint_index]);
    }
    return sampled;
  }

  finished = true;
  return last_point;
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

void RcArmController::publish_feedback(
  const rclcpp::Time & time,
  const TrajectoryPoint & desired,
  const std::shared_ptr<GoalHandle> & goal_handle)
{
  if (!goal_handle) {
    return;
  }

  auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
  feedback->header.stamp = time;
  feedback->joint_names = joint_names_;
  feedback->desired.positions = desired.position;
  feedback->desired.velocities = desired.velocity;
  feedback->desired.accelerations = desired.acceleration;
  feedback->desired.effort = desired.effort;
  feedback->actual.positions.resize(joint_names_.size(), 0.0);
  feedback->actual.velocities.resize(joint_names_.size(), 0.0);
  feedback->actual.accelerations.resize(joint_names_.size(), 0.0);
  feedback->actual.effort.resize(joint_names_.size(), 0.0);
  feedback->error.positions.resize(joint_names_.size(), 0.0);
  feedback->error.velocities.resize(joint_names_.size(), 0.0);
  feedback->error.accelerations.resize(joint_names_.size(), 0.0);
  feedback->error.effort.resize(joint_names_.size(), 0.0);

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const double actual_position = state_interfaces_[3 * i].get_value();
    const double actual_velocity = state_interfaces_[3 * i + 1].get_value();
    const double actual_effort = state_interfaces_[3 * i + 2].get_value();
    feedback->actual.positions[i] = actual_position;
    feedback->actual.velocities[i] = actual_velocity;
    feedback->actual.effort[i] = actual_effort;
    feedback->error.positions[i] = desired.position[i] - actual_position;
    feedback->error.velocities[i] = desired.velocity[i] - actual_velocity;
    feedback->error.accelerations[i] = desired.acceleration[i];
    feedback->error.effort[i] = desired.effort[i] - actual_effort;
  }

  goal_handle->publish_feedback(feedback);
}

void RcArmController::finish_goal(
  const std::shared_ptr<GoalHandle> & goal_handle,
  int32_t error_code,
  const std::string & error_string,
  bool canceled)
{
  if (!goal_handle) {
    return;
  }

  auto result = std::make_shared<FollowJointTrajectory::Result>();
  result->error_code = error_code;
  result->error_string = error_string;

  if (canceled) {
    goal_handle->canceled(result);
  } else if (error_code == FollowJointTrajectory::Result::SUCCESSFUL) {
    goal_handle->succeed(result);
  } else {
    goal_handle->abort(result);
  }
}

void RcArmController::log_timing_event(
  const std::string & event,
  const std::string & trace_id,
  const std::string & status,
  const std::string & detail) const
{
  std::lock_guard<std::mutex> lock(timing_file_mutex_);
  std::ofstream out(timing_log_path_, std::ios::app);
  if (!out.is_open()) {
    return;
  }

  out
    << "{\"trace_id\":\"" << json_escape(trace_id)
    << "\",\"component\":\"controller\""
    << ",\"event\":\"" << json_escape(event)
    << "\",\"wall_time_ns\":" << wall_time_now_ns()
    << ",\"ros_stamp_ns\":" << get_node()->now().nanoseconds()
    << ",\"status\":\"" << json_escape(status) << "\"";
  if (!detail.empty()) {
    out << ",\"detail\":\"" << json_escape(detail) << "\"";
  }
  out << "}\n";
}

}  // namespace rc_arm_controller

PLUGINLIB_EXPORT_CLASS(rc_arm_controller::RcArmController, controller_interface::ControllerInterface)
