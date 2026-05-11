#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "arm_msgs/srv/generate_joint_trajectory.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ruckig/input_parameter.hpp"
#include "ruckig/ruckig.hpp"
#include "ruckig/trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace
{

builtin_interfaces::msg::Duration to_duration_msg(const double seconds)
{
  builtin_interfaces::msg::Duration msg;
  const double clamped = std::max(0.0, seconds);
  const auto sec = static_cast<int32_t>(std::floor(clamped));
  const auto nanosec = static_cast<uint32_t>((clamped - static_cast<double>(sec)) * 1.0e9);
  msg.sec = sec;
  msg.nanosec = nanosec;
  return msg;
}

template <typename T>
std::vector<double> normalize_vector(
  const std::vector<T> & values,
  const size_t expected_size,
  const double fallback)
{
  if (values.size() == expected_size) {
    return std::vector<double>(values.begin(), values.end());
  }
  return std::vector<double>(expected_size, fallback);
}

}  // namespace

class RuckigTrajectoryServer : public rclcpp::Node
{
public:
  RuckigTrajectoryServer()
  : Node("rc_arm_ruckig_trajectory_server")
  {
    service_ = create_service<arm_msgs::srv::GenerateJointTrajectory>(
      "~/generate_joint_trajectory",
      std::bind(
        &RuckigTrajectoryServer::handle_request,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
  }

private:
  void handle_request(
    const std::shared_ptr<arm_msgs::srv::GenerateJointTrajectory::Request> request,
    std::shared_ptr<arm_msgs::srv::GenerateJointTrajectory::Response> response)
  {
    const auto dofs = request->joint_names.size();
    if (dofs == 0) {
      response->success = false;
      response->message = "joint_names cannot be empty";
      return;
    }

    if (
      request->current_position.size() != dofs || request->target_position.size() != dofs ||
      request->max_velocity.size() != dofs || request->max_acceleration.size() != dofs ||
      request->max_jerk.size() != dofs)
    {
      response->success = false;
      response->message = "vector size mismatch";
      return;
    }

    auto current_velocity = normalize_vector(request->current_velocity, dofs, 0.0);
    auto current_acceleration = normalize_vector(request->current_acceleration, dofs, 0.0);
    auto target_velocity = normalize_vector(request->target_velocity, dofs, 0.0);
    auto target_acceleration = normalize_vector(request->target_acceleration, dofs, 0.0);

    const double control_period = std::max(0.001, request->control_period);
    ruckig::Ruckig<0> otg(dofs, control_period);
    ruckig::InputParameter<0> input(dofs);
    ruckig::Trajectory<0> trajectory(dofs);

    input.current_position = request->current_position;
    input.current_velocity = current_velocity;
    input.current_acceleration = current_acceleration;
    input.target_position = request->target_position;
    input.target_velocity = target_velocity;
    input.target_acceleration = target_acceleration;
    input.max_velocity = request->max_velocity;
    input.max_acceleration = request->max_acceleration;
    input.max_jerk = request->max_jerk;
    if (request->minimum_duration > 0.0) {
      input.minimum_duration = request->minimum_duration;
    }

    const auto result = otg.calculate(input, trajectory);
    if (result != ruckig::Result::Working && result != ruckig::Result::Finished) {
      response->success = false;
      response->message = "ruckig calculation failed";
      return;
    }

    trajectory_msgs::msg::JointTrajectory message;
    message.joint_names = request->joint_names;

    trajectory_msgs::msg::JointTrajectoryPoint initial;
    initial.positions = request->current_position;
    initial.velocities = current_velocity;
    initial.accelerations = current_acceleration;
    initial.time_from_start = to_duration_msg(0.0);
    message.points.push_back(initial);

    std::vector<double> positions(dofs, 0.0);
    std::vector<double> velocities(dofs, 0.0);
    std::vector<double> accelerations(dofs, 0.0);
    const double duration = trajectory.get_duration();
    for (double t = control_period; t < duration; t += control_period) {
      trajectory.at_time(t, positions, velocities, accelerations);
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = positions;
      point.velocities = velocities;
      point.accelerations = accelerations;
      point.time_from_start = to_duration_msg(t);
      message.points.push_back(point);
    }

    trajectory.at_time(duration, positions, velocities, accelerations);
    trajectory_msgs::msg::JointTrajectoryPoint final_point;
    final_point.positions = positions;
    final_point.velocities = velocities;
    final_point.accelerations = accelerations;
    final_point.time_from_start = to_duration_msg(duration);
    message.points.push_back(final_point);

    response->trajectory = message;
    response->success = true;
    response->message = "ok";
  }

  rclcpp::Service<arm_msgs::srv::GenerateJointTrajectory>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RuckigTrajectoryServer>());
  rclcpp::shutdown();
  return 0;
}
