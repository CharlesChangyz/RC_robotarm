#ifndef RC_ARM_HARDWARE__SCALAR_PATH_TIME_PLANNER_HPP_
#define RC_ARM_HARDWARE__SCALAR_PATH_TIME_PLANNER_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "rc_arm_hardware/s_curve_generator.hpp"

namespace rc_arm_hardware
{

struct ScalarPathPlannerConfig
{
  bool enabled{true};
  size_t dof{0};
  double waypoint_merge_distance{1e-3};
  double command_append_distance{1e-3};
  double max_waypoint_jump{1.2};
  size_t max_raw_waypoints{120};
  size_t min_waypoints{2};
  size_t constraint_samples{200};

  std::vector<std::string> joint_names;
  std::vector<double> velocity_limits;
  std::vector<double> acceleration_limits;
  std::vector<double> jerk_limits;
  std::vector<double> lower_limits;
  std::vector<double> upper_limits;
  std::vector<bool> is_continuous;
};

struct ScalarPathSample
{
  bool valid{false};
  double t{0.0};
  double s{0.0};
  double sd{0.0};
  double sdd{0.0};
  std::vector<double> q;
  std::vector<double> v;
  std::vector<double> a;
};

struct ScalarPathDiagnostics
{
  bool profile_active{false};
  size_t raw_waypoints{0};
  size_t cleaned_waypoints{0};
  double path_length{0.0};
  double total_time{0.0};
  double v_s_limit{0.0};
  double a_s_limit{0.0};
  double j_s_limit{0.0};
  size_t vel_bottleneck_joint{0};
  size_t acc_bottleneck_joint{0};
  std::string status;
};

class ScalarPathTimePlanner
{
public:
  explicit ScalarPathTimePlanner(const ScalarPathPlannerConfig& config);

  void resetToPosition(const std::vector<double>& q0);

  // Returns true if a new valid profile has been built.
  bool ingestWaypoint(const std::vector<double>& waypoint, bool force_rebuild, std::string& message);

  bool hasActiveProfile() const;

  // Advance profile by dt and return sampled q/v/a.
  bool sample(double dt, ScalarPathSample& out);

  const ScalarPathDiagnostics& diagnostics() const { return diagnostics_; }

private:
  struct CubicSplineSegment
  {
    double s0{0.0};
    double s1{0.0};
    double a{0.0};
    double b{0.0};
    double c{0.0};
    double d{0.0};
  };

  struct PathModel
  {
    bool valid{false};
    double length{0.0};
    std::vector<double> s_knots;
    std::vector<std::vector<CubicSplineSegment>> joint_segments;
  };

  bool isFiniteVector(const std::vector<double>& q) const;
  double weightedDistance(const std::vector<double>& q0, const std::vector<double>& q1) const;
  double unwrapToNearest(double reference, double value) const;

  bool preprocessWaypoints(
    const std::vector<std::vector<double>>& raw,
    std::vector<std::vector<double>>& cleaned,
    std::string& reason) const;

  bool buildPathModel(
    const std::vector<std::vector<double>>& waypoints,
    PathModel& out_model,
    std::string& reason) const;

  bool evaluatePath(
    const PathModel& model,
    double s,
    std::vector<double>& q,
    std::vector<double>& q_s,
    std::vector<double>& q_ss) const;

  bool computeGlobalConservativeLimits(
    const PathModel& model,
    double& v_s,
    double& a_s,
    double& j_s,
    size_t& vel_bottleneck,
    size_t& acc_bottleneck,
    std::string& reason) const;

  bool rebuildProfileFromRaw(std::string& message);

private:
  ScalarPathPlannerConfig config_;
  ScalarPathDiagnostics diagnostics_;

  std::vector<std::vector<double>> raw_waypoints_;
  std::vector<std::vector<double>> cleaned_waypoints_;

  PathModel path_model_;

  SCurveGenerator scalar_s_generator_;
  SCurveProfile scalar_s_profile_;

  bool profile_active_{false};
  double elapsed_time_{0.0};

  std::vector<double> current_q_;
};

}  // namespace rc_arm_hardware

#endif  // RC_ARM_HARDWARE__SCALAR_PATH_TIME_PLANNER_HPP_
