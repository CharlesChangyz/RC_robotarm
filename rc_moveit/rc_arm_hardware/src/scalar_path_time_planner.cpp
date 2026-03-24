#include "rc_arm_hardware/scalar_path_time_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace rc_arm_hardware
{

namespace
{
constexpr double kEps = 1e-9;
constexpr double kPi = 3.14159265358979323846;

inline double clampAbsMin(double value, double min_value)
{
  return (std::abs(value) < min_value) ? min_value : value;
}

}  // namespace

ScalarPathTimePlanner::ScalarPathTimePlanner(const ScalarPathPlannerConfig& config)
  : config_(config)
  , scalar_s_generator_(1.0, 1.0, 1.0)
{
  if (config_.dof == 0) {
    config_.enabled = false;
    diagnostics_.status = "invalid dof";
    return;
  }

  auto resize_to_dof = [&](auto& vec, const auto default_value) {
    if (vec.size() != config_.dof) {
      vec.resize(config_.dof, default_value);
    }
  };

  resize_to_dof(config_.joint_names, std::string("joint"));
  resize_to_dof(config_.velocity_limits, 1.0);
  resize_to_dof(config_.acceleration_limits, 1.0);
  resize_to_dof(config_.jerk_limits, 1.0);
  resize_to_dof(config_.lower_limits, -std::numeric_limits<double>::infinity());
  resize_to_dof(config_.upper_limits, std::numeric_limits<double>::infinity());
  resize_to_dof(config_.is_continuous, false);

  for (size_t i = 0; i < config_.dof; ++i) {
    config_.velocity_limits[i] = clampAbsMin(std::abs(config_.velocity_limits[i]), 1e-3);
    config_.acceleration_limits[i] = clampAbsMin(std::abs(config_.acceleration_limits[i]), 1e-3);
    config_.jerk_limits[i] = clampAbsMin(std::abs(config_.jerk_limits[i]), 1e-3);
  }

  current_q_.assign(config_.dof, 0.0);
  diagnostics_.status = "idle";
}

void ScalarPathTimePlanner::resetToPosition(const std::vector<double>& q0)
{
  if (q0.size() == config_.dof) {
    current_q_ = q0;
  }

  raw_waypoints_.clear();
  cleaned_waypoints_.clear();
  path_model_ = PathModel();
  scalar_s_profile_ = SCurveProfile();
  profile_active_ = false;
  elapsed_time_ = 0.0;
  diagnostics_.profile_active = false;
  diagnostics_.status = "reset";
}

bool ScalarPathTimePlanner::hasActiveProfile() const
{
  return profile_active_ && path_model_.valid;
}

bool ScalarPathTimePlanner::isFiniteVector(const std::vector<double>& q) const
{
  if (q.size() != config_.dof) {
    return false;
  }
  for (double v : q) {
    if (!std::isfinite(v)) {
      return false;
    }
  }
  return true;
}

double ScalarPathTimePlanner::weightedDistance(const std::vector<double>& q0, const std::vector<double>& q1) const
{
  double accum = 0.0;
  for (size_t i = 0; i < config_.dof; ++i) {
    const double w = 1.0 / std::max(config_.velocity_limits[i], 1e-3);
    const double dq = (q1[i] - q0[i]) * w;
    accum += dq * dq;
  }
  return std::sqrt(accum);
}

double ScalarPathTimePlanner::unwrapToNearest(double reference, double value) const
{
  double candidate = value;
  while (candidate - reference > kPi) {
    candidate -= 2.0 * kPi;
  }
  while (candidate - reference < -kPi) {
    candidate += 2.0 * kPi;
  }
  return candidate;
}

bool ScalarPathTimePlanner::preprocessWaypoints(
  const std::vector<std::vector<double>>& raw,
  std::vector<std::vector<double>>& cleaned,
  std::string& reason) const
{
  cleaned.clear();

  if (raw.empty()) {
    reason = "empty waypoint list";
    return false;
  }

  if (raw.size() == 1) {
    reason = "single-point trajectory";
    return false;
  }

  std::vector<std::vector<double>> unwrapped = raw;

  for (const auto& q : unwrapped) {
    if (!isFiniteVector(q)) {
      reason = "contains NaN/Inf or invalid dimension";
      return false;
    }
  }

  // Continuous joint unwrap.
  for (size_t i = 0; i < config_.dof; ++i) {
    if (!config_.is_continuous[i]) {
      continue;
    }
    for (size_t k = 1; k < unwrapped.size(); ++k) {
      unwrapped[k][i] = unwrapToNearest(unwrapped[k - 1][i], unwrapped[k][i]);
    }
  }

  // Bounds + jump checks.
  for (size_t k = 0; k < unwrapped.size(); ++k) {
    for (size_t i = 0; i < config_.dof; ++i) {
      if (!config_.is_continuous[i]) {
        const double lo = config_.lower_limits[i] - 1e-6;
        const double hi = config_.upper_limits[i] + 1e-6;
        if (unwrapped[k][i] < lo || unwrapped[k][i] > hi) {
          reason = "waypoint exceeds joint limits";
          return false;
        }
      }

      if (k > 0) {
        const double jump = std::abs(unwrapped[k][i] - unwrapped[k - 1][i]);
        if (jump > config_.max_waypoint_jump) {
          reason = "extreme waypoint jump detected";
          return false;
        }
      }
    }
  }

  cleaned.push_back(unwrapped.front());
  for (size_t k = 1; k < unwrapped.size(); ++k) {
    if (weightedDistance(cleaned.back(), unwrapped[k]) > config_.waypoint_merge_distance) {
      cleaned.push_back(unwrapped[k]);
    }
  }

  if (weightedDistance(cleaned.back(), unwrapped.back()) > kEps) {
    cleaned.push_back(unwrapped.back());
  }

  if (cleaned.size() < 2) {
    reason = "all waypoints collapsed after dedup";
    return false;
  }

  return true;
}

bool ScalarPathTimePlanner::buildPathModel(
  const std::vector<std::vector<double>>& waypoints,
  PathModel& out_model,
  std::string& reason) const
{
  out_model = PathModel();

  if (waypoints.size() < 2) {
    reason = "not enough waypoints";
    return false;
  }

  const size_t n = waypoints.size();
  out_model.s_knots.assign(n, 0.0);

  for (size_t k = 1; k < n; ++k) {
    const double ds = weightedDistance(waypoints[k - 1], waypoints[k]);
    if (ds < kEps) {
      reason = "degenerate segment after preprocessing";
      return false;
    }
    out_model.s_knots[k] = out_model.s_knots[k - 1] + ds;
  }

  out_model.length = out_model.s_knots.back();
  if (out_model.length < kEps) {
    reason = "zero path length";
    return false;
  }

  out_model.joint_segments.assign(config_.dof, std::vector<CubicSplineSegment>());

  // Build natural cubic spline for each joint.
  for (size_t j = 0; j < config_.dof; ++j) {
    std::vector<double> y(n, 0.0);
    for (size_t k = 0; k < n; ++k) {
      y[k] = waypoints[k][j];
    }

    std::vector<double> m(n, 0.0);
    if (n > 2) {
      std::vector<double> lower(n, 0.0), diag(n, 0.0), upper(n, 0.0), rhs(n, 0.0);

      diag[0] = 1.0;
      diag[n - 1] = 1.0;

      for (size_t i = 1; i + 1 < n; ++i) {
        const double h0 = out_model.s_knots[i] - out_model.s_knots[i - 1];
        const double h1 = out_model.s_knots[i + 1] - out_model.s_knots[i];
        lower[i] = h0;
        diag[i] = 2.0 * (h0 + h1);
        upper[i] = h1;
        rhs[i] = 6.0 * ((y[i + 1] - y[i]) / h1 - (y[i] - y[i - 1]) / h0);
      }

      // Thomas algorithm.
      for (size_t i = 1; i < n; ++i) {
        const double w = lower[i] / diag[i - 1];
        diag[i] -= w * upper[i - 1];
        rhs[i] -= w * rhs[i - 1];
      }
      m[n - 1] = rhs[n - 1] / diag[n - 1];
      for (size_t i = n - 1; i-- > 0;) {
        m[i] = (rhs[i] - upper[i] * m[i + 1]) / diag[i];
      }
    }

    auto& segs = out_model.joint_segments[j];
    segs.reserve(n - 1);

    for (size_t k = 0; k + 1 < n; ++k) {
      const double s0 = out_model.s_knots[k];
      const double s1 = out_model.s_knots[k + 1];
      const double h = s1 - s0;

      CubicSplineSegment seg;
      seg.s0 = s0;
      seg.s1 = s1;
      seg.a = y[k];
      seg.b = (y[k + 1] - y[k]) / h - h * (2.0 * m[k] + m[k + 1]) / 6.0;
      seg.c = m[k] / 2.0;
      seg.d = (m[k + 1] - m[k]) / (6.0 * h);
      segs.push_back(seg);
    }
  }

  out_model.valid = true;
  return true;
}

bool ScalarPathTimePlanner::evaluatePath(
  const PathModel& model,
  double s,
  std::vector<double>& q,
  std::vector<double>& q_s,
  std::vector<double>& q_ss) const
{
  if (!model.valid || model.joint_segments.size() != config_.dof || model.s_knots.size() < 2) {
    return false;
  }

  const double s_clamped = std::clamp(s, 0.0, model.length);
  auto it = std::upper_bound(model.s_knots.begin(), model.s_knots.end(), s_clamped);
  size_t seg_idx = 0;
  if (it == model.s_knots.begin()) {
    seg_idx = 0;
  } else if (it == model.s_knots.end()) {
    seg_idx = model.s_knots.size() - 2;
  } else {
    seg_idx = static_cast<size_t>(std::distance(model.s_knots.begin(), it) - 1);
  }

  q.assign(config_.dof, 0.0);
  q_s.assign(config_.dof, 0.0);
  q_ss.assign(config_.dof, 0.0);

  for (size_t j = 0; j < config_.dof; ++j) {
    const auto& seg = model.joint_segments[j][seg_idx];
    const double u = s_clamped - seg.s0;
    q[j] = seg.a + seg.b * u + seg.c * u * u + seg.d * u * u * u;
    q_s[j] = seg.b + 2.0 * seg.c * u + 3.0 * seg.d * u * u;
    q_ss[j] = 2.0 * seg.c + 6.0 * seg.d * u;
  }

  return true;
}

bool ScalarPathTimePlanner::computeGlobalConservativeLimits(
  const PathModel& model,
  double& v_s,
  double& a_s,
  double& j_s,
  size_t& vel_bottleneck,
  size_t& acc_bottleneck,
  std::string& reason) const
{
  if (!model.valid || model.length < kEps) {
    reason = "invalid path for constraints";
    return false;
  }

  const size_t samples = std::max<size_t>(config_.constraint_samples, 32);

  v_s = std::numeric_limits<double>::infinity();
  vel_bottleneck = 0;

  std::vector<double> q, qs, qss;
  for (size_t k = 0; k < samples; ++k) {
    const double ratio = static_cast<double>(k) / static_cast<double>(samples - 1);
    const double s = model.length * ratio;
    if (!evaluatePath(model, s, q, qs, qss)) {
      reason = "failed to evaluate path for velocity limits";
      return false;
    }
    for (size_t i = 0; i < config_.dof; ++i) {
      if (std::abs(qs[i]) < 1e-9) {
        continue;
      }
      const double cand = config_.velocity_limits[i] / std::abs(qs[i]);
      if (cand < v_s) {
        v_s = cand;
        vel_bottleneck = i;
      }
    }
  }

  if (!std::isfinite(v_s) || v_s < 1e-4) {
    reason = "invalid global path velocity limit";
    return false;
  }

  // Conservative acceleration bound with coupling term |q_ss|*sd^2.
  double test_v_s = v_s;
  a_s = 0.0;
  acc_bottleneck = 0;
  bool acc_ok = false;

  for (size_t iter = 0; iter < 8; ++iter) {
    double min_as = std::numeric_limits<double>::infinity();
    size_t min_joint = 0;
    bool feasible = true;

    for (size_t k = 0; k < samples; ++k) {
      const double ratio = static_cast<double>(k) / static_cast<double>(samples - 1);
      const double s = model.length * ratio;
      if (!evaluatePath(model, s, q, qs, qss)) {
        reason = "failed to evaluate path for acceleration limits";
        return false;
      }

      for (size_t i = 0; i < config_.dof; ++i) {
        const double ai = config_.acceleration_limits[i];
        const double abs_qs = std::abs(qs[i]);
        const double abs_qss = std::abs(qss[i]);

        if (abs_qs < 1e-9) {
          if (abs_qss * test_v_s * test_v_s > ai + 1e-9) {
            feasible = false;
          }
          continue;
        }

        const double rem = ai - abs_qss * test_v_s * test_v_s;
        const double cand = rem / abs_qs;
        if (cand < min_as) {
          min_as = cand;
          min_joint = i;
        }
      }
    }

    if (feasible && std::isfinite(min_as) && min_as > 1e-4) {
      a_s = min_as;
      acc_bottleneck = min_joint;
      acc_ok = true;
      break;
    }

    test_v_s *= 0.8;
  }

  if (!acc_ok) {
    reason = "failed to find feasible global acceleration bound";
    return false;
  }

  v_s = test_v_s;

  // Conservative jerk bound: dominant term q_s * s_ddd.
  j_s = std::numeric_limits<double>::infinity();
  for (size_t k = 0; k < samples; ++k) {
    const double ratio = static_cast<double>(k) / static_cast<double>(samples - 1);
    const double s = model.length * ratio;
    if (!evaluatePath(model, s, q, qs, qss)) {
      reason = "failed to evaluate path for jerk limits";
      return false;
    }

    for (size_t i = 0; i < config_.dof; ++i) {
      const double abs_qs = std::abs(qs[i]);
      if (abs_qs < 1e-9) {
        continue;
      }
      const double cand = config_.jerk_limits[i] / abs_qs;
      j_s = std::min(j_s, cand);
    }
  }

  if (!std::isfinite(j_s) || j_s < 1e-4) {
    reason = "invalid global jerk bound";
    return false;
  }

  return true;
}

bool ScalarPathTimePlanner::rebuildProfileFromRaw(std::string& message)
{
  if (!config_.enabled) {
    message = "planner disabled";
    return false;
  }

  if (raw_waypoints_.empty()) {
    message = "no raw waypoints";
    return false;
  }

  // Always anchor path start at current executed position.
  raw_waypoints_.front() = current_q_;

  std::string reason;
  if (!preprocessWaypoints(raw_waypoints_, cleaned_waypoints_, reason)) {
    diagnostics_.status = reason;
    profile_active_ = false;
    message = reason;
    return false;
  }

  PathModel new_model;
  if (!buildPathModel(cleaned_waypoints_, new_model, reason)) {
    diagnostics_.status = reason;
    profile_active_ = false;
    message = reason;
    return false;
  }

  double v_s = 0.0;
  double a_s = 0.0;
  double j_s = 0.0;
  size_t vel_joint = 0;
  size_t acc_joint = 0;
  if (!computeGlobalConservativeLimits(new_model, v_s, a_s, j_s, vel_joint, acc_joint, reason)) {
    diagnostics_.status = reason;
    profile_active_ = false;
    message = reason;
    return false;
  }

  scalar_s_generator_.setConstraints(v_s, a_s, j_s);
  scalar_s_profile_ = scalar_s_generator_.calculateProfile(0.0, new_model.length, 0.0, 0.0);
  if (scalar_s_profile_.total_time <= 0.0) {
    diagnostics_.status = "invalid scalar S-curve profile";
    profile_active_ = false;
    message = diagnostics_.status;
    return false;
  }

  path_model_ = std::move(new_model);
  elapsed_time_ = 0.0;
  profile_active_ = true;

  diagnostics_.profile_active = true;
  diagnostics_.raw_waypoints = raw_waypoints_.size();
  diagnostics_.cleaned_waypoints = cleaned_waypoints_.size();
  diagnostics_.path_length = path_model_.length;
  diagnostics_.total_time = scalar_s_profile_.total_time;
  diagnostics_.v_s_limit = v_s;
  diagnostics_.a_s_limit = a_s;
  diagnostics_.j_s_limit = j_s;
  diagnostics_.vel_bottleneck_joint = vel_joint;
  diagnostics_.acc_bottleneck_joint = acc_joint;
  diagnostics_.status = "profile rebuilt";

  message = diagnostics_.status;
  return true;
}

bool ScalarPathTimePlanner::ingestWaypoint(
  const std::vector<double>& waypoint,
  bool force_rebuild,
  std::string& message)
{
  if (!config_.enabled) {
    message = "planner disabled";
    return false;
  }

  if (!isFiniteVector(waypoint)) {
    message = "invalid waypoint";
    return false;
  }

  if (raw_waypoints_.empty()) {
    raw_waypoints_.push_back(current_q_);
  }

  const double d_append = weightedDistance(raw_waypoints_.back(), waypoint);
  if (d_append > config_.command_append_distance || force_rebuild) {
    raw_waypoints_.push_back(waypoint);
    if (raw_waypoints_.size() > config_.max_raw_waypoints) {
      raw_waypoints_.erase(raw_waypoints_.begin() + 1);
    }
  }

  if (raw_waypoints_.size() < config_.min_waypoints) {
    message = "insufficient waypoints";
    return false;
  }

  return rebuildProfileFromRaw(message);
}

bool ScalarPathTimePlanner::sample(double dt, ScalarPathSample& out)
{
  out = ScalarPathSample();

  if (!hasActiveProfile()) {
    return false;
  }

  if (dt <= 0.0 || dt > 0.1) {
    dt = 0.005;
  }

  elapsed_time_ += dt;
  if (elapsed_time_ >= scalar_s_profile_.total_time) {
    elapsed_time_ = scalar_s_profile_.total_time;
  }

  const double s = scalar_s_generator_.getPositionAtTime(scalar_s_profile_, elapsed_time_);
  const double sd = scalar_s_generator_.getVelocityAtTime(scalar_s_profile_, elapsed_time_);
  const double sdd = scalar_s_generator_.getAccelerationAtTime(scalar_s_profile_, elapsed_time_);

  std::vector<double> q;
  std::vector<double> q_s;
  std::vector<double> q_ss;
  if (!evaluatePath(path_model_, s, q, q_s, q_ss)) {
    diagnostics_.status = "path evaluate failed";
    profile_active_ = false;
    return false;
  }

  out.valid = true;
  out.t = elapsed_time_;
  out.s = s;
  out.sd = sd;
  out.sdd = sdd;
  out.q = q;
  out.v.assign(config_.dof, 0.0);
  out.a.assign(config_.dof, 0.0);

  for (size_t i = 0; i < config_.dof; ++i) {
    out.v[i] = q_s[i] * sd;
    out.a[i] = q_ss[i] * sd * sd + q_s[i] * sdd;
  }

  current_q_ = out.q;

  if (elapsed_time_ >= scalar_s_profile_.total_time - 1e-6) {
    profile_active_ = false;
    diagnostics_.profile_active = false;
    diagnostics_.status = "profile finished";
  }

  return true;
}

}  // namespace rc_arm_hardware
