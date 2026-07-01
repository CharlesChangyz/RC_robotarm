#ifndef RC_ARM_HARDWARE__PAYLOAD_BLEND_HPP_
#define RC_ARM_HARDWARE__PAYLOAD_BLEND_HPP_

#include <algorithm>
#include <cmath>

namespace rc_arm_hardware
{

inline double clampPayloadBlend(double value)
{
  if (!std::isfinite(value)) {
    return 0.0;
  }
  return std::clamp(value, 0.0, 1.0);
}

inline double lerpPayloadValue(double unloaded_value, double payload_value, double blend)
{
  const double clamped_blend = clampPayloadBlend(blend);
  return unloaded_value * (1.0 - clamped_blend) + payload_value * clamped_blend;
}

inline void updatePayloadBlend(
  double & current_blend,
  double target_blend,
  double dt,
  double ramp_up_sec,
  double ramp_down_sec,
  bool enabled)
{
  target_blend = clampPayloadBlend(target_blend);
  current_blend = clampPayloadBlend(current_blend);

  if (!enabled) {
    current_blend = target_blend;
    return;
  }
  if (dt <= 0.0 || !std::isfinite(dt)) {
    return;
  }

  const double delta = target_blend - current_blend;
  if (std::abs(delta) <= 1e-12) {
    current_blend = target_blend;
    return;
  }

  const double ramp_sec = delta > 0.0 ? ramp_up_sec : ramp_down_sec;
  if (ramp_sec <= 0.0 || !std::isfinite(ramp_sec)) {
    current_blend = target_blend;
    return;
  }

  const double max_step = dt / ramp_sec;
  if (std::abs(delta) <= max_step) {
    current_blend = target_blend;
    return;
  }

  current_blend += (delta > 0.0 ? max_step : -max_step);
  current_blend = clampPayloadBlend(current_blend);
}

}  // namespace rc_arm_hardware

#endif  // RC_ARM_HARDWARE__PAYLOAD_BLEND_HPP_
