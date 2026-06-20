#ifndef RC_ARM_HARDWARE__MOTOR_PARAMS_HPP_
#define RC_ARM_HARDWARE__MOTOR_PARAMS_HPP_

namespace rc_arm_hardware
{

enum class MotorType
{
  RS00,
  EL05
};

struct MotorParams
{
  double p_min;
  double p_max;
  double v_min;
  double v_max;
  double t_min;
  double t_max;
  double kp_min;
  double kp_max;
  double kd_min;
  double kd_max;
};

inline MotorParams getMotorParams(MotorType type)
{
  MotorParams params{};
  params.p_min = -12.57;
  params.p_max = 12.57;
  params.kp_min = 0.0;
  params.kp_max = 500.0;
  params.kd_min = 0.0;
  params.kd_max = 5.0;

  switch (type) {
    case MotorType::RS00:
      params.v_min = -33.0;
      params.v_max = 33.0;
      params.t_min = -14.0;
      params.t_max = 14.0;
      break;
    case MotorType::EL05:
      params.v_min = -50.0;
      params.v_max = 50.0;
      params.t_min = -6.0;
      params.t_max = 6.0;
      break;
  }

  return params;
}

}  // namespace rc_arm_hardware

#endif  // RC_ARM_HARDWARE__MOTOR_PARAMS_HPP_
