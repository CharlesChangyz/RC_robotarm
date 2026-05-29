#include "dmbot_serial/dm_motor_driver.hpp"

#include <algorithm>
#include <array>
#include <exception>

namespace dmbot_serial
{

namespace
{
constexpr size_t kDriverSlotCount = 6;
constexpr size_t kJ5FeedbackIndex = 4;
}

DmMotorDriver::DmMotorDriver(
  std::string serial_number,
  uint32_t nominal_baud,
  uint32_t data_baud,
  std::vector<MotorConfig> motor_configs)
: serial_number_(std::move(serial_number)),
  nominal_baud_(nominal_baud),
  data_baud_(data_baud),
  motor_configs_(std::move(motor_configs))
{
}

DmMotorDriver::~DmMotorDriver()
{
  disconnect();
}

bool DmMotorDriver::connect()
{
  std::lock_guard<std::mutex> lock(driver_mutex_);
  if (motor_control_) {
    return true;
  }

  init_data_.clear();
  init_data_.reserve(motor_configs_.size());
  for (const auto & config : motor_configs_) {
    damiao::DmActData data{};
    data.motorType = toDamiaoMotorType(config.motor_type);
    data.mode = damiao::MIT_MODE;
    data.can_id = config.motor_id;
    data.mst_id = config.master_id;
    init_data_.push_back(data);
  }

  try {
    motor_control_ = std::make_shared<damiao::Motor_Control>(
      nominal_baud_, data_baud_, serial_number_, &init_data_);
  } catch (const std::exception &) {
    motor_control_.reset();
    return false;
  } catch (...) {
    motor_control_.reset();
    return false;
  }

  return static_cast<bool>(motor_control_);
}

void DmMotorDriver::disconnect()
{
  std::lock_guard<std::mutex> lock(driver_mutex_);
  motor_control_.reset();
}

bool DmMotorDriver::isConnected() const
{
  std::lock_guard<std::mutex> lock(driver_mutex_);
  return static_cast<bool>(motor_control_);
}

bool DmMotorDriver::enable()
{
  std::lock_guard<std::mutex> lock(driver_mutex_);
  if (!motor_control_) {
    return false;
  }
  motor_control_->enable_motor();
  return true;
}

bool DmMotorDriver::disable()
{
  std::lock_guard<std::mutex> lock(driver_mutex_);
  if (!motor_control_) {
    return false;
  }
  motor_control_->disable_motor();
  return true;
}

bool DmMotorDriver::enableVacuum()
{
  std::lock_guard<std::mutex> lock(driver_mutex_);
  if (!motor_control_) {
    return false;
  }
  motor_control_->enable_vacuum_gripper();
  return true;
}

bool DmMotorDriver::disableVacuum()
{
  std::lock_guard<std::mutex> lock(driver_mutex_);
  if (!motor_control_) {
    return false;
  }
  motor_control_->disable_vacuum_gripper();
  return true;
}

bool DmMotorDriver::writeCommands(
  const std::vector<double> & position,
  const std::vector<double> & velocity,
  const std::vector<double> & kp,
  const std::vector<double> & kd,
  const std::vector<double> & effort)
{
  std::lock_guard<std::mutex> lock(driver_mutex_);
  if (!motor_control_) {
    return false;
  }

  std::array<float, kDriverSlotCount> pos{};
  std::array<float, kDriverSlotCount> vel{};
  std::array<float, kDriverSlotCount> gains_p{};
  std::array<float, kDriverSlotCount> gains_d{};
  std::array<float, kDriverSlotCount> tau{};

  const size_t count = std::min(motor_configs_.size(), kDriverSlotCount);
  for (size_t i = 0; i < count; ++i) {
    pos[i] = static_cast<float>(position[i]);
    vel[i] = static_cast<float>(velocity[i]);
    gains_p[i] = static_cast<float>(kp[i]);
    gains_d[i] = static_cast<float>(kd[i]);
    tau[i] = static_cast<float>(effort[i]);
  }

  motor_control_->CtrlMotors(
    pos.data(), vel.data(), gains_p.data(), gains_d.data(), tau.data());
  return true;
}

bool DmMotorDriver::writeJ5Command(double position, double kp, double kd)
{
  std::lock_guard<std::mutex> lock(driver_mutex_);
  if (!motor_control_) {
    return false;
  }

  motor_control_->CtrlMotors_2(
    static_cast<float>(position),
    0.0f,
    static_cast<float>(kp),
    static_cast<float>(kd),
    0.0f);
  return true;
}

std::vector<MotorState> DmMotorDriver::readStates() const
{
  std::lock_guard<std::mutex> lock(driver_mutex_);
  std::vector<MotorState> states(motor_configs_.size());
  if (!motor_control_) {
    return states;
  }

  for (size_t i = 0; i < motor_configs_.size(); ++i) {
    const size_t feedback_index = feedbackIndexForMotor(motor_configs_[i], i);
    if (feedback_index >= kDriverSlotCount) {
      continue;
    }
    states[i].position = motor_control_->current_motor_pos[feedback_index];
    states[i].velocity = motor_control_->current_motor_vel[feedback_index];
    states[i].effort = motor_control_->current_motor_tor[feedback_index];
    states[i].valid = true;
  }

  return states;
}

double DmMotorDriver::readJ5Position() const
{
  std::lock_guard<std::mutex> lock(driver_mutex_);
  if (!motor_control_) {
    return 0.0;
  }

  return motor_control_->current_motor_pos[kJ5FeedbackIndex];
}

uint32_t DmMotorDriver::readLaserDistance() const
{
  std::lock_guard<std::mutex> lock(driver_mutex_);
  if (!motor_control_) {
    return 0;
  }

  return motor_control_->laser_distance;
}

damiao::DM_Motor_Type DmMotorDriver::toDamiaoMotorType(MotorType type)
{
  switch (type) {
    case MotorType::DM4340:
      return damiao::DM4340;
    case MotorType::DM4310:
    default:
      return damiao::DM4310;
  }
}

size_t DmMotorDriver::feedbackIndexForMotor(const MotorConfig & config, size_t ordinal) const
{
  if (config.motor_id < kDriverSlotCount) {
    return static_cast<size_t>(config.motor_id);
  }
  return std::min(ordinal, kDriverSlotCount - 1);
}

}  // namespace dmbot_serial
