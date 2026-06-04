#ifndef DMBOT_SERIAL__DM_MOTOR_DRIVER_HPP_
#define DMBOT_SERIAL__DM_MOTOR_DRIVER_HPP_

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "dmbot_serial/protocol/damiao.h"

namespace dmbot_serial
{

enum class MotorType
{
  DM4310 = 1,
  DM4340 = 3,
};

struct MotorConfig
{
  uint16_t motor_id;
  uint16_t master_id;
  MotorType motor_type;
};

struct MotorState
{
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
  bool valid{false};
};

struct RawCanFrame
{
  uint32_t id{0};
  bool is_extended{false};
  bool is_remote{false};
  bool is_fd{true};
  uint8_t dlc{0};
  std::array<uint8_t, 64> data{};
};

class DmMotorDriver
{
public:
  DmMotorDriver(
    std::string serial_number,
    uint32_t nominal_baud,
    uint32_t data_baud,
    std::vector<MotorConfig> motor_configs);
  ~DmMotorDriver();

  bool connect();
  void disconnect();
  bool isConnected() const;

  bool enable();
  bool disable();
  bool enableVacuum();
  bool disableVacuum();

  bool writeCommands(
    const std::vector<double> & position,
    const std::vector<double> & velocity,
    const std::vector<double> & kp,
    const std::vector<double> & kd,
    const std::vector<double> & effort);
  bool writeJ5Command(double position, double kp, double kd);
  void setRawFrameCallback(std::function<void(const RawCanFrame &)> callback);
  bool sendRawFrame(const RawCanFrame & frame);

  std::vector<MotorState> readStates() const;
  double readCameraPosition() const;
  double readJ5Position() const;
  uint32_t readLaserDistance() const;

private:
  static damiao::DM_Motor_Type toDamiaoMotorType(MotorType type);
  size_t feedbackIndexForMotor(const MotorConfig & config, size_t ordinal) const;

  std::string serial_number_;
  uint32_t nominal_baud_;
  uint32_t data_baud_;
  std::vector<MotorConfig> motor_configs_;
  std::vector<damiao::DmActData> init_data_;
  std::shared_ptr<damiao::Motor_Control> motor_control_;
  mutable std::mutex driver_mutex_;
  mutable std::mutex raw_frame_callback_mutex_;
  std::function<void(const RawCanFrame &)> raw_frame_callback_;
};

}  // namespace dmbot_serial

#endif  // DMBOT_SERIAL__DM_MOTOR_DRIVER_HPP_
