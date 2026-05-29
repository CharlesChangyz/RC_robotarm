/**
 * @file rc_arm_hardware.cpp
 * @brief 
 */

#include "rc_arm_hardware/rc_arm_hardware.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <fstream>
#include <limits>
#include <sstream>
#include <vector>
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace rc_arm_hardware
{

namespace
{
bool parseBoolParam(const std::string& raw_value)
{
  std::string value = raw_value;
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value == "true" || value == "1" || value == "yes" || value == "on";
}

double getDoubleParamOr(
  const hardware_interface::HardwareInfo & info,
  const std::string & key,
  double fallback_value)
{
  const auto it = info.hardware_parameters.find(key);
  if (it == info.hardware_parameters.end()) {
    return fallback_value;
  }
  return std::stod(it->second);
}
}  // namespace

RsA3HardwareInterface::RsA3HardwareInterface()
  : backend_mode_(BackendMode::REAL)
  , backend_name_("real")
  , dm_serial_number_("9940F4E149D904A69924737E3DE6629F")
  , dm_nominal_baud_(1000000)
  , dm_data_baud_(2000000)
  , dm_control_mode_(0)
  , dm_startup_delay_sec_(0.0)
  , dm_enable_retry_count_(3)
  , dm_enable_retry_interval_sec_(1.0)
  , mujoco_command_topic_("/rc_arm_2/mujoco_joint_command")
  , velocity_limit_(10.0)
  , vacuum_activate_topic_("/rc_arm_2/vacuum_activate")
  , payload_command_topic_("/rc_arm_2/payload_active_command")
  , payload_active_topic_("/rc_arm_2/payload_active")
  , j5_command_topic_("/rc_arm_2/j5/command_position")
  , j5_position_topic_("/rc_arm_2/j5/actual_position")
  , payload_frame_("end_effector")
  , payload_mass_(0.63)
  , payload_diaginertia_{0.02, 0.02, 0.02}
  , payload_com_offset_{0.0, 0.0, 0.0}
  , payload_box_size_{0.05, 0.05, 0.05}
  , mujoco_payload_body_name_("payload_block")
  , mujoco_payload_site_name_("attachment_site")
  , mujoco_payload_initial_pos_{0.30, 0.0, 0.20}
  , payload_active_(false)
  , j5_kp_(0.0)
  , j5_kd_(0.0)
  , latest_j5_command_(0.0)
  , latest_j5_position_(0.0)
  , j5_command_received_(false)
  , control_mode_(ControlMode::POSITION)
  , use_mock_hardware_(false)
  , external_feedback_enabled_(false)
  , external_feedback_topic_("/rc_arm_2/feedback_joint_states")
  , external_feedback_timeout_sec_(0.2)
  , external_feedback_received_(false)
  , external_feedback_last_time_(std::chrono::steady_clock::now())
  , zero_torque_mode_(false)
  , zero_torque_kd_(1.0)
  , low_stiffness_mode_(false)
  , gravity_comp_enabled_(false)
  , gravity_feedforward_ratio_(0.5)  // 默认 50% 重力补偿前馈
  , use_pinocchio_gravity_(false)    // 默认使用简化重力模型
  , use_pinocchio_inverse_dynamics_(true)
  , pinocchio_initialized_(false)
  , pinocchio_loaded_initialized_(false)
  , pinocchio_mapping_ready_(false)
  , use_calibrated_inertia_(false)   // 默认不使用标定后的惯量参数
  , spin_thread_running_(false)      // 服务回调线程运行标志初始化
  , limit_margin_(0.15)          // 约在 ~15° 处开始减速（≈8.6°）
  , limit_stop_margin_(0.02)     // 约在 ~1° 处硬停止（≈1.1°）
  , limit_decel_factor_(0.3)     // 减速到 30%
{
}

RsA3HardwareInterface::~RsA3HardwareInterface()
{
  // 停止 spin 线程
  spin_thread_running_ = false;
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  on_shutdown(rclcpp_lifecycle::State());
}

hardware_interface::CallbackReturn RsA3HardwareInterface::on_init(
  const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 解析参数
  if (info_.hardware_parameters.count("backend")) {
    backend_name_ = info_.hardware_parameters.at("backend");
  }
  if (info_.hardware_parameters.count("velocity_limit")) {
    velocity_limit_ = std::stod(info_.hardware_parameters.at("velocity_limit"));
  }
  if (info_.hardware_parameters.count("vacuum_activate_topic")) {
    vacuum_activate_topic_ = info_.hardware_parameters.at("vacuum_activate_topic");
  }
  if (info_.hardware_parameters.count("payload_command_topic")) {
    payload_command_topic_ = info_.hardware_parameters.at("payload_command_topic");
  }
  if (info_.hardware_parameters.count("payload_active_topic")) {
    payload_active_topic_ = info_.hardware_parameters.at("payload_active_topic");
  }
  if (info_.hardware_parameters.count("j5_command_topic")) {
    j5_command_topic_ = info_.hardware_parameters.at("j5_command_topic");
  }
  if (info_.hardware_parameters.count("j5_position_topic")) {
    j5_position_topic_ = info_.hardware_parameters.at("j5_position_topic");
  }
  if (info_.hardware_parameters.count("payload_frame")) {
    payload_frame_ = info_.hardware_parameters.at("payload_frame");
  }
  j5_kp_ = getDoubleParamOr(info_, "j5_kp", j5_kp_);
  j5_kd_ = getDoubleParamOr(info_, "j5_kd", j5_kd_);
  payload_mass_ = getDoubleParamOr(
    info_, "payload_mass", payload_mass_);
  payload_diaginertia_[0] = getDoubleParamOr(info_, "payload_diaginertia_x", payload_diaginertia_[0]);
  payload_diaginertia_[1] = getDoubleParamOr(info_, "payload_diaginertia_y", payload_diaginertia_[1]);
  payload_diaginertia_[2] = getDoubleParamOr(info_, "payload_diaginertia_z", payload_diaginertia_[2]);
  payload_com_offset_[0] = getDoubleParamOr(info_, "payload_com_offset_x", payload_com_offset_[0]);
  payload_com_offset_[1] = getDoubleParamOr(info_, "payload_com_offset_y", payload_com_offset_[1]);
  payload_com_offset_[2] = getDoubleParamOr(info_, "payload_com_offset_z", payload_com_offset_[2]);
  payload_box_size_[0] = getDoubleParamOr(info_, "payload_box_size_x", payload_box_size_[0]);
  payload_box_size_[1] = getDoubleParamOr(info_, "payload_box_size_y", payload_box_size_[1]);
  payload_box_size_[2] = getDoubleParamOr(info_, "payload_box_size_z", payload_box_size_[2]);
  if (info_.hardware_parameters.count("mujoco_payload_body_name")) {
    mujoco_payload_body_name_ = info_.hardware_parameters.at("mujoco_payload_body_name");
  }
  if (info_.hardware_parameters.count("mujoco_payload_site_name")) {
    mujoco_payload_site_name_ = info_.hardware_parameters.at("mujoco_payload_site_name");
  }
  mujoco_payload_initial_pos_[0] = getDoubleParamOr(
    info_, "mujoco_payload_initial_pos_x", mujoco_payload_initial_pos_[0]);
  mujoco_payload_initial_pos_[1] = getDoubleParamOr(
    info_, "mujoco_payload_initial_pos_y", mujoco_payload_initial_pos_[1]);
  mujoco_payload_initial_pos_[2] = getDoubleParamOr(
    info_, "mujoco_payload_initial_pos_z", mujoco_payload_initial_pos_[2]);
  if (info_.hardware_parameters.count("use_mock_hardware")) {
    use_mock_hardware_ = parseBoolParam(info_.hardware_parameters.at("use_mock_hardware"));
  }
  if (info_.hardware_parameters.count("external_feedback_topic")) {
    external_feedback_topic_ = info_.hardware_parameters.at("external_feedback_topic");
  }
  if (info_.hardware_parameters.count("external_feedback_timeout")) {
    external_feedback_timeout_sec_ = std::stod(info_.hardware_parameters.at("external_feedback_timeout"));
  }
  external_feedback_timeout_sec_ = std::max(0.0, external_feedback_timeout_sec_);
  if (info_.hardware_parameters.count("mujoco_command_topic")) {
    mujoco_command_topic_ = info_.hardware_parameters.at("mujoco_command_topic");
  }
  if (info_.hardware_parameters.count("dm_sn")) {
    dm_serial_number_ = info_.hardware_parameters.at("dm_sn");
  }
  if (info_.hardware_parameters.count("dm_nom_baud")) {
    dm_nominal_baud_ = static_cast<uint32_t>(std::stoul(info_.hardware_parameters.at("dm_nom_baud")));
  }
  if (info_.hardware_parameters.count("dm_dat_baud")) {
    dm_data_baud_ = static_cast<uint32_t>(std::stoul(info_.hardware_parameters.at("dm_dat_baud")));
  }
  if (info_.hardware_parameters.count("dm_control_mode")) {
    dm_control_mode_ = std::stoi(info_.hardware_parameters.at("dm_control_mode"));
  }
  dm_startup_delay_sec_ = std::max(
    0.0,
    getDoubleParamOr(info_, "dm_startup_delay_sec", dm_startup_delay_sec_));
  dm_enable_retry_count_ = std::max(
    1,
    static_cast<int>(getDoubleParamOr(info_, "dm_enable_retry_count", dm_enable_retry_count_)));
  dm_enable_retry_interval_sec_ = std::max(
    0.0,
    getDoubleParamOr(info_, "dm_enable_retry_interval_sec", dm_enable_retry_interval_sec_));

  std::string backend_lower = backend_name_;
  std::transform(
    backend_lower.begin(), backend_lower.end(), backend_lower.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (backend_lower == "real") {
    backend_mode_ = BackendMode::REAL;
  } else if (backend_lower == "mujoco") {
    backend_mode_ = BackendMode::MUJOCO;
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger("RsA3HardwareInterface"),
      "unsupported backend '%s', expected 'real' or 'mujoco'",
      backend_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  external_feedback_enabled_ = backend_mode_ == BackendMode::MUJOCO;

  if (info_.hardware_parameters.count("low_stiffness_mode")) {
    low_stiffness_mode_ = parseBoolParam(info_.hardware_parameters.at("low_stiffness_mode"));
  }
  unloaded_gains_.position_kp = getDoubleParamOr(
    info_, "unloaded_position_kp", getDoubleParamOr(info_, "position_kp", unloaded_gains_.position_kp));
  unloaded_gains_.position_kd = getDoubleParamOr(
    info_, "unloaded_position_kd", getDoubleParamOr(info_, "position_kd", unloaded_gains_.position_kd));
  unloaded_gains_.low_stiffness_kp = std::clamp(
    getDoubleParamOr(
      info_, "unloaded_low_stiffness_kp",
      getDoubleParamOr(info_, "low_stiffness_kp", unloaded_gains_.low_stiffness_kp)),
    0.0, 500.0);
  unloaded_gains_.low_stiffness_kd = std::clamp(
    getDoubleParamOr(
      info_, "unloaded_low_stiffness_kd",
      getDoubleParamOr(info_, "low_stiffness_kd", unloaded_gains_.low_stiffness_kd)),
    0.0, 5.0);
  unloaded_gains_.low_stiffness_torque_bias = getDoubleParamOr(
    info_, "unloaded_low_stiffness_torque_bias",
    getDoubleParamOr(info_, "low_stiffness_torque_bias", unloaded_gains_.low_stiffness_torque_bias));

  payload_gains_.position_kp = getDoubleParamOr(
    info_, "payload_position_kp", unloaded_gains_.position_kp);
  payload_gains_.position_kd = getDoubleParamOr(
    info_, "payload_position_kd", unloaded_gains_.position_kd);
  payload_gains_.low_stiffness_kp = std::clamp(
    getDoubleParamOr(info_, "payload_low_stiffness_kp", unloaded_gains_.low_stiffness_kp),
    0.0, 500.0);
  payload_gains_.low_stiffness_kd = std::clamp(
    getDoubleParamOr(info_, "payload_low_stiffness_kd", unloaded_gains_.low_stiffness_kd),
    0.0, 5.0);
  payload_gains_.low_stiffness_torque_bias = getDoubleParamOr(
    info_, "payload_low_stiffness_torque_bias", unloaded_gains_.low_stiffness_torque_bias);

  if (info_.hardware_parameters.count("use_pinocchio_gravity")) {
    use_pinocchio_gravity_ = parseBoolParam(info_.hardware_parameters.at("use_pinocchio_gravity"));
  }
  if (info_.hardware_parameters.count("use_pinocchio_inverse_dynamics")) {
    use_pinocchio_inverse_dynamics_ = parseBoolParam(info_.hardware_parameters.at("use_pinocchio_inverse_dynamics"));
  }
  if (info_.hardware_parameters.count("urdf_path")) {
    urdf_path_ = info_.hardware_parameters.at("urdf_path");
  }
  // 解析关节配置
  if (!parseJointConfig(info)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 分配状态与指令向量
  size_t num_joints = joint_configs_.size();
  hw_positions_.resize(num_joints, 0.0);
  hw_velocities_.resize(num_joints, 0.0);
  hw_efforts_.resize(num_joints, 0.0);
  hw_temperatures_.resize(num_joints, 0.0);
  hw_commands_positions_.resize(num_joints, 0.0);
  hw_commands_velocities_.resize(num_joints, 0.0);
  hw_commands_accelerations_.resize(num_joints, 0.0);
  hw_commands_efforts_.resize(num_joints, 0.0);
  final_cmd_positions_.resize(num_joints, 0.0);
  final_cmd_velocities_.resize(num_joints, 0.0);
  final_cmd_efforts_.resize(num_joints, 0.0);
  final_cmd_kps_.resize(num_joints, 0.0);
  final_cmd_kds_.resize(num_joints, 0.0);
  final_cmd_torque_ff_.resize(num_joints, 0.0);
  external_feedback_positions_.resize(num_joints, 0.0);
  external_feedback_velocities_.resize(num_joints, 0.0);
  external_feedback_efforts_.resize(num_joints, 0.0);
  
  // 初始化参考轨迹缓存
  smoothed_positions_.resize(num_joints, 0.0);
  smoothed_velocities_.resize(num_joints, 0.0);
  smoothed_accelerations_.resize(num_joints, 0.0);
  
  // 初始化速度前馈相关变量
  last_cmd_positions_.resize(num_joints, 0.0);         // 上一周期指令位置
  cmd_velocities_.resize(num_joints, 0.0);             // 计算得到的指令速度
  filtered_cmd_velocities_.resize(num_joints, 0.0);    // 一阶滤波后的指令速度
  velocity_ff_stage2_.resize(num_joints, 0.0);         // 二阶滤波中间量
  
  // 默认参数
  max_velocity_ = 2.0;          // 最大速度 2 rad/s
  max_acceleration_ = 8.0;      // 最大加速度 8 rad/s²
  fallback_control_period_ = 0.005;  // 默认 200Hz -> 5ms
  first_command_ = true;
  gravity_feedforward_ratio_ = 0.5;  // 默认 50% 重力补偿前馈
  
  // 从参数读取速度上限
  if (info_.hardware_parameters.count("max_velocity")) {
    max_velocity_ = std::stod(info_.hardware_parameters.at("max_velocity"));
  }
  
  // 从参数读取加速度上限
  if (info_.hardware_parameters.count("max_acceleration")) {
    max_acceleration_ = std::stod(info_.hardware_parameters.at("max_acceleration"));
  }

  if (info_.hardware_parameters.count("fallback_control_period")) {
    fallback_control_period_ = std::stod(info_.hardware_parameters.at("fallback_control_period"));
  }
  fallback_control_period_ = std::max(1e-4, fallback_control_period_);
  
  // 从参数读取重力补偿前馈比例
  if (info_.hardware_parameters.count("gravity_feedforward_ratio")) {
    gravity_feedforward_ratio_ = std::stod(info_.hardware_parameters.at("gravity_feedforward_ratio"));
    gravity_feedforward_ratio_ = std::clamp(gravity_feedforward_ratio_, 0.0, 1.0);
  }
  
  // 从参数读取关节限位保护参数
  if (info_.hardware_parameters.count("limit_margin")) {
    limit_margin_ = std::stod(info_.hardware_parameters.at("limit_margin"));
  }
  if (info_.hardware_parameters.count("limit_stop_margin")) {
    limit_stop_margin_ = std::stod(info_.hardware_parameters.at("limit_stop_margin"));
  }
  if (info_.hardware_parameters.count("limit_decel_factor")) {
    limit_decel_factor_ = std::stod(info_.hardware_parameters.at("limit_decel_factor"));
    limit_decel_factor_ = std::clamp(limit_decel_factor_, 0.0, 1.0);
  }
  
  // 初始化关节限位保护状态
  joint_at_limit_.resize(num_joints, false);
  limit_warn_counter_.resize(num_joints, 0);

  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "已初始化：%zu 个关节，backend=%s",
              num_joints, backend_name_.c_str());
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "  轨迹参考：直接执行上层 position/velocity，max_vel=%.1f rad/s，max_acc=%.1f rad/s²",
              max_velocity_, max_acceleration_);
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "  回退控制周期：%.4f s",
              fallback_control_period_);
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "  空载 PID：Kp=%.1f，Kd=%.1f，低刚度[Kp=%.1f Kd=%.1f bias=%.3f]",
              unloaded_gains_.position_kp, unloaded_gains_.position_kd,
              unloaded_gains_.low_stiffness_kp, unloaded_gains_.low_stiffness_kd,
              unloaded_gains_.low_stiffness_torque_bias);
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "  带载 PID：Kp=%.1f，Kd=%.1f，低刚度[Kp=%.1f Kd=%.1f bias=%.3f]，重力补偿前馈比例=%.0f%%",
              payload_gains_.position_kp, payload_gains_.position_kd,
              payload_gains_.low_stiffness_kp, payload_gains_.low_stiffness_kd,
              payload_gains_.low_stiffness_torque_bias,
              gravity_feedforward_ratio_ * 100.0);
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "  低刚度模式：%s，vacuum_topic=%s，payload_command_topic=%s，payload_topic=%s",
              low_stiffness_mode_ ? "启用" : "禁用",
              vacuum_activate_topic_.c_str(),
              payload_command_topic_.c_str(),
              payload_active_topic_.c_str());
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "  J5：command_topic=%s，position_topic=%s，kp=%.3f，kd=%.3f",
              j5_command_topic_.c_str(),
              j5_position_topic_.c_str(),
              j5_kp_,
              j5_kd_);
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "  Pinocchio：gravity=%s, inverse_dynamics=%s",
              use_pinocchio_gravity_ ? "启用" : "禁用",
              use_pinocchio_inverse_dynamics_ ? "启用" : "禁用");

  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "关节限位保护：margin=%.3f rad，stop_margin=%.3f rad，decel_factor=%.2f",
              limit_margin_, limit_stop_margin_, limit_decel_factor_);

  if (external_feedback_enabled_) {
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "  MuJoCo 状态输入：topic=%s，timeout=%.3fs",
                external_feedback_topic_.c_str(), external_feedback_timeout_sec_);
  }

  // 初始化调试发布器节点
  debug_node_ = rclcpp::Node::make_shared("rc_arm_hw_debug");
  hw_cmd_pub_ = debug_node_->create_publisher<sensor_msgs::msg::JointState>("/debug/hw_command", 10);
  smoothed_cmd_pub_ = debug_node_->create_publisher<sensor_msgs::msg::JointState>("/debug/smoothed_command", 10);
  gravity_torque_pub_ = debug_node_->create_publisher<sensor_msgs::msg::JointState>("/debug/gravity_torque", 10);
  velocity_ff_pub_ = debug_node_->create_publisher<sensor_msgs::msg::JointState>("/debug/velocity_feedforward", 10);
  temperature_pub_ = debug_node_->create_publisher<sensor_msgs::msg::JointState>("/debug/motor_temperature", 10);
  final_cmd_pub_ = debug_node_->create_publisher<sensor_msgs::msg::JointState>("/debug/final_joint_command", 10);
  final_cmd_joint_frame_pub_ = debug_node_->create_publisher<sensor_msgs::msg::JointState>("/debug/final_joint_command_joint_frame", 10);
  final_pd_pub_ = debug_node_->create_publisher<sensor_msgs::msg::JointState>("/debug/final_pd_gains", 10);
  final_torque_ff_pub_ = debug_node_->create_publisher<sensor_msgs::msg::JointState>("/debug/final_joint_torque_ff", 10);
  j2_qd_ref_pub_ = debug_node_->create_publisher<std_msgs::msg::Float64>("/debug/j2_qd_ref", 10);
  j2_qd_actual_pub_ = debug_node_->create_publisher<std_msgs::msg::Float64>("/debug/j2_qd_actual", 10);
  laser_distance_pub_ = debug_node_->create_publisher<std_msgs::msg::UInt32>("/rc_arm_2/laser_distance", 10);
  payload_active_pub_ = debug_node_->create_publisher<std_msgs::msg::Bool>(payload_active_topic_, 10);
  j5_position_pub_ = debug_node_->create_publisher<std_msgs::msg::Float64>(j5_position_topic_, 10);
  mujoco_command_pub_ = debug_node_->create_publisher<sensor_msgs::msg::JointState>(mujoco_command_topic_, 10);
  vacuum_activate_sub_ = debug_node_->create_subscription<std_msgs::msg::Bool>(
    vacuum_activate_topic_,
    10,
    std::bind(&RsA3HardwareInterface::vacuumActivateCallback, this, std::placeholders::_1));
  payload_command_sub_ = debug_node_->create_subscription<std_msgs::msg::Bool>(
    payload_command_topic_,
    10,
    std::bind(&RsA3HardwareInterface::payloadActiveCommandCallback, this, std::placeholders::_1));
  j5_command_sub_ = debug_node_->create_subscription<std_msgs::msg::Float64>(
    j5_command_topic_,
    10,
    std::bind(&RsA3HardwareInterface::j5CommandCallback, this, std::placeholders::_1));

  if (external_feedback_enabled_) {
    external_feedback_sub_ = debug_node_->create_subscription<sensor_msgs::msg::JointState>(
      external_feedback_topic_,
      20,
      std::bind(&RsA3HardwareInterface::externalFeedbackCallback, this, std::placeholders::_1));
  }
  
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "调试发布器已创建：/debug/hw_command, /debug/smoothed_command, /debug/gravity_torque, /debug/velocity_feedforward, /debug/motor_temperature, /debug/final_joint_command, /debug/final_joint_command_joint_frame, /debug/final_pd_gains, /debug/final_joint_torque_ff, /debug/j2_qd_ref, /debug/j2_qd_actual");
  publishPayloadActiveState();

  // ============ 初始化重力补偿参数 ============
  gravity_params_.resize(num_joints);
  for (size_t i = 0; i < num_joints; ++i) {
    gravity_params_[i] = {0.0, 0.0, 0.0};  // Default to 0
  }
  
  // 从参数读取重力补偿（如果存在）
  // 格式：gravity_comp_L1_sin, gravity_comp_L1_cos, gravity_comp_L1_offset 等
  std::vector<std::string> joint_prefixes = {"L1", "L2", "L3", "L4", "L5", "L6"};
  for (size_t i = 0; i < num_joints && i < joint_prefixes.size(); ++i) {
    std::string prefix = "gravity_comp_" + joint_prefixes[i];
    if (info_.hardware_parameters.count(prefix + "_sin")) {
      gravity_params_[i].sin_coeff = std::stod(info_.hardware_parameters.at(prefix + "_sin"));
    }
    if (info_.hardware_parameters.count(prefix + "_cos")) {
      gravity_params_[i].cos_coeff = std::stod(info_.hardware_parameters.at(prefix + "_cos"));
    }
    if (info_.hardware_parameters.count(prefix + "_offset")) {
      gravity_params_[i].offset = std::stod(info_.hardware_parameters.at(prefix + "_offset"));
    }
  }
  
  // 检查是否存在非零重力补偿参数
  for (size_t i = 0; i < num_joints; ++i) {
    if (gravity_params_[i].sin_coeff != 0.0 || 
        gravity_params_[i].cos_coeff != 0.0 || 
        gravity_params_[i].offset != 0.0) {
      gravity_comp_enabled_ = true;
      break;
    }
  }
  
  if (gravity_comp_enabled_) {
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "重力补偿已启用，参数如下：");
    for (size_t i = 0; i < num_joints && i < joint_prefixes.size(); ++i) {
      RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                  "  %s: sin=%.4f, cos=%.4f, offset=%.4f",
                  joint_prefixes[i].c_str(),
                  gravity_params_[i].sin_coeff,
                  gravity_params_[i].cos_coeff,
                  gravity_params_[i].offset);
    }
  }
  
  // 读取零力矩模式的 Kd 参数
  if (info_.hardware_parameters.count("zero_torque_kd")) {
    zero_torque_kd_ = std::stod(info_.hardware_parameters.at("zero_torque_kd"));
  }
  
  // ============ Pinocchio 动力学模型初始化 ============
  if (!urdf_path_.empty()) {
    if (initPinocchioModel(urdf_path_)) {
      // 读取惯量缩放因子（用于标定微调）
      inertia_scale_params_.resize(joint_configs_.size());
      for (size_t i = 0; i < joint_configs_.size(); ++i) {
        inertia_scale_params_[i].mass_scale = 1.0;
        inertia_scale_params_[i].com_x_offset = 0.0;
        inertia_scale_params_[i].com_y_offset = 0.0;
        inertia_scale_params_[i].com_z_offset = 0.0;

        std::string prefix = "inertia_scale_L" + std::to_string(i + 1);
        if (info_.hardware_parameters.count(prefix + "_mass")) {
          inertia_scale_params_[i].mass_scale = std::stod(info_.hardware_parameters.at(prefix + "_mass"));
        }
        if (info_.hardware_parameters.count(prefix + "_com_x")) {
          inertia_scale_params_[i].com_x_offset = std::stod(info_.hardware_parameters.at(prefix + "_com_x"));
        }
        if (info_.hardware_parameters.count(prefix + "_com_y")) {
          inertia_scale_params_[i].com_y_offset = std::stod(info_.hardware_parameters.at(prefix + "_com_y"));
        }
        if (info_.hardware_parameters.count(prefix + "_com_z")) {
          inertia_scale_params_[i].com_z_offset = std::stod(info_.hardware_parameters.at(prefix + "_com_z"));
        }
      }

      // 读取惯量配置文件路径
      if (info_.hardware_parameters.count("inertia_config_path")) {
        inertia_config_path_ = info_.hardware_parameters.at("inertia_config_path");

        // 加载标定后的惯量参数
        if (loadCalibratedInertia(inertia_config_path_)) {
          // 应用到 Pinocchio 模型
          applyCalibratedInertiaToModel();
          use_calibrated_inertia_ = true;
          RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                      "已从以下路径加载标定惯量参数：%s", inertia_config_path_.c_str());
        }
      }
      RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                  "Pinocchio 功能：gravity=%s, inverse_dynamics=%s（标定惯量：%s）",
                  use_pinocchio_gravity_ ? "启用" : "禁用",
                  use_pinocchio_inverse_dynamics_ ? "启用" : "禁用",
                  use_calibrated_inertia_ ? "是" : "否");
    } else {
      use_pinocchio_gravity_ = false;
      use_pinocchio_inverse_dynamics_ = false;
    }
  } else if (use_pinocchio_gravity_ || use_pinocchio_inverse_dynamics_) {
    RCLCPP_WARN(rclcpp::get_logger("RsA3HardwareInterface"),
                "已请求 Pinocchio 功能，但未提供 urdf_path，相关功能将被禁用");
    use_pinocchio_gravity_ = false;
    use_pinocchio_inverse_dynamics_ = false;
  }

  // 创建零力矩模式服务
  zero_torque_srv_ = debug_node_->create_service<std_srvs::srv::SetBool>(
    "/rc_arm/set_zero_torque_mode",
    std::bind(&RsA3HardwareInterface::zeroTorqueModeCallback, this,
              std::placeholders::_1, std::placeholders::_2));
  
  // 启动独立线程处理服务回调
  spin_thread_running_ = true;
  spin_thread_ = std::thread([this]() {
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "调试节点 spin 线程已启动");
    while (spin_thread_running_ && rclcpp::ok()) {
      rclcpp::spin_some(debug_node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 100Hz
    }
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "调试节点 spin 线程已停止");
  });
  
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "零力矩模式服务已创建：/rc_arm/set_zero_torque_mode（Kd=%.1f）", zero_torque_kd_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

bool RsA3HardwareInterface::parseJointConfig(const hardware_interface::HardwareInfo& info)
{
  joint_configs_.clear();
  
  for (const auto& joint : info.joints) {
    JointConfig config;
    config.name = joint.name;
    
    // Parse motor_id
    if (joint.parameters.count("motor_id")) {
      config.motor_id = std::stoi(joint.parameters.at("motor_id"));
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("RsA3HardwareInterface"),
                   "关节 %s 缺少 motor_id 参数", joint.name.c_str());
      return false;
    }
    
    // Parse motor_type
    if (joint.parameters.count("motor_type")) {
      std::string type_str = joint.parameters.at("motor_type");
      if (type_str == "RS00") {
        config.motor_type = MotorType::RS00;
      } else if (type_str == "EL05") {
        config.motor_type = MotorType::EL05;
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("RsA3HardwareInterface"),
                     "关节 %s 的 motor_type 未知：%s", joint.name.c_str(), type_str.c_str());
        return false;
      }
    } else {
      // Default based on motor_id: 1-3 = RS00, 4-6 = EL05
      config.motor_type = (config.motor_id <= 3) ? MotorType::RS00 : MotorType::EL05;
    }
    
    // Parse offset
    if (joint.parameters.count("position_offset")) {
      config.position_offset = std::stod(joint.parameters.at("position_offset"));
    } else {
      config.position_offset = 0.0;
    }
    
    // Parse direction
    if (joint.parameters.count("direction")) {
      config.direction = std::stod(joint.parameters.at("direction"));
    } else {
      config.direction = 1.0;
    }
    
    // Parse joint limits (from parameters or command_interface)
    config.lower_limit = -6.28;  // Default ±360°
    config.upper_limit = 6.28;
    
    if (joint.parameters.count("lower_limit")) {
      config.lower_limit = std::stod(joint.parameters.at("lower_limit"));
    }
    if (joint.parameters.count("upper_limit")) {
      config.upper_limit = std::stod(joint.parameters.at("upper_limit"));
    }
    
    // Try to get limits from command_interface (ros2_control standard method)
    for (const auto& cmd_if : joint.command_interfaces) {
      if (cmd_if.name == "position") {
        if (cmd_if.min != cmd_if.max) {  // Valid limits
          config.lower_limit = std::stod(cmd_if.min);
          config.upper_limit = std::stod(cmd_if.max);
        }
        break;
      }
    }
    
    // Parse velocity limit
    config.velocity_limit = std::max(0.1, velocity_limit_);
    for (const auto& cmd_if : joint.command_interfaces) {
      if (cmd_if.name == "velocity") {
        try {
          const double vmin = std::stod(cmd_if.min);
          const double vmax = std::stod(cmd_if.max);
          const double vlim = std::max(std::abs(vmin), std::abs(vmax));
          if (std::isfinite(vlim) && vlim > 1e-6) {
            config.velocity_limit = vlim;
          }
        } catch (...) {
          // ignore malformed limits
        }
        break;
      }
    }

    // Continuous joint heuristic: span >= 2*pi means unwrap needed.
    const double span = config.upper_limit - config.lower_limit;
    config.is_continuous = std::isfinite(span) && span >= (2.0 * M_PI - 1e-3);

    // Parse joint-specific unloaded/payload Kp/Kd (0 means use global value)
    config.unloaded_kp = joint.parameters.count("unloaded_kp")
      ? std::stod(joint.parameters.at("unloaded_kp"))
      : (joint.parameters.count("kp") ? std::stod(joint.parameters.at("kp")) : 0.0);
    config.unloaded_kd = joint.parameters.count("unloaded_kd")
      ? std::stod(joint.parameters.at("unloaded_kd"))
      : (joint.parameters.count("kd") ? std::stod(joint.parameters.at("kd")) : 0.0);
    config.payload_kp = joint.parameters.count("payload_kp")
      ? std::stod(joint.parameters.at("payload_kp"))
      : config.unloaded_kp;
    config.payload_kd = joint.parameters.count("payload_kd")
      ? std::stod(joint.parameters.at("payload_kd"))
      : config.unloaded_kd;

    config.unloaded_low_stiffness_kp = joint.parameters.count("unloaded_low_stiffness_kp")
      ? std::stod(joint.parameters.at("unloaded_low_stiffness_kp"))
      : (joint.parameters.count("low_stiffness_kp") ? std::stod(joint.parameters.at("low_stiffness_kp")) : 0.0);
    config.unloaded_low_stiffness_kd = joint.parameters.count("unloaded_low_stiffness_kd")
      ? std::stod(joint.parameters.at("unloaded_low_stiffness_kd"))
      : (joint.parameters.count("low_stiffness_kd") ? std::stod(joint.parameters.at("low_stiffness_kd")) : 0.0);
    config.payload_low_stiffness_kp = joint.parameters.count("payload_low_stiffness_kp")
      ? std::stod(joint.parameters.at("payload_low_stiffness_kp"))
      : config.unloaded_low_stiffness_kp;
    config.payload_low_stiffness_kd = joint.parameters.count("payload_low_stiffness_kd")
      ? std::stod(joint.parameters.at("payload_low_stiffness_kd"))
      : config.unloaded_low_stiffness_kd;

    config.unloaded_low_stiffness_kp = std::clamp(config.unloaded_low_stiffness_kp, 0.0, 500.0);
    config.unloaded_low_stiffness_kd = std::clamp(config.unloaded_low_stiffness_kd, 0.0, 5.0);
    config.payload_low_stiffness_kp = std::clamp(config.payload_low_stiffness_kp, 0.0, 500.0);
    config.payload_low_stiffness_kd = std::clamp(config.payload_low_stiffness_kd, 0.0, 5.0);

    joint_configs_.push_back(config);

    const bool has_joint_pd = (
      config.unloaded_kp > 0.0 || config.unloaded_kd > 0.0 ||
      config.payload_kp > 0.0 || config.payload_kd > 0.0);
    const bool has_low_stiffness_pd = (
      config.unloaded_low_stiffness_kp > 0.0 || config.unloaded_low_stiffness_kd > 0.0 ||
      config.payload_low_stiffness_kp > 0.0 || config.payload_low_stiffness_kd > 0.0);

    if (has_joint_pd || has_low_stiffness_pd) {
      RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                  "关节 %s：motor_id=%d，type=%s，dir=%.1f，限位=[%.1f°~%.1f°]，空载PD[Kp=%.1f，Kd=%.1f]，带载PD[Kp=%.1f，Kd=%.1f]，空载低刚度[Kp=%.1f，Kd=%.1f]，带载低刚度[Kp=%.1f，Kd=%.1f]",
                  config.name.c_str(), config.motor_id,
                  config.motor_type == MotorType::RS00 ? "RS00" : "EL05",
                  config.direction,
                  config.lower_limit * 180.0 / M_PI, config.upper_limit * 180.0 / M_PI,
                  config.unloaded_kp, config.unloaded_kd,
                  config.payload_kp, config.payload_kd,
                  config.unloaded_low_stiffness_kp, config.unloaded_low_stiffness_kd,
                  config.payload_low_stiffness_kp, config.payload_low_stiffness_kd);
    } else {
      RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                  "关节 %s：motor_id=%d，type=%s，dir=%.1f，限位=[%.1f°~%.1f°]",
                  config.name.c_str(), config.motor_id,
                  config.motor_type == MotorType::RS00 ? "RS00" : "EL05",
                  config.direction,
                  config.lower_limit * 180.0 / M_PI, config.upper_limit * 180.0 / M_PI);
    }
  }
  
  return true;
}

hardware_interface::CallbackReturn RsA3HardwareInterface::on_configure(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (use_mock_hardware_) {
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "使用 mock 硬件：跳过 CAN 初始化");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  if (backend_mode_ == BackendMode::MUJOCO) {
    RCLCPP_INFO(
      rclcpp::get_logger("RsA3HardwareInterface"),
      "配置 MuJoCo 后端：state topic=%s, command topic=%s",
      external_feedback_topic_.c_str(),
      mujoco_command_topic_.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<dmbot_serial::MotorConfig> motor_configs;
  motor_configs.reserve(joint_configs_.size());
  for (const auto & config : joint_configs_) {
    dmbot_serial::MotorConfig motor_config{};
    motor_config.motor_id = config.motor_id;
    motor_config.master_id = static_cast<uint16_t>(0x100 + config.motor_id);
    motor_config.motor_type =
      config.motor_type == MotorType::EL05 ? dmbot_serial::MotorType::DM4340 : dmbot_serial::MotorType::DM4310;
    motor_configs.push_back(motor_config);
  }

  dm_driver_ = std::make_unique<dmbot_serial::DmMotorDriver>(
    dm_serial_number_, dm_nominal_baud_, dm_data_baud_, motor_configs);

  if (dm_startup_delay_sec_ > 0.0) {
    RCLCPP_INFO(
      rclcpp::get_logger("RsA3HardwareInterface"),
      "等待 dmbot_serial 启动 %.2f 秒，规避 USB2CANFD 过早初始化",
      dm_startup_delay_sec_);
    std::this_thread::sleep_for(std::chrono::duration<double>(dm_startup_delay_sec_));
  }

  if (!dm_driver_->connect()) {
    RCLCPP_ERROR(rclcpp::get_logger("RsA3HardwareInterface"),
                 "dmbot_serial 后端初始化失败");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("RsA3HardwareInterface"),
    "实机后端已配置：dmbot_serial(sn=%s, nom=%u, data=%u, control_mode=%d)",
    dm_serial_number_.c_str(),
    dm_nominal_baud_,
    dm_data_baud_,
    dm_control_mode_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RsA3HardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (dm_driver_) {
    dm_driver_->disconnect();
    dm_driver_.reset();
  }

  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"), "硬件资源已清理");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RsA3HardwareInterface::on_activate(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (use_mock_hardware_) {
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "Mock 硬件已激活");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  if (backend_mode_ == BackendMode::REAL && !dm_driver_) {
    RCLCPP_ERROR(rclcpp::get_logger("RsA3HardwareInterface"), "实机后端未配置");
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    const double initial_position = hw_positions_[i];
    hw_commands_positions_[i] = initial_position;
    hw_commands_velocities_[i] = 0.0;
    hw_commands_accelerations_[i] = 0.0;
    hw_commands_efforts_[i] = 0.0;
    smoothed_positions_[i] = initial_position;
    smoothed_velocities_[i] = 0.0;
    smoothed_accelerations_[i] = 0.0;
  }

  first_command_ = false;

  if (backend_mode_ == BackendMode::REAL) {
    for (int attempt = 1; attempt <= dm_enable_retry_count_; ++attempt) {
      if (attempt > 1 && dm_enable_retry_interval_sec_ > 0.0) {
        std::this_thread::sleep_for(std::chrono::duration<double>(dm_enable_retry_interval_sec_));
      }

      dm_driver_->enable();
      RCLCPP_INFO(
        rclcpp::get_logger("RsA3HardwareInterface"),
        "已发送 DM 电机使能，第 %d/%d 次",
        attempt,
        dm_enable_retry_count_);
    }
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"), "实机后端已激活");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"), "MuJoCo 后端已激活");
  }
  publishPayloadActiveState();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RsA3HardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (!use_mock_hardware_ && backend_mode_ == BackendMode::REAL && dm_driver_) {
    dm_driver_->disable();
  }

  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"), "硬件已停用");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RsA3HardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  on_deactivate(rclcpp_lifecycle::State());
  on_cleanup(rclcpp_lifecycle::State());
  
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"), "硬件已关闭");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RsA3HardwareInterface::on_error(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (!use_mock_hardware_ && backend_mode_ == BackendMode::REAL && dm_driver_) {
    dm_driver_->disable();
  }

  RCLCPP_ERROR(rclcpp::get_logger("RsA3HardwareInterface"), "硬件发生错误");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RsA3HardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_configs_[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_configs_[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_configs_[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
    // Add temperature state interface
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_configs_[i].name, "temperature", &hw_temperatures_[i]));
  }
  
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RsA3HardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint_configs_[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint_configs_[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint_configs_[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_commands_accelerations_[i]));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint_configs_[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_efforts_[i]));
  }
  
  return command_interfaces;
}

void RsA3HardwareInterface::externalFeedbackCallback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  bool got_any_position = false;
  {
    std::lock_guard<std::mutex> lock(external_feedback_mutex_);

    const size_t joint_count = joint_configs_.size();
    const bool has_name_mapping = !msg->name.empty();

    for (size_t joint_idx = 0; joint_idx < joint_count; ++joint_idx) {
      size_t msg_idx = joint_idx;

      if (has_name_mapping) {
        const auto it = std::find(msg->name.begin(), msg->name.end(), joint_configs_[joint_idx].name);
        if (it == msg->name.end()) {
          continue;
        }
        msg_idx = static_cast<size_t>(std::distance(msg->name.begin(), it));
      }

      if (msg_idx < msg->position.size()) {
        external_feedback_positions_[joint_idx] = msg->position[msg_idx];
        got_any_position = true;
      }
      if (msg_idx < msg->velocity.size()) {
        external_feedback_velocities_[joint_idx] = msg->velocity[msg_idx];
      }
      if (msg_idx < msg->effort.size()) {
        external_feedback_efforts_[joint_idx] = msg->effort[msg_idx];
      }
    }

    if (got_any_position || !msg->velocity.empty() || !msg->effort.empty()) {
      external_feedback_last_time_ = std::chrono::steady_clock::now();
      external_feedback_received_.store(true);
    }
  }
}

void RsA3HardwareInterface::j5CommandCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  latest_j5_command_ = msg->data;
  j5_command_received_ = true;
}


hardware_interface::return_type RsA3HardwareInterface::read(
  const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  if (!use_mock_hardware_ && backend_mode_ == BackendMode::REAL && dm_driver_) {
    const auto motor_states = dm_driver_->readStates();
    const uint32_t laser_distance = dm_driver_->readLaserDistance();
    latest_j5_position_ = dm_driver_->readJ5Position();
    for (size_t i = 0; i < joint_configs_.size() && i < motor_states.size(); ++i) {
      const auto & config = joint_configs_[i];
      const auto & state = motor_states[i];
      if (!state.valid) {
        continue;
      }
      hw_positions_[i] = (state.position - config.position_offset) * config.direction;
      hw_velocities_[i] = state.velocity * config.direction;
      hw_efforts_[i] = state.effort * config.direction;
      hw_temperatures_[i] = 25.0;
    }

    if (laser_distance_pub_) {
      std_msgs::msg::UInt32 msg;
      msg.data = laser_distance;
      laser_distance_pub_->publish(msg);
    }
    if (j5_position_pub_) {
      std_msgs::msg::Float64 msg;
      msg.data = latest_j5_position_;
      j5_position_pub_->publish(msg);
    }
  } else {
    bool used_external_feedback = false;

    if (external_feedback_enabled_ && external_feedback_received_.load()) {
      const auto now = std::chrono::steady_clock::now();
      std::lock_guard<std::mutex> lock(external_feedback_mutex_);
      const double feedback_age = std::chrono::duration<double>(now - external_feedback_last_time_).count();
      if (feedback_age <= external_feedback_timeout_sec_) {
        for (size_t i = 0; i < joint_configs_.size(); ++i) {
          hw_positions_[i] = external_feedback_positions_[i];
          hw_velocities_[i] = external_feedback_velocities_[i];
          hw_efforts_[i] = external_feedback_efforts_[i];
          hw_temperatures_[i] = 25.0;
        }
        used_external_feedback = true;
      }
    }

    if (!used_external_feedback) {
      // 无外部反馈时回退到内部平滑状态
      for (size_t i = 0; i < joint_configs_.size(); ++i) {
        hw_positions_[i] = smoothed_positions_[i];
        hw_velocities_[i] = smoothed_velocities_[i];
        hw_efforts_[i] = final_cmd_efforts_[i];
        hw_temperatures_[i] = 25.0;  // Mock 温度
      }
    }
  }
  
  // 发布温度数据（降频：每 50 次 read 发布一次）
  static int temp_pub_counter = 0;
  if (++temp_pub_counter >= 50) {
    temp_pub_counter = 0;
    sensor_msgs::msg::JointState temp_msg;
    temp_msg.header.stamp = debug_node_->now();
    for (size_t i = 0; i < joint_configs_.size(); ++i) {
      temp_msg.name.push_back(joint_configs_[i].name);
      temp_msg.effort.push_back(hw_temperatures_[i]);  // 使用 effort 字段存储温度
    }
    temperature_pub_->publish(temp_msg);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RsA3HardwareInterface::write(
  const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
  static int write_counter = 0;
  static std::vector<double> last_tau_for_spike_check;

  if (use_mock_hardware_) {
    return hardware_interface::return_type::OK;
  }

  const bool real_backend_ready =
    backend_mode_ == BackendMode::REAL && dm_driver_ && dm_driver_->isConnected();
  if (backend_mode_ == BackendMode::REAL && !real_backend_ready) {
    return hardware_interface::return_type::ERROR;
  }

  double dt = period.seconds();
  if (dt <= 0.0 || dt > 0.1) {
    dt = fallback_control_period_;
  }

  if (first_command_) {
    for (size_t i = 0; i < joint_configs_.size(); ++i) {
      smoothed_positions_[i] = hw_commands_positions_[i];
      smoothed_velocities_[i] = 0.0;
      smoothed_accelerations_[i] = 0.0;

      last_cmd_positions_[i] = hw_commands_positions_[i];
      filtered_cmd_velocities_[i] = 0.0;
      velocity_ff_stage2_[i] = 0.0;
    }

    first_command_ = false;
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "收到首条指令，初始化执行参考（直接跟随上层 position/velocity/acceleration）");
  }

  const bool payload_active = payload_active_.load();
  const ControlGainSet & active_gains = payload_active ? payload_gains_ : unloaded_gains_;

  std::vector<double> cmd_positions_motor(joint_configs_.size(), 0.0);
  std::vector<double> cmd_velocities_motor(joint_configs_.size(), 0.0);
  std::vector<double> final_motor_positions(joint_configs_.size(), 0.0);
  std::vector<double> final_motor_velocities(joint_configs_.size(), 0.0);
  std::vector<double> final_motor_efforts(joint_configs_.size(), 0.0);

  std::vector<double> id_ref_positions = smoothed_positions_;
  std::vector<double> id_ref_velocities = smoothed_velocities_;
  std::vector<double> id_ref_accelerations = smoothed_accelerations_;

  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    const auto& config = joint_configs_[i];

    const double target_position = hw_commands_positions_[i];
    const double prev_position = smoothed_positions_[i];
    const double prev_velocity = smoothed_velocities_[i];

    double new_position = target_position;
    double new_velocity = (new_position - prev_position) / dt;
    double new_acceleration = 0.0;

    if (std::isfinite(hw_commands_velocities_[i])) {
      new_velocity = std::clamp(hw_commands_velocities_[i], -config.velocity_limit, config.velocity_limit);
    }
    if (std::isfinite(hw_commands_accelerations_[i])) {
      new_acceleration = hw_commands_accelerations_[i];
    } else {
      new_acceleration = (new_velocity - prev_velocity) / dt;
    }

    if (std::abs(new_velocity) > config.velocity_limit * 1.05) {
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("RsA3HardwareInterface"),
        *debug_node_->get_clock(),
        2000,
        "关节 %s 速度超限：|v|=%.4f, limit=%.4f",
        config.name.c_str(),
        std::abs(new_velocity),
        config.velocity_limit);
    }
    if (std::abs(new_acceleration) > max_acceleration_ * 1.10) {
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("RsA3HardwareInterface"),
        *debug_node_->get_clock(),
        2000,
        "关节 %s 加速度超限：|a|=%.4f, limit=%.4f",
        config.name.c_str(),
        std::abs(new_acceleration),
        max_acceleration_);
    }

    const double raw_cmd_velocity = new_velocity;

    id_ref_positions[i] = new_position;
    id_ref_velocities[i] = new_velocity;
    id_ref_accelerations[i] = new_acceleration;

    last_cmd_positions_[i] = target_position;
    cmd_velocities_[i] = new_velocity;

    smoothed_positions_[i] = new_position;
    smoothed_velocities_[i] = new_velocity;
    smoothed_accelerations_[i] = new_acceleration;

    double cmd_position = smoothed_positions_[i] * config.direction + config.position_offset;

    auto params = getMotorParams(config.motor_type);
    cmd_position = std::clamp(cmd_position, params.p_min, params.p_max);

    double filtered_velocity = 0.0;
    const double velocity_threshold = 0.001;
    double cmd_velocity = (std::abs(raw_cmd_velocity) < velocity_threshold) ? 0.0 : raw_cmd_velocity;
    const double joint_velocity_limit = std::max(0.1, config.velocity_limit);
    cmd_velocity = std::clamp(cmd_velocity, -joint_velocity_limit, joint_velocity_limit);

    const double alpha1 = 0.1;
    const double first_stage = alpha1 * cmd_velocity + (1.0 - alpha1) * filtered_cmd_velocities_[i];
    filtered_cmd_velocities_[i] = first_stage;

    const double alpha2 = 0.15;
    filtered_velocity = alpha2 * first_stage + (1.0 - alpha2) * velocity_ff_stage2_[i];

    const double max_velocity_change = max_acceleration_ * dt;
    const double velocity_change = filtered_velocity - velocity_ff_stage2_[i];
    if (std::abs(velocity_change) > max_velocity_change) {
      filtered_velocity = velocity_ff_stage2_[i] +
                          max_velocity_change * (velocity_change > 0 ? 1.0 : -1.0);
    }

    if (std::abs(filtered_velocity) < 0.005) {
      filtered_velocity = 0.0;
    }

    velocity_ff_stage2_[i] = filtered_velocity;
    cmd_positions_motor[i] = cmd_position;
    cmd_velocities_motor[i] = velocity_ff_stage2_[i] * config.direction;
  }

  std::vector<double> pinocchio_gravity_torques_actual;
  if (pinocchio_initialized_ && (use_pinocchio_gravity_ || use_pinocchio_inverse_dynamics_)) {
    pinocchio_gravity_torques_actual = computePinocchioGravity(hw_positions_, payload_active);
  }

  std::vector<double> pinocchio_id_torques;
  if (use_pinocchio_inverse_dynamics_ && pinocchio_initialized_) {
    pinocchio_id_torques = computePinocchioInverseDynamics(
      id_ref_positions, id_ref_velocities, id_ref_accelerations, payload_active);
  }
  std::vector<double> pinocchio_gravity_torques_ref;
  if (use_pinocchio_inverse_dynamics_ && pinocchio_initialized_) {
    pinocchio_gravity_torques_ref = computePinocchioGravity(id_ref_positions, payload_active);
  }
  std::vector<double> pinocchio_dynamic_only_torques(joint_configs_.size(), 0.0);
  if (pinocchio_initialized_) {
    for (size_t i = 0; i < joint_configs_.size(); ++i) {
      if (i < pinocchio_id_torques.size() && i < pinocchio_gravity_torques_ref.size()) {
        pinocchio_dynamic_only_torques[i] = pinocchio_id_torques[i] - pinocchio_gravity_torques_ref[i];
      }
    }
  }

  if (last_tau_for_spike_check.size() != joint_configs_.size()) {
    last_tau_for_spike_check.assign(joint_configs_.size(), 0.0);
  }

  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    const auto& config = joint_configs_[i];
    auto params = getMotorParams(config.motor_type);

    double gravity_torque = 0.0;
    if (gravity_comp_enabled_ || (use_pinocchio_gravity_ && pinocchio_initialized_)) {
      if (use_pinocchio_gravity_ && pinocchio_initialized_ && i < pinocchio_gravity_torques_actual.size()) {
        gravity_torque = pinocchio_gravity_torques_actual[i] * gravity_feedforward_ratio_;
      } else {
        gravity_torque = computeGravityTorque(i, hw_positions_[i]) * gravity_feedforward_ratio_;
      }
    }

    double model_feedforward_torque = gravity_torque;
    if (use_pinocchio_inverse_dynamics_ && pinocchio_initialized_ && i < pinocchio_id_torques.size()) {
      model_feedforward_torque = pinocchio_id_torques[i];
      if (i < pinocchio_gravity_torques_ref.size()) {
        model_feedforward_torque -= (1.0 - gravity_feedforward_ratio_) * pinocchio_gravity_torques_ref[i];
      }
    }

    const double active_low_joint_kp = payload_active
      ? config.payload_low_stiffness_kp
      : config.unloaded_low_stiffness_kp;
    const double active_low_joint_kd = payload_active
      ? config.payload_low_stiffness_kd
      : config.unloaded_low_stiffness_kd;
    const double low_joint_kp = std::clamp(
      (active_low_joint_kp > 0.0) ? active_low_joint_kp : active_gains.low_stiffness_kp, 0.0, 500.0);
    const double low_joint_kd = std::clamp(
      (active_low_joint_kd > 0.0) ? active_low_joint_kd : active_gains.low_stiffness_kd, 0.0, 5.0);

    double motor_kp = 0.0;
    double motor_kd = 0.0;
    double joint_cmd_torque = 0.0;
    double final_cmd_position = cmd_positions_motor[i];

    if (zero_torque_mode_) {
      motor_kp = 0.0;
      motor_kd = std::clamp(zero_torque_kd_, 0.0, 5.0);

      if ((use_pinocchio_gravity_ || use_pinocchio_inverse_dynamics_) &&
          pinocchio_initialized_ &&
          i < pinocchio_gravity_torques_actual.size()) {
        joint_cmd_torque = pinocchio_gravity_torques_actual[i];
      } else {
        joint_cmd_torque = computeGravityTorque(i, hw_positions_[i]);
      }

      final_cmd_position = hw_positions_[i] * config.direction + config.position_offset;
    } else if (low_stiffness_mode_) {
      motor_kp = low_joint_kp;
      motor_kd = low_joint_kd;
      joint_cmd_torque = model_feedforward_torque + active_gains.low_stiffness_torque_bias;
    } else {
      const double active_joint_kp = payload_active ? config.payload_kp : config.unloaded_kp;
      const double active_joint_kd = payload_active ? config.payload_kd : config.unloaded_kd;
      const double joint_kp = (active_joint_kp > 0.0) ? active_joint_kp : active_gains.position_kp;
      const double joint_kd = (active_joint_kd > 0.0) ? active_joint_kd : active_gains.position_kd;
      motor_kp = std::clamp(joint_kp, 0.0, 500.0);
      motor_kd = std::clamp(joint_kd, 0.0, 5.0);
      joint_cmd_torque = model_feedforward_torque;
    }

    joint_cmd_torque += hw_commands_efforts_[i];

    joint_cmd_torque = std::clamp(joint_cmd_torque, params.t_min, params.t_max);

    const double tau_jump = std::abs(joint_cmd_torque - last_tau_for_spike_check[i]);
    if (tau_jump > 4.0) {
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("RsA3HardwareInterface"),
        *debug_node_->get_clock(),
        2000,
        "关节 %s 力矩跳变较大：|Δtau|=%.4f",
        config.name.c_str(),
        tau_jump);
    }
    last_tau_for_spike_check[i] = joint_cmd_torque;

    const double final_cmd_velocity = zero_torque_mode_ ? 0.0 : cmd_velocities_motor[i];
    const double final_motor_torque = joint_cmd_torque * config.direction;

    final_cmd_positions_[i] = (final_cmd_position - config.position_offset) * config.direction;
    final_cmd_velocities_[i] = final_cmd_velocity * config.direction;
    final_cmd_efforts_[i] = joint_cmd_torque;
    final_cmd_kps_[i] = motor_kp;
    final_cmd_kds_[i] = motor_kd;
    final_cmd_torque_ff_[i] = joint_cmd_torque;

    final_motor_positions[i] = final_cmd_position;
    final_motor_velocities[i] = final_cmd_velocity;
    final_motor_efforts[i] = final_motor_torque;
  }

  if (backend_mode_ == BackendMode::REAL && real_backend_ready) {
    if (!dm_driver_->writeCommands(
          final_motor_positions,
          final_motor_velocities,
          final_cmd_kps_,
          final_cmd_kds_,
          final_motor_efforts)) {
      static int warn_counter = 0;
      if (warn_counter++ % 1000 == 0) {
        RCLCPP_WARN(rclcpp::get_logger("RsA3HardwareInterface"),
                    "向 dmbot_serial 后端发送控制包失败");
      }
    }
    if (j5_command_received_ && !dm_driver_->writeJ5Command(latest_j5_command_, j5_kp_, j5_kd_)) {
      static int j5_warn_counter = 0;
      if (j5_warn_counter++ % 1000 == 0) {
        RCLCPP_WARN(rclcpp::get_logger("RsA3HardwareInterface"),
                    "向 dmbot_serial 后端发送 J5 控制包失败");
      }
    }
  } else if (backend_mode_ == BackendMode::MUJOCO && mujoco_command_pub_ && debug_node_) {
    sensor_msgs::msg::JointState mujoco_cmd_msg;
    mujoco_cmd_msg.header.stamp = debug_node_->get_clock()->now();
    for (const auto & config : joint_configs_) {
      mujoco_cmd_msg.name.push_back(config.name);
    }
    mujoco_cmd_msg.position = final_cmd_positions_;
    mujoco_cmd_msg.velocity = final_cmd_velocities_;
    mujoco_cmd_msg.effort = final_cmd_efforts_;
    mujoco_command_pub_->publish(mujoco_cmd_msg);
  }

  if (write_counter % 10 == 0 && debug_node_ && hw_cmd_pub_ && smoothed_cmd_pub_) {
    auto now = debug_node_->get_clock()->now();

    sensor_msgs::msg::JointState hw_cmd_msg;
    hw_cmd_msg.header.stamp = now;
    for (const auto& config : joint_configs_) {
      hw_cmd_msg.name.push_back(config.name);
    }
    hw_cmd_msg.position = hw_commands_positions_;
    hw_cmd_msg.velocity = hw_commands_velocities_;
    hw_cmd_msg.effort = hw_commands_efforts_;
    hw_cmd_pub_->publish(hw_cmd_msg);

    sensor_msgs::msg::JointState smoothed_msg;
    smoothed_msg.header.stamp = now;
    smoothed_msg.name = hw_cmd_msg.name;
    smoothed_msg.position = smoothed_positions_;
    smoothed_msg.velocity = smoothed_velocities_;
    smoothed_msg.effort = smoothed_accelerations_;
    smoothed_cmd_pub_->publish(smoothed_msg);

    if (gravity_comp_enabled_ && gravity_torque_pub_) {
      sensor_msgs::msg::JointState gravity_msg;
      gravity_msg.header.stamp = now;
      gravity_msg.name = hw_cmd_msg.name;
      for (size_t i = 0; i < joint_configs_.size(); ++i) {
        gravity_msg.effort.push_back(computeGravityTorque(i, hw_positions_[i]));
      }
      gravity_torque_pub_->publish(gravity_msg);
    }
  }

  if (velocity_ff_pub_ && debug_node_) {
    auto now = debug_node_->get_clock()->now();
    sensor_msgs::msg::JointState velocity_ff_msg;
    velocity_ff_msg.header.stamp = now;
    for (const auto& config : joint_configs_) {
      velocity_ff_msg.name.push_back(config.name);
    }
    velocity_ff_msg.velocity = velocity_ff_stage2_;
    velocity_ff_pub_->publish(velocity_ff_msg);

    if (j2_qd_ref_pub_ && j2_qd_actual_pub_) {
      std_msgs::msg::Float64 qd_ref_msg;
      std_msgs::msg::Float64 qd_actual_msg;
      if (smoothed_velocities_.size() > 1) {
        qd_ref_msg.data = smoothed_velocities_[1];
      } else {
        qd_ref_msg.data = 0.0;
      }
      if (hw_velocities_.size() > 1) {
        qd_actual_msg.data = hw_velocities_[1];
      } else {
        qd_actual_msg.data = 0.0;
      }
      j2_qd_ref_pub_->publish(qd_ref_msg);
      j2_qd_actual_pub_->publish(qd_actual_msg);
    }
  }

  if (debug_node_ && final_cmd_pub_ && final_cmd_joint_frame_pub_ && final_pd_pub_ &&
      final_torque_ff_pub_) {
    auto now = debug_node_->get_clock()->now();

    sensor_msgs::msg::JointState final_cmd_msg;
    final_cmd_msg.header.stamp = now;
    sensor_msgs::msg::JointState final_pd_msg;
    final_pd_msg.header.stamp = now;
    sensor_msgs::msg::JointState final_torque_ff_msg;
    final_torque_ff_msg.header.stamp = now;

    for (const auto& config : joint_configs_) {
      final_cmd_msg.name.push_back(config.name);
      final_pd_msg.name.push_back(config.name);
      final_torque_ff_msg.name.push_back(config.name);
    }

    final_cmd_msg.position = final_motor_positions;
    final_cmd_msg.velocity = final_motor_velocities;
    final_cmd_msg.effort = final_motor_efforts;

    sensor_msgs::msg::JointState final_cmd_joint_frame_msg;
    final_cmd_joint_frame_msg.header.stamp = now;
    final_cmd_joint_frame_msg.name = final_cmd_msg.name;
    final_cmd_joint_frame_msg.position = final_cmd_positions_;
    final_cmd_joint_frame_msg.velocity = final_cmd_velocities_;
    final_cmd_joint_frame_msg.effort = final_cmd_efforts_;

    final_pd_msg.position = final_cmd_kps_;
    final_pd_msg.velocity = final_cmd_kds_;

    final_torque_ff_msg.effort = final_cmd_torque_ff_;

    final_cmd_pub_->publish(final_cmd_msg);
    final_cmd_joint_frame_pub_->publish(final_cmd_joint_frame_msg);
    final_pd_pub_->publish(final_pd_msg);
    final_torque_ff_pub_->publish(final_torque_ff_msg);
  }

  write_counter++;
  return hardware_interface::return_type::OK;
}
double RsA3HardwareInterface::computeGravityTorque(size_t joint_idx, double position)
{
  if (joint_idx >= gravity_params_.size()) {
    return 0.0;
  }
  
  const auto& params = gravity_params_[joint_idx];
  return params.sin_coeff * std::sin(position)
       + params.cos_coeff * std::cos(position)
       + params.offset;
}

void RsA3HardwareInterface::zeroTorqueModeCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  zero_torque_mode_ = request->data;
  
  if (zero_torque_mode_) {
    RCLCPP_WARN(rclcpp::get_logger("RsA3HardwareInterface"),
                "零力矩模式已启用！Kp=0，Kd=%.1f，重力补偿=%s",
                zero_torque_kd_, gravity_comp_enabled_ ? "开启" : "关闭");
    response->message = "零力矩模式已启用：机械臂可手动拖动";
  } else {
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "零力矩模式已关闭：恢复位置控制");
    response->message = "零力矩模式已关闭：位置控制生效";
  }
  
  response->success = true;
}

void RsA3HardwareInterface::publishPayloadActiveState()
{
  if (!payload_active_pub_ || !debug_node_) {
    return;
  }
  std_msgs::msg::Bool msg;
  msg.data = payload_active_.load();
  payload_active_pub_->publish(msg);
}

void RsA3HardwareInterface::vacuumActivateCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  if (backend_mode_ == BackendMode::REAL && dm_driver_) {
    const bool ok = msg->data ? dm_driver_->enableVacuum() : dm_driver_->disableVacuum();
    if (!ok) {
      RCLCPP_WARN(
        rclcpp::get_logger("RsA3HardwareInterface"),
        "真空命令已接收，但当前 dmbot_serial 后端未就绪");
    }
  }
  RCLCPP_INFO(
    rclcpp::get_logger("RsA3HardwareInterface"),
    "vacuum -> %s (vacuum topic=%s)",
    msg->data ? "true" : "false",
    vacuum_activate_topic_.c_str());
}

void RsA3HardwareInterface::payloadActiveCommandCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  const bool previous = payload_active_.exchange(msg->data);
  publishPayloadActiveState();
  if (previous != msg->data) {
    RCLCPP_INFO(
      rclcpp::get_logger("RsA3HardwareInterface"),
      "payload_active -> %s (payload command topic=%s)",
      msg->data ? "true" : "false",
      payload_command_topic_.c_str());
  }
}

double RsA3HardwareInterface::computeLimitProtectionFactor(
  size_t joint_idx, double current_pos, double target_pos)
{
  if (joint_idx >= joint_configs_.size()) {
    return 1.0;
  }
  
  const auto& config = joint_configs_[joint_idx];
  double lower = config.lower_limit;
  double upper = config.upper_limit;
  
  // Calculate distance to boundary
  double dist_to_lower = current_pos - lower;
  double dist_to_upper = upper - current_pos;
  
  // Determine motion direction
  double motion_dir = target_pos - current_pos;
  
  // Select relevant boundary distance
  double relevant_dist;
  if (motion_dir < 0) {
    // Moving toward lower limit
    relevant_dist = dist_to_lower;
  } else if (motion_dir > 0) {
    // Moving toward upper limit
    relevant_dist = dist_to_upper;
  } else {
    return 1.0;  // No motion
  }
  
  // If within limit boundary, compute deceleration factor
  if (relevant_dist < limit_margin_) {
    if (relevant_dist <= limit_stop_margin_) {
      // Hard stop zone
      return 0.0;
    }
    // Linear deceleration zone: from limit_decel_factor_ to 1.0
    double ratio = (relevant_dist - limit_stop_margin_) / (limit_margin_ - limit_stop_margin_);
    return limit_decel_factor_ + (1.0 - limit_decel_factor_) * ratio;
  }
  
  return 1.0;
}

bool RsA3HardwareInterface::applyJointLimitProtection(size_t joint_idx, double& target_pos)
{
  if (joint_idx >= joint_configs_.size()) {
    return false;
  }
  
  const auto& config = joint_configs_[joint_idx];
  double lower = config.lower_limit;
  double upper = config.upper_limit;
  
  bool hit_limit = false;
  
  // Check and clamp target position
  if (target_pos < lower + limit_stop_margin_) {
    target_pos = lower + limit_stop_margin_;
    hit_limit = true;
  } else if (target_pos > upper - limit_stop_margin_) {
    target_pos = upper - limit_stop_margin_;
    hit_limit = true;
  }
  
  // Joint limit warning (print every 100 triggers to prevent log flooding)
  if (hit_limit) {
    if (!joint_at_limit_[joint_idx]) {
      joint_at_limit_[joint_idx] = true;
      RCLCPP_WARN(rclcpp::get_logger("RsA3HardwareInterface"),
                  "⚠️ 关节 %s 达到限位！pos=%.3f rad（%.1f°），limits=[%.2f, %.2f]",
                  config.name.c_str(), target_pos, target_pos * 180.0 / M_PI,
                  lower, upper);
    }
    limit_warn_counter_[joint_idx]++;
    if (limit_warn_counter_[joint_idx] % 500 == 0) {
      RCLCPP_WARN(rclcpp::get_logger("RsA3HardwareInterface"),
                  "关节 %s 仍处于限位区（count=%d）",
                  config.name.c_str(), limit_warn_counter_[joint_idx]);
    }
  } else {
    if (joint_at_limit_[joint_idx]) {
      joint_at_limit_[joint_idx] = false;
      limit_warn_counter_[joint_idx] = 0;
      RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                  "✓ 关节 %s 已离开限位区", config.name.c_str());
    }
  }
  
  return hit_limit;
}

// ============ Pinocchio dynamics function implementation ============

bool RsA3HardwareInterface::initPinocchioModel(const std::string& urdf_path)
{
  try {
    pinocchio::urdf::buildModel(urdf_path, pinocchio_model_);
    pinocchio_data_ = pinocchio::Data(pinocchio_model_);
    pinocchio_loaded_model_ = pinocchio_model_;
    pinocchio_loaded_data_ = pinocchio::Data(pinocchio_loaded_model_);

    pinocchio_initialized_ = true;
    pinocchio_loaded_initialized_ = true;
    pinocchio_mapping_ready_ = false;

    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "Pinocchio 模型初始化成功：");
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "  - 模型名称：%s", pinocchio_model_.name.c_str());
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "  - 关节数量：%d", pinocchio_model_.njoints);
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "  - 自由度数量：%d", pinocchio_model_.nv);

    for (int i = 1; i < pinocchio_model_.njoints; ++i) {
      RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                  "  - 关节 %d：%s", i, pinocchio_model_.names[i].c_str());
    }

    if (!buildPinocchioJointMapping()) {
      RCLCPP_ERROR(rclcpp::get_logger("RsA3HardwareInterface"),
                   "Pinocchio 关节映射构建失败，将禁用 Pinocchio 相关输出");
      pinocchio_initialized_ = false;
      pinocchio_mapping_ready_ = false;
      return false;
    }

    configureLoadedPinocchioModel();

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("RsA3HardwareInterface"),
                 "从 %s 初始化 Pinocchio 模型失败：%s",
                 urdf_path.c_str(), e.what());
    pinocchio_initialized_ = false;
    pinocchio_mapping_ready_ = false;
    return false;
  }
}

bool RsA3HardwareInterface::buildPinocchioJointMapping()
{
  pinocchio_q_index_map_.assign(joint_configs_.size(), -1);
  pinocchio_v_index_map_.assign(joint_configs_.size(), -1);
  pinocchio_mapping_ready_ = false;

  if (!pinocchio_initialized_) {
    RCLCPP_ERROR(rclcpp::get_logger("RsA3HardwareInterface"),
                 "Pinocchio 未初始化，无法构建关节映射");
    return false;
  }

  if (pinocchio_model_.nq != pinocchio_model_.nv) {
    RCLCPP_ERROR(rclcpp::get_logger("RsA3HardwareInterface"),
                 "当前仅支持固定基座串联机械臂（要求 nq==nv），当前 nq=%d nv=%d",
                 pinocchio_model_.nq,
                 pinocchio_model_.nv);
    return false;
  }

  auto find_joint_id = [&](const std::string& hw_name) -> pinocchio::JointIndex {
    for (pinocchio::JointIndex jid = 1; jid < static_cast<pinocchio::JointIndex>(pinocchio_model_.njoints); ++jid) {
      if (pinocchio_model_.names[jid] == hw_name) {
        return jid;
      }
    }

    if (hw_name.size() > 6 && hw_name.compare(hw_name.size() - 6, 6, "_joint") == 0) {
      const std::string short_name = hw_name.substr(0, hw_name.size() - 6);
      for (pinocchio::JointIndex jid = 1; jid < static_cast<pinocchio::JointIndex>(pinocchio_model_.njoints); ++jid) {
        if (pinocchio_model_.names[jid] == short_name) {
          return jid;
        }
      }
    }

    return 0;
  };

  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    const auto& hw_joint = joint_configs_[i].name;
    const pinocchio::JointIndex jid = find_joint_id(hw_joint);
    if (jid == 0) {
      RCLCPP_ERROR(rclcpp::get_logger("RsA3HardwareInterface"),
                   "Pinocchio 映射失败：找不到关节 %s",
                   hw_joint.c_str());
      return false;
    }

    const auto& pj = pinocchio_model_.joints[jid];
    if (pj.nq() != 1 || pj.nv() != 1) {
      RCLCPP_ERROR(rclcpp::get_logger("RsA3HardwareInterface"),
                   "Pinocchio 关节 %s 不是 1-DoF（nq=%d nv=%d），当前版本不支持",
                   pinocchio_model_.names[jid].c_str(),
                   pj.nq(),
                   pj.nv());
      return false;
    }

    const int q_idx = static_cast<int>(pj.idx_q());
    const int v_idx = static_cast<int>(pj.idx_v());

    if (q_idx < 0 || q_idx >= pinocchio_model_.nq || v_idx < 0 || v_idx >= pinocchio_model_.nv) {
      RCLCPP_ERROR(rclcpp::get_logger("RsA3HardwareInterface"),
                   "Pinocchio 索引越界：关节 %s q_idx=%d v_idx=%d",
                   hw_joint.c_str(),
                   q_idx,
                   v_idx);
      return false;
    }

    pinocchio_q_index_map_[i] = q_idx;
    pinocchio_v_index_map_[i] = v_idx;

    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "Pinocchio 映射：HW[%s] -> model[%s], q[%d], v[%d]",
                hw_joint.c_str(),
                pinocchio_model_.names[jid].c_str(),
                q_idx,
                v_idx);
  }

  pinocchio_mapping_ready_ = true;
  return true;
}

void RsA3HardwareInterface::configureLoadedPinocchioModel()
{
  if (!pinocchio_initialized_) {
    return;
  }

  pinocchio_loaded_model_ = pinocchio_model_;

  if (payload_mass_ <= 0.0) {
    pinocchio_loaded_data_ = pinocchio::Data(pinocchio_loaded_model_);
    pinocchio_loaded_initialized_ = true;
    return;
  }

  if (!pinocchio_loaded_model_.existFrame(payload_frame_)) {
    RCLCPP_WARN(
      rclcpp::get_logger("RsA3HardwareInterface"),
      "Pinocchio 带载模型未找到 frame=%s，将回退到基础模型",
      payload_frame_.c_str());
    pinocchio_loaded_data_ = pinocchio::Data(pinocchio_loaded_model_);
    pinocchio_loaded_initialized_ = false;
    return;
  }

  const pinocchio::FrameIndex payload_frame_id = pinocchio_loaded_model_.getFrameId(payload_frame_);
  const auto & payload_frame = pinocchio_loaded_model_.frames[payload_frame_id];

  Eigen::Matrix3d payload_inertia_matrix = Eigen::Matrix3d::Zero();
  payload_inertia_matrix.diagonal() << payload_diaginertia_[0], payload_diaginertia_[1], payload_diaginertia_[2];

  const Eigen::Vector3d payload_com_offset(
    payload_com_offset_[0], payload_com_offset_[1], payload_com_offset_[2]);
  const pinocchio::Inertia payload_inertia(
    payload_mass_, payload_com_offset, payload_inertia_matrix);

  // The payload frame placement is expressed in the supporting joint frame.
  // Appending the inertia at that placement makes the payload follow the end effector rigidly.
  pinocchio_loaded_model_.appendBodyToJoint(
    payload_frame.parentJoint,
    payload_inertia,
    payload_frame.placement);
  pinocchio_loaded_data_ = pinocchio::Data(pinocchio_loaded_model_);
  pinocchio_loaded_initialized_ = true;

  RCLCPP_INFO(
    rclcpp::get_logger("RsA3HardwareInterface"),
    "Pinocchio 带载模型已配置：frame=%s mass=%.3f com=[%.4f %.4f %.4f] inertia=[%.4f %.4f %.4f]",
    payload_frame_.c_str(),
    payload_mass_,
    payload_com_offset_[0], payload_com_offset_[1], payload_com_offset_[2],
    payload_diaginertia_[0], payload_diaginertia_[1], payload_diaginertia_[2]);
}

std::vector<double> RsA3HardwareInterface::computePinocchioGravity(
  const std::vector<double>& positions)
{
  return computePinocchioGravity(positions, false);
}

std::vector<double> RsA3HardwareInterface::computePinocchioGravity(
  const std::vector<double>& positions,
  bool use_payload_model)
{
  std::vector<double> gravity_torques(joint_configs_.size(), 0.0);

  if (!pinocchio_initialized_ || !pinocchio_mapping_ready_ || positions.size() != joint_configs_.size()) {
    return gravity_torques;
  }

  try {
    const pinocchio::Model & model =
      (use_payload_model && pinocchio_loaded_initialized_) ? pinocchio_loaded_model_ : pinocchio_model_;
    pinocchio::Data & data =
      (use_payload_model && pinocchio_loaded_initialized_) ? pinocchio_loaded_data_ : pinocchio_data_;
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);

    for (size_t i = 0; i < joint_configs_.size(); ++i) {
      const int q_idx = pinocchio_q_index_map_[i];
      if (q_idx >= 0 && q_idx < q.size()) {
        q[q_idx] = positions[i];
      }
    }

    const Eigen::VectorXd tau = pinocchio::rnea(model, data, q, v, a);

    for (size_t i = 0; i < joint_configs_.size(); ++i) {
      const int v_idx = pinocchio_v_index_map_[i];
      if (v_idx < 0 || v_idx >= tau.size()) {
        continue;
      }

      double scale = 1.0;
      if (!use_calibrated_inertia_ && i < inertia_scale_params_.size()) {
        scale = inertia_scale_params_[i].mass_scale;
      }

      gravity_torques[i] = tau[v_idx] * scale;
    }

  } catch (const std::exception& e) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("RsA3HardwareInterface"),
                         *debug_node_->get_clock(), 5000,
                         "Pinocchio 重力计算异常：%s", e.what());
  }

  return gravity_torques;
}

std::vector<double> RsA3HardwareInterface::computePinocchioInverseDynamics(
  const std::vector<double>& positions,
  const std::vector<double>& velocities,
  const std::vector<double>& accelerations)
{
  return computePinocchioInverseDynamics(positions, velocities, accelerations, false);
}

std::vector<double> RsA3HardwareInterface::computePinocchioInverseDynamics(
  const std::vector<double>& positions,
  const std::vector<double>& velocities,
  const std::vector<double>& accelerations,
  bool use_payload_model)
{
  std::vector<double> id_torques(joint_configs_.size(), 0.0);

  if (!pinocchio_initialized_ || !pinocchio_mapping_ready_ ||
      positions.size() != joint_configs_.size() ||
      velocities.size() != joint_configs_.size() ||
      accelerations.size() != joint_configs_.size()) {
    return id_torques;
  }

  try {
    const pinocchio::Model & model =
      (use_payload_model && pinocchio_loaded_initialized_) ? pinocchio_loaded_model_ : pinocchio_model_;
    pinocchio::Data & data =
      (use_payload_model && pinocchio_loaded_initialized_) ? pinocchio_loaded_data_ : pinocchio_data_;
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);

    for (size_t i = 0; i < joint_configs_.size(); ++i) {
      const int q_idx = pinocchio_q_index_map_[i];
      const int v_idx = pinocchio_v_index_map_[i];

      if (q_idx >= 0 && q_idx < q.size()) {
        q[q_idx] = positions[i];
      }
      if (v_idx >= 0 && v_idx < v.size()) {
        v[v_idx] = velocities[i];
      }
      if (v_idx >= 0 && v_idx < a.size()) {
        a[v_idx] = accelerations[i];
      }
    }

    const Eigen::VectorXd tau = pinocchio::rnea(model, data, q, v, a);

    for (size_t i = 0; i < joint_configs_.size(); ++i) {
      const int v_idx = pinocchio_v_index_map_[i];
      if (v_idx < 0 || v_idx >= tau.size()) {
        continue;
      }

      double scale = 1.0;
      if (!use_calibrated_inertia_ && i < inertia_scale_params_.size()) {
        scale = inertia_scale_params_[i].mass_scale;
      }
      id_torques[i] = tau[v_idx] * scale;
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("RsA3HardwareInterface"),
                         *debug_node_->get_clock(), 5000,
                         "Pinocchio 逆动力学计算异常：%s", e.what());
  }

  return id_torques;
}

bool RsA3HardwareInterface::loadCalibratedInertia(const std::string& config_path)
{
  try {
    // Simple YAML parsing (no external library dependency)
    std::ifstream file(config_path);
    if (!file.is_open()) {
      RCLCPP_WARN(rclcpp::get_logger("RsA3HardwareInterface"),
                  "无法打开惯量配置文件：%s", config_path.c_str());
      return false;
    }
    
    // Initialize parameter container
    calibrated_inertia_params_.resize(6);  // L1-L6
    
    // Read defaults from Pinocchio model
    for (size_t i = 0; i < 6 && i + 1 < static_cast<size_t>(pinocchio_model_.nbodies); ++i) {
      const auto& inertia = pinocchio_model_.inertias[i + 1];  // Skip universe
      calibrated_inertia_params_[i].mass = inertia.mass();
      calibrated_inertia_params_[i].com_x = inertia.lever()[0];
      calibrated_inertia_params_[i].com_y = inertia.lever()[1];
      calibrated_inertia_params_[i].com_z = inertia.lever()[2];
    }
    
    // Parse YAML (simplified, line-by-line parsing)
    std::string line;
    std::string current_joint;
    bool in_inertia_params = false;
    bool use_calibrated = false;
    
    while (std::getline(file, line)) {
      // Skip blank lines and comments
      size_t pos = line.find_first_not_of(" \t");
      if (pos == std::string::npos || line[pos] == '#') continue;
      
      // Check use_calibrated_params
      if (line.find("use_calibrated_params:") != std::string::npos) {
        use_calibrated = (line.find("true") != std::string::npos);
        continue;
      }
      
      // Check inertia_params section
      if (line.find("inertia_params:") != std::string::npos) {
        in_inertia_params = true;
        continue;
      }
      
      if (!in_inertia_params) continue;
      
      // Check joint name (L2:, L3:, etc.)
      for (int j = 2; j <= 6; ++j) {
        std::string joint_key = "L" + std::to_string(j) + ":";
        if (line.find(joint_key) != std::string::npos && 
            line.find("mass") == std::string::npos &&
            line.find("com") == std::string::npos) {
          current_joint = "L" + std::to_string(j);
          break;
        }
      }
      
      // Parse mass
      if (!current_joint.empty() && line.find("mass:") != std::string::npos) {
        size_t colon_pos = line.find("mass:");
        std::string value_str = line.substr(colon_pos + 5);
        // Remove comments
        size_t comment_pos = value_str.find('#');
        if (comment_pos != std::string::npos) {
          value_str = value_str.substr(0, comment_pos);
        }
        // Remove whitespace
        value_str.erase(std::remove_if(value_str.begin(), value_str.end(), ::isspace), value_str.end());
        
        int joint_idx = std::stoi(current_joint.substr(1)) - 1;
        if (joint_idx >= 0 && joint_idx < 6) {
          calibrated_inertia_params_[joint_idx].mass = std::stod(value_str);
        }
      }
      
      // Parse com
      if (!current_joint.empty() && line.find("com:") != std::string::npos) {
        size_t bracket_start = line.find('[');
        size_t bracket_end = line.find(']');
        if (bracket_start != std::string::npos && bracket_end != std::string::npos) {
          std::string com_str = line.substr(bracket_start + 1, bracket_end - bracket_start - 1);
          // Parse three values
          std::vector<double> com_values;
          std::stringstream ss(com_str);
          std::string token;
          while (std::getline(ss, token, ',')) {
            token.erase(std::remove_if(token.begin(), token.end(), ::isspace), token.end());
            if (!token.empty()) {
              com_values.push_back(std::stod(token));
            }
          }
          
          int joint_idx = std::stoi(current_joint.substr(1)) - 1;
          if (joint_idx >= 0 && joint_idx < 6 && com_values.size() >= 3) {
            calibrated_inertia_params_[joint_idx].com_x = com_values[0];
            calibrated_inertia_params_[joint_idx].com_y = com_values[1];
            calibrated_inertia_params_[joint_idx].com_z = com_values[2];
          }
        }
      }
    }
    
    file.close();
    
    if (!use_calibrated) {
      RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                  "配置中未启用标定惯量：将使用 URDF 默认值");
      return false;
    }
    
    // 输出加载到的参数
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "已加载标定惯量参数：");
    for (size_t i = 1; i < 6; ++i) {  // L2-L6
      RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                  "  L%zu: mass=%.4f kg, com=[%.4f, %.4f, %.4f] m",
                  i + 1,
                  calibrated_inertia_params_[i].mass,
                  calibrated_inertia_params_[i].com_x,
                  calibrated_inertia_params_[i].com_y,
                  calibrated_inertia_params_[i].com_z);
    }
    
    return true;
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("RsA3HardwareInterface"),
                 "加载惯量配置失败：%s", e.what());
    return false;
  }
}

void RsA3HardwareInterface::applyCalibratedInertiaToModel()
{
  if (!pinocchio_initialized_ || calibrated_inertia_params_.size() < 6) {
    return;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "正在将标定惯量应用到 Pinocchio 模型...");
  
  // Update inertia parameters for L2-L6 (Pinocchio model index starts from 1, skip universe)
  for (size_t i = 1; i < 6 && i + 1 < static_cast<size_t>(pinocchio_model_.nbodies); ++i) {
    const auto& params = calibrated_inertia_params_[i];
    
    // Update mass (mass() returns reference)
    pinocchio_model_.inertias[i + 1].mass() = params.mass;
    
    // Update center of mass position (lever() returns vector reference)
    pinocchio_model_.inertias[i + 1].lever()[0] = params.com_x;
    pinocchio_model_.inertias[i + 1].lever()[1] = params.com_y;
    pinocchio_model_.inertias[i + 1].lever()[2] = params.com_z;
    
    RCLCPP_DEBUG(rclcpp::get_logger("RsA3HardwareInterface"),
                 "  已更新 L%zu：mass=%.4f，com=[%.4f, %.4f, %.4f]",
                 i + 1, params.mass, params.com_x, params.com_y, params.com_z);
  }
  
  // Recreate Pinocchio Data object to apply changes
  pinocchio_data_ = pinocchio::Data(pinocchio_model_);
  configureLoadedPinocchioModel();
  
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "Pinocchio 模型已更新（已应用标定惯量参数）");
}

}  // namespace rc_arm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rc_arm_hardware::RsA3HardwareInterface,
  hardware_interface::SystemInterface)
