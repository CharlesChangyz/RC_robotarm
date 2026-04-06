/**
 * @file rc_arm_hardware.cpp
 * @brief EL-A3 机械臂 ROS2 Control 硬件接口实现
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
}  // namespace

RsA3HardwareInterface::RsA3HardwareInterface()
  : can_interface_("can0")
  , host_can_id_(0xFD)
  , can_enabled_(true)
  , position_kp_(60.0)    // 降低 Kp 以减小振荡
  , position_kd_(3.5)     // 增大 Kd 以提高阻尼
  , velocity_limit_(10.0)
  , control_mode_(ControlMode::POSITION)
  , use_mock_hardware_(false)
  , external_feedback_enabled_(false)
  , external_feedback_topic_("/rc_arm_2/feedback_joint_states")
  , external_feedback_timeout_sec_(0.2)
  , external_feedback_received_(false)
  , external_feedback_last_time_(std::chrono::steady_clock::now())
  , s_curve_enabled_(true)       // 默认启用 S 曲线规划
  , scalar_path_time_enabled_(true)
  , zero_torque_mode_(false)
  , zero_torque_kd_(1.0)
  , low_stiffness_mode_(false)
  , low_stiffness_kp_(20.0)
  , low_stiffness_kd_(2.0)
  , low_stiffness_torque_bias_(0.0)
  , gravity_comp_enabled_(false)
  , gravity_feedforward_ratio_(0.5)  // 默认 50% 重力补偿前馈
  , use_pinocchio_gravity_(false)    // 默认使用简化重力模型
  , use_pinocchio_inverse_dynamics_(true)
  , pinocchio_initialized_(false)
  , pinocchio_mapping_ready_(false)
  , use_calibrated_inertia_(false)   // 默认不使用标定后的惯量参数
  , spin_thread_running_(false)      // 服务回调线程运行标志初始化
  , limit_margin_(0.15)          // 约在 ~15° 处开始减速（≈8.6°）
  , limit_stop_margin_(0.02)     // 约在 ~1° 处硬停止（≈1.1°）
  , limit_decel_factor_(0.3)     // 减速到 30%
  , max_jerk_(50.0)              // 默认最大加加速度 50 rad/s³（S 曲线规划）
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
  if (info_.hardware_parameters.count("can_interface")) {
    can_interface_ = info_.hardware_parameters.at("can_interface");
  }
  if (info_.hardware_parameters.count("host_can_id")) {
    host_can_id_ = std::stoi(info_.hardware_parameters.at("host_can_id"));
  }
  if (info_.hardware_parameters.count("can_enabled")) {
    can_enabled_ = parseBoolParam(info_.hardware_parameters.at("can_enabled"));
  }
  if (info_.hardware_parameters.count("position_kp")) {
    position_kp_ = std::stod(info_.hardware_parameters.at("position_kp"));
  }
  if (info_.hardware_parameters.count("position_kd")) {
    position_kd_ = std::stod(info_.hardware_parameters.at("position_kd"));
  }
  if (info_.hardware_parameters.count("velocity_limit")) {
    velocity_limit_ = std::stod(info_.hardware_parameters.at("velocity_limit"));
  }
  if (info_.hardware_parameters.count("use_mock_hardware")) {
    use_mock_hardware_ = parseBoolParam(info_.hardware_parameters.at("use_mock_hardware"));
  }
  if (info_.hardware_parameters.count("external_feedback_enabled")) {
    external_feedback_enabled_ = parseBoolParam(info_.hardware_parameters.at("external_feedback_enabled"));
  }
  if (info_.hardware_parameters.count("external_feedback_topic")) {
    external_feedback_topic_ = info_.hardware_parameters.at("external_feedback_topic");
  }
  if (info_.hardware_parameters.count("external_feedback_timeout")) {
    external_feedback_timeout_sec_ = std::stod(info_.hardware_parameters.at("external_feedback_timeout"));
  }
  external_feedback_timeout_sec_ = std::max(0.0, external_feedback_timeout_sec_);

  if (info_.hardware_parameters.count("low_stiffness_mode")) {
    low_stiffness_mode_ = parseBoolParam(info_.hardware_parameters.at("low_stiffness_mode"));
  }
  if (info_.hardware_parameters.count("low_stiffness_kp")) {
    low_stiffness_kp_ = std::stod(info_.hardware_parameters.at("low_stiffness_kp"));
  }
  if (info_.hardware_parameters.count("low_stiffness_kd")) {
    low_stiffness_kd_ = std::stod(info_.hardware_parameters.at("low_stiffness_kd"));
  }
  if (info_.hardware_parameters.count("low_stiffness_torque_bias")) {
    low_stiffness_torque_bias_ = std::stod(info_.hardware_parameters.at("low_stiffness_torque_bias"));
  }
  low_stiffness_kp_ = std::clamp(low_stiffness_kp_, 0.0, 500.0);
  low_stiffness_kd_ = std::clamp(low_stiffness_kd_, 0.0, 5.0);

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
  
  // 初始化位置指令平滑滤波（含速度/加速度/加加速度限制 - S 曲线规划）
  smoothed_positions_.resize(num_joints, 0.0);
  smoothed_velocities_.resize(num_joints, 0.0);
  smoothed_accelerations_.resize(num_joints, 0.0);  // 用于 S 曲线规划
  
  // 初始化速度前馈相关变量
  last_cmd_positions_.resize(num_joints, 0.0);         // 上一周期指令位置
  last_hw_commands_positions_.resize(num_joints, 0.0); // 上一帧 hw_commands（用于检测指令更新）
  cmd_velocities_.resize(num_joints, 0.0);             // 计算得到的指令速度
  filtered_cmd_velocities_.resize(num_joints, 0.0);    // 一阶滤波后的指令速度
  velocity_ff_stage2_.resize(num_joints, 0.0);         // 二阶滤波中间量
  velocity_filter_alpha_ = 0.3;                        // 速度滤波系数
  
  // 默认参数
  smoothing_alpha_ = 0.08;      // 平滑系数（越小越平滑）
  max_velocity_ = 2.0;          // 最大速度 2 rad/s
  max_acceleration_ = 8.0;      // 最大加速度 8 rad/s²
  max_jerk_ = 50.0;             // 最大加加速度 50 rad/s³（S 曲线规划）
  control_period_ = 0.005;      // 默认 200Hz -> 5ms
  first_command_ = true;
  gravity_feedforward_ratio_ = 0.5;  // 默认 50% 重力补偿前馈
  s_curve_enabled_ = true;      // 默认启用 S 曲线
  
  // 从参数读取平滑系数
  if (info_.hardware_parameters.count("smoothing_alpha")) {
    smoothing_alpha_ = std::stod(info_.hardware_parameters.at("smoothing_alpha"));
    smoothing_alpha_ = std::clamp(smoothing_alpha_, 0.01, 1.0);
  }
  
  // 从参数读取速度上限
  if (info_.hardware_parameters.count("max_velocity")) {
    max_velocity_ = std::stod(info_.hardware_parameters.at("max_velocity"));
  }
  
  // 从参数读取加速度上限
  if (info_.hardware_parameters.count("max_acceleration")) {
    max_acceleration_ = std::stod(info_.hardware_parameters.at("max_acceleration"));
  }
  
  // 从参数读取加加速度上限（S 曲线规划）
  if (info_.hardware_parameters.count("max_jerk")) {
    max_jerk_ = std::stod(info_.hardware_parameters.at("max_jerk"));
  }
  
  // 从参数读取 S 曲线开关
  if (info_.hardware_parameters.count("s_curve_enabled")) {
    s_curve_enabled_ = parseBoolParam(info_.hardware_parameters.at("s_curve_enabled"));
  }

  // 公共标量路径参数化开关（方案 A）
  if (info_.hardware_parameters.count("scalar_path_time_enabled")) {
    scalar_path_time_enabled_ = parseBoolParam(info_.hardware_parameters.at("scalar_path_time_enabled"));
  }
  
  // 初始化 S 曲线生成器（每个关节一个）
  s_curve_generators_.clear();
  for (size_t i = 0; i < num_joints; ++i) {
    s_curve_generators_.push_back(
      std::make_unique<SCurveGenerator>(max_velocity_, max_acceleration_, max_jerk_));
  }
  
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

  // 初始化“几何路径 q(s) + 公共标量 s(t)”参数化器
  if (scalar_path_time_enabled_) {
    ScalarPathPlannerConfig planner_cfg;
    planner_cfg.enabled = true;
    planner_cfg.dof = num_joints;
    planner_cfg.waypoint_merge_distance = 1e-3;
    planner_cfg.command_append_distance = 2e-3;
    planner_cfg.max_waypoint_jump = M_PI;
    planner_cfg.max_raw_waypoints = 160;
    planner_cfg.min_waypoints = 2;
    planner_cfg.constraint_samples = 200;

    planner_cfg.joint_names.reserve(num_joints);
    planner_cfg.velocity_limits.reserve(num_joints);
    planner_cfg.acceleration_limits.reserve(num_joints);
    planner_cfg.jerk_limits.reserve(num_joints);
    planner_cfg.lower_limits.reserve(num_joints);
    planner_cfg.upper_limits.reserve(num_joints);
    planner_cfg.is_continuous.reserve(num_joints);

    for (const auto& cfg : joint_configs_) {
      planner_cfg.joint_names.push_back(cfg.name);
      planner_cfg.velocity_limits.push_back(std::max(1e-3, cfg.velocity_limit));
      planner_cfg.acceleration_limits.push_back(std::max(1e-3, max_acceleration_));
      planner_cfg.jerk_limits.push_back(std::max(1e-3, max_jerk_));
      planner_cfg.lower_limits.push_back(cfg.lower_limit);
      planner_cfg.upper_limits.push_back(cfg.upper_limit);
      planner_cfg.is_continuous.push_back(cfg.is_continuous);
    }

    scalar_path_planner_ = std::make_unique<ScalarPathTimePlanner>(planner_cfg);
    scalar_path_planner_->resetToPosition(hw_positions_);
  } else {
    scalar_path_planner_.reset();
  }

  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "已初始化：%zu 个关节，CAN 接口：%s（CAN %s）",
              num_joints, can_interface_.c_str(), can_enabled_ ? "启用" : "禁用");
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "  S 曲线：%s，max_vel=%.1f rad/s，max_acc=%.1f rad/s²，max_jerk=%.1f rad/s³",
              s_curve_enabled_ ? "启用" : "禁用",
              max_velocity_, max_acceleration_, max_jerk_);
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "  公共标量路径参数化：%s",
              scalar_path_time_enabled_ ? "启用" : "禁用");
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "  PID：Kp=%.1f，Kd=%.1f，重力补偿前馈比例=%.0f%%",
              position_kp_, position_kd_, gravity_feedforward_ratio_ * 100.0);
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "  低刚度模式：%s（Kp=%.1f，Kd=%.1f，torque_bias=%.3f）",
              low_stiffness_mode_ ? "启用" : "禁用",
              low_stiffness_kp_, low_stiffness_kd_,
              low_stiffness_torque_bias_);
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "  Pinocchio：gravity=%s, inverse_dynamics=%s",
              use_pinocchio_gravity_ ? "启用" : "禁用",
              use_pinocchio_inverse_dynamics_ ? "启用" : "禁用");

  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "关节限位保护：margin=%.3f rad，stop_margin=%.3f rad，decel_factor=%.2f",
              limit_margin_, limit_stop_margin_, limit_decel_factor_);

  if (external_feedback_enabled_) {
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "  外部反馈：启用，topic=%s，timeout=%.3fs",
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
  scalar_path_debug_pub_ = debug_node_->create_publisher<sensor_msgs::msg::JointState>("/debug/scalar_path_state", 10);

  if (external_feedback_enabled_) {
    external_feedback_sub_ = debug_node_->create_subscription<sensor_msgs::msg::JointState>(
      external_feedback_topic_,
      20,
      std::bind(&RsA3HardwareInterface::externalFeedbackCallback, this, std::placeholders::_1));
  }
  
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "调试发布器已创建：/debug/hw_command, /debug/smoothed_command, /debug/gravity_torque, /debug/velocity_feedforward, /debug/motor_temperature, /debug/final_joint_command, /debug/final_joint_command_joint_frame, /debug/final_pd_gains, /debug/final_joint_torque_ff");

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

    // Parse joint-specific Kp/Kd (0 means use global value)
    config.kp = 0.0;
    config.kd = 0.0;
    if (joint.parameters.count("kp")) {
      config.kp = std::stod(joint.parameters.at("kp"));
    }
    if (joint.parameters.count("kd")) {
      config.kd = std::stod(joint.parameters.at("kd"));
    }

    // Parse joint-specific low-stiffness Kp/Kd (0 means use global low-stiffness value)
    config.low_stiffness_kp = 0.0;
    config.low_stiffness_kd = 0.0;
    if (joint.parameters.count("low_stiffness_kp")) {
      config.low_stiffness_kp = std::stod(joint.parameters.at("low_stiffness_kp"));
    }
    if (joint.parameters.count("low_stiffness_kd")) {
      config.low_stiffness_kd = std::stod(joint.parameters.at("low_stiffness_kd"));
    }
    config.low_stiffness_kp = std::clamp(config.low_stiffness_kp, 0.0, 500.0);
    config.low_stiffness_kd = std::clamp(config.low_stiffness_kd, 0.0, 5.0);

    joint_configs_.push_back(config);

    const bool has_joint_pd = (config.kp > 0.0 || config.kd > 0.0);
    const bool has_low_stiffness_pd = (config.low_stiffness_kp > 0.0 || config.low_stiffness_kd > 0.0);

    if (has_joint_pd || has_low_stiffness_pd) {
      RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                  "关节 %s：motor_id=%d，type=%s，dir=%.1f，限位=[%.1f°~%.1f°]，普通PD[Kp=%.1f，Kd=%.1f]，低刚度PD[Kp=%.1f，Kd=%.1f]",
                  config.name.c_str(), config.motor_id,
                  config.motor_type == MotorType::RS00 ? "RS00" : "EL05",
                  config.direction,
                  config.lower_limit * 180.0 / M_PI, config.upper_limit * 180.0 / M_PI,
                  config.kp, config.kd,
                  config.low_stiffness_kp, config.low_stiffness_kd);
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

  if (!can_enabled_) {
    RCLCPP_WARN(rclcpp::get_logger("RsA3HardwareInterface"),
                "CAN 已禁用：跳过 CAN 初始化，将仅执行控制计算与调试发布");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // 创建并初始化 CAN 驱动
  can_driver_ = std::make_unique<RobstrideCanDriver>(can_interface_, host_can_id_);
  
  if (!can_driver_->init()) {
    RCLCPP_ERROR(rclcpp::get_logger("RsA3HardwareInterface"),
                 "CAN 驱动初始化失败：%s", can_interface_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 为每个关节设置电机型号（添加延时避免 CAN 缓冲区溢出）
  for (const auto& config : joint_configs_) {
    can_driver_->setMotorType(config.motor_id, config.motor_type);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));  // Add delay
  }

  // 启动接收线程
  can_driver_->startReceiveThread();

  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"), "硬件已配置完成：%s", can_interface_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RsA3HardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (can_driver_) {
    can_driver_->stopReceiveThread();
    can_driver_->close();
    can_driver_.reset();
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

  if (!can_enabled_) {
    for (size_t i = 0; i < joint_configs_.size(); ++i) {
      const auto& config = joint_configs_[i];
      const double initial_position = hw_positions_[i];

      hw_commands_positions_[i] = initial_position;
      smoothed_positions_[i] = initial_position;
      smoothed_velocities_[i] = 0.0;
      smoothed_accelerations_[i] = 0.0;

      const double low_joint_kp = std::clamp(
        (config.low_stiffness_kp > 0.0) ? config.low_stiffness_kp : low_stiffness_kp_, 0.0, 500.0);
      const double low_joint_kd = std::clamp(
        (config.low_stiffness_kd > 0.0) ? config.low_stiffness_kd : low_stiffness_kd_, 0.0, 5.0);

      final_cmd_positions_[i] = initial_position;
      final_cmd_velocities_[i] = 0.0;
      final_cmd_efforts_[i] = low_stiffness_mode_ ? low_stiffness_torque_bias_ : 0.0;
      final_cmd_kps_[i] = low_stiffness_mode_ ? low_joint_kp : position_kp_;
      final_cmd_kds_[i] = low_stiffness_mode_ ? low_joint_kd : position_kd_;

      if (s_curve_enabled_ && i < s_curve_generators_.size() && s_curve_generators_[i]) {
        s_curve_generators_[i]->initialize(initial_position, 0.0, 0.0);
      }

      RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                  "关节 %s CAN 关闭仿真初始位置：%.4f rad",
                  config.name.c_str(), initial_position);
    }

    first_command_ = false;
    if (scalar_path_planner_) {
      scalar_path_planner_->resetToPosition(hw_positions_);
    }
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "CAN 关闭仿真模式已激活：将持续发布最终控制包，不下发电机");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // ============ 步骤 0：清除所有电机故障 ============
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "正在清除电机故障...");
  for (const auto& config : joint_configs_) {
    can_driver_->disableMotor(config.motor_id, true);  // clear_fault=true
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  
  // ============ 步骤 1：使能所有电机（Kp=0，无位置控制） ============
  // 电机在失能状态不会回传反馈，因此必须先使能
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "正在以软模式使能电机（Kp=0）...");
  
  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    const auto& config = joint_configs_[i];
    
    // 1) 先停止电机（不清故障，前面已清过）
    can_driver_->disableMotor(config.motor_id, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    
    // 2) 设置为运控模式（run_mode = 0）
    if (!can_driver_->setRunMode(config.motor_id, RunMode::MOTION_CONTROL)) {
      RCLCPP_ERROR(rclcpp::get_logger("RsA3HardwareInterface"),
                   "电机 %d 设置运控模式失败", config.motor_id);
      return hardware_interface::CallbackReturn::ERROR;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    
    // 3) 使能电机
    if (!can_driver_->enableMotor(config.motor_id)) {
      RCLCPP_ERROR(rclcpp::get_logger("RsA3HardwareInterface"),
                   "电机 %d 使能失败", config.motor_id);
      return hardware_interface::CallbackReturn::ERROR;
    }
    
    // 4) [关键] 发送 Kp=0 的指令，使电机进入“软”状态，不跟踪任何位置
    //    这样即使发送任意位置指令，电机也不会运动
    can_driver_->sendMotionControl(
        config.motor_id,
        config.motor_type,
        0.0,          // 位置无关紧要，因为 Kp=0
        0.0,          // velocity = 0
        0.0,          // Kp = 0（不跟踪位置！）
        4.0,          // Kd = 4.0（提高阻尼，防止抖动）
        0.0           // torque = 0
    );
    
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "电机 %d 已以软模式使能（Kp=0，Kd=4）", config.motor_id);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  // ============ 步骤 2：开环控制 - 使用默认位置 (0)，跳过反馈等待 ============
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "开环模式：所有关节使用默认位置 0.0，跳过反馈等待");
  
  std::vector<double> initial_positions(joint_configs_.size(), 0.0);
  
  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    const auto& config = joint_configs_[i];
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "电机 %d 初始位置：0.0 rad（开环）", config.motor_id);
  }

  // ============ 步骤 3：发送保持指令（切换到正常控制） ============
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "正在切换到位置保持模式...");
  
  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    const auto& config = joint_configs_[i];
    
    // 初始化状态变量
    hw_positions_[i] = initial_positions[i];
    hw_commands_positions_[i] = initial_positions[i];
    smoothed_positions_[i] = initial_positions[i];
    smoothed_velocities_[i] = 0.0;
    smoothed_accelerations_[i] = 0.0;
    
    // 初始化 S 曲线生成器状态
    if (s_curve_enabled_ && i < s_curve_generators_.size()) {
      s_curve_generators_[i]->initialize(initial_positions[i], 0.0, 0.0);
    }
    
    // 计算电机坐标系下的位置
    double motor_pos = initial_positions[i] * config.direction + config.position_offset;
    
    // 发送“保持当前位置”指令（按当前模式选择 Kp/Kd/torque）
    const double low_joint_kp = std::clamp(
      (config.low_stiffness_kp > 0.0) ? config.low_stiffness_kp : low_stiffness_kp_, 0.0, 500.0);
    const double low_joint_kd = std::clamp(
      (config.low_stiffness_kd > 0.0) ? config.low_stiffness_kd : low_stiffness_kd_, 0.0, 5.0);

    double hold_kp = low_stiffness_mode_ ? low_joint_kp : position_kp_;
    double hold_kd = low_stiffness_mode_ ? low_joint_kd : position_kd_;
    double hold_torque = low_stiffness_mode_ ? low_stiffness_torque_bias_ : 0.0;

    can_driver_->sendMotionControl(
        config.motor_id,
        config.motor_type,
        motor_pos,        // 使用当前位置
        0.0,              // velocity = 0
        hold_kp,
        hold_kd,
        hold_torque
    );

    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "电机 %d 保持在 %.4f rad（Kp=%.1f，Kd=%.1f，torque=%.3f）",
                config.motor_id, initial_positions[i], hold_kp, hold_kd, hold_torque);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }
  
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "全部 %zu 个电机初始化成功", joint_configs_.size());
  
  first_command_ = false;
  if (scalar_path_planner_) {
    scalar_path_planner_->resetToPosition(hw_positions_);
  }

  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"), 
              "硬件已激活（CSP 位置模式）");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RsA3HardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (!use_mock_hardware_ && can_enabled_ && can_driver_) {
    // 失能所有电机
    for (const auto& config : joint_configs_) {
      can_driver_->disableMotor(config.motor_id);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
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
  if (!use_mock_hardware_ && can_enabled_ && can_driver_) {
    // 急停：失能所有电机并清故障
    for (const auto& config : joint_configs_) {
      can_driver_->disableMotor(config.motor_id, true);
    }
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
    const size_t position_count = std::min(joint_count, msg->position.size());
    const size_t velocity_count = std::min(joint_count, msg->velocity.size());
    const size_t effort_count = std::min(joint_count, msg->effort.size());

    // 不再按 name 匹配，直接按数组顺序映射：第 i 个消息元素 -> 第 i 个关节
    for (size_t i = 0; i < position_count; ++i) {
      external_feedback_positions_[i] = msg->position[i];
      got_any_position = true;
    }
    for (size_t i = 0; i < velocity_count; ++i) {
      external_feedback_velocities_[i] = msg->velocity[i];
    }
    for (size_t i = 0; i < effort_count; ++i) {
      external_feedback_efforts_[i] = msg->effort[i];
    }

    if (got_any_position || !msg->velocity.empty() || !msg->effort.empty()) {
      external_feedback_last_time_ = std::chrono::steady_clock::now();
      external_feedback_received_.store(true);
    }
  }
}


hardware_interface::return_type RsA3HardwareInterface::read(
  const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  // ============ Read all states from actual motors ============
  if (!use_mock_hardware_ && can_enabled_ && can_driver_) {
    for (size_t i = 0; i < joint_configs_.size(); ++i) {
      const auto& config = joint_configs_[i];
      auto feedback = can_driver_->getMotorFeedback(config.motor_id);
      
      if (feedback.is_valid) {
        // Convert from motor coordinate frame to joint coordinate frame
        // motor_pos = joint_pos * direction + offset
        // joint_pos = (motor_pos - offset) / direction = (motor_pos - offset) * direction
        hw_positions_[i] = (feedback.position - config.position_offset) * config.direction;
        hw_velocities_[i] = feedback.velocity * config.direction;
        hw_efforts_[i] = feedback.torque * config.direction;
        hw_temperatures_[i] = feedback.temperature;  // 电机温度 (°C)
      }
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
      else{
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("RsA3HardwareInterface"), *debug_node_->get_clock(), 5000,
                             "外部反馈数据过旧 (%.2f 秒)，已放弃使用", feedback_age);
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
  static double last_scalar_s = 0.0;
  static bool scalar_s_initialized = false;

  if (use_mock_hardware_) {
    return hardware_interface::return_type::OK;
  }

  const bool can_ready = can_enabled_ && can_driver_ && can_driver_->isConnected();
  if (can_enabled_ && !can_ready) {
    return hardware_interface::return_type::ERROR;
  }

  double dt = period.seconds();
  if (dt <= 0.0 || dt > 0.1) {
    dt = control_period_;
  }

  const bool using_scalar_path = scalar_path_time_enabled_ && static_cast<bool>(scalar_path_planner_);

  if (first_command_) {
    for (size_t i = 0; i < joint_configs_.size(); ++i) {
      smoothed_positions_[i] = hw_commands_positions_[i];
      smoothed_velocities_[i] = 0.0;
      smoothed_accelerations_[i] = 0.0;

      last_cmd_positions_[i] = hw_commands_positions_[i];
      filtered_cmd_velocities_[i] = 0.0;
      velocity_ff_stage2_[i] = 0.0;

      if (s_curve_enabled_ && i < s_curve_generators_.size() && s_curve_generators_[i]) {
        s_curve_generators_[i]->initialize(hw_commands_positions_[i], 0.0, 0.0);
      }
    }

    if (using_scalar_path) {
      scalar_path_planner_->resetToPosition(smoothed_positions_);
      scalar_s_initialized = false;
    }

    first_command_ = false;
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "收到首条指令，初始化轨迹（公共标量路径：%s，逐关节 S 曲线：%s）",
                using_scalar_path ? "启用" : "禁用",
                s_curve_enabled_ ? "启用" : "禁用");
  }

  const double command_change_eps = 1e-3;
  bool command_changed = false;
  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    if (std::abs(hw_commands_positions_[i] - last_hw_commands_positions_[i]) > command_change_eps) {
      command_changed = true;
      break;
    }
  }

  bool command_updated = command_changed;

  if (!command_updated) {
    if (using_scalar_path && scalar_path_planner_->hasActiveProfile()) {
      command_updated = true;
    } else if (!using_scalar_path && s_curve_enabled_) {
      for (size_t i = 0; i < joint_configs_.size() && i < s_curve_generators_.size(); ++i) {
        if (s_curve_generators_[i] && s_curve_generators_[i]->isMoving()) {
          command_updated = true;
          break;
        }
      }
    }
  }

  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    last_hw_commands_positions_[i] = hw_commands_positions_[i];
  }

  std::vector<double> cmd_positions_motor(joint_configs_.size(), 0.0);
  std::vector<double> cmd_velocities_motor(joint_configs_.size(), 0.0);

  std::vector<double> id_ref_positions = smoothed_positions_;
  std::vector<double> id_ref_velocities = smoothed_velocities_;
  std::vector<double> id_ref_accelerations = smoothed_accelerations_;

  bool scalar_sample_valid = false;
  ScalarPathSample scalar_sample;
  if (using_scalar_path) {
    if (command_changed) {
      std::string planner_message;
      const bool profile_rebuilt = scalar_path_planner_->ingestWaypoint(
        hw_commands_positions_, false, planner_message);

      if (profile_rebuilt) {
        const auto& diag = scalar_path_planner_->diagnostics();
        const std::string vel_joint = (diag.vel_bottleneck_joint < joint_configs_.size())
                                      ? joint_configs_[diag.vel_bottleneck_joint].name
                                      : std::string("unknown");
        const std::string acc_joint = (diag.acc_bottleneck_joint < joint_configs_.size())
                                      ? joint_configs_[diag.acc_bottleneck_joint].name
                                      : std::string("unknown");
        RCLCPP_INFO(
          rclcpp::get_logger("RsA3HardwareInterface"),
          "公共标量路径已重建：raw=%zu cleaned=%zu L=%.6f dt=%.4f T=%.6f Vs=%.6f As=%.6f Js=%.6f bottleneck[v=%s,a=%s]",
          diag.raw_waypoints,
          diag.cleaned_waypoints,
          diag.path_length,
          dt,
          diag.total_time,
          diag.v_s_limit,
          diag.a_s_limit,
          diag.j_s_limit,
          vel_joint.c_str(),
          acc_joint.c_str());
        scalar_s_initialized = false;
      } else if (planner_message != "insufficient waypoints" && planner_message != "planner disabled") {
        RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger("RsA3HardwareInterface"),
          *debug_node_->get_clock(),
          2000,
          "公共标量路径构建失败：%s",
          planner_message.c_str());
      }
    }

    scalar_sample_valid = scalar_path_planner_->sample(dt, scalar_sample);

    if (scalar_sample_valid) {
      if (!scalar_s_initialized || command_changed) {
        last_scalar_s = scalar_sample.s;
        scalar_s_initialized = true;
      } else {
        if (scalar_sample.s + 1e-6 < last_scalar_s) {
          RCLCPP_WARN_THROTTLE(
            rclcpp::get_logger("RsA3HardwareInterface"),
            *debug_node_->get_clock(),
            2000,
            "公共标量路径推进非单调：last_s=%.6f, s=%.6f",
            last_scalar_s,
            scalar_sample.s);
        }
        last_scalar_s = scalar_sample.s;
      }

      if (write_counter % 100 == 0) {
        RCLCPP_DEBUG(
          rclcpp::get_logger("RsA3HardwareInterface"),
          "[ScalarPath] t=%.4f s=%.6f sd=%.6f sdd=%.6f",
          scalar_sample.t,
          scalar_sample.s,
          scalar_sample.sd,
          scalar_sample.sdd);
      }
    }
  }

  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    const auto& config = joint_configs_[i];

    const double target_position = hw_commands_positions_[i];
    const double prev_position = smoothed_positions_[i];
    const double prev_velocity = smoothed_velocities_[i];

    double new_position = target_position;
    double new_velocity = 0.0;
    double new_acceleration = 0.0;

    if (scalar_sample_valid &&
        i < scalar_sample.q.size() &&
        i < scalar_sample.v.size() &&
        i < scalar_sample.a.size()) {
      new_position = scalar_sample.q[i];
      new_velocity = scalar_sample.v[i];
      new_acceleration = scalar_sample.a[i];
    } else if (!using_scalar_path && s_curve_enabled_ && i < s_curve_generators_.size() && s_curve_generators_[i]) {
      s_curve_generators_[i]->setTarget(target_position);
      new_position = s_curve_generators_[i]->update(dt);
      new_velocity = s_curve_generators_[i]->getVelocity();
      new_acceleration = s_curve_generators_[i]->getAcceleration();
    } else {
      new_velocity = (new_position - prev_position) / dt;
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
    if (command_updated) {
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
    } else {
      filtered_velocity = velocity_ff_stage2_[i] * 0.95;
      filtered_cmd_velocities_[i] *= 0.95;
    }

    if (std::abs(filtered_velocity) < 0.005) {
      filtered_velocity = 0.0;
    }

    velocity_ff_stage2_[i] = filtered_velocity;
    cmd_positions_motor[i] = cmd_position;
    cmd_velocities_motor[i] = velocity_ff_stage2_[i] * config.direction;
  }

  if (scalar_sample_valid && scalar_path_debug_pub_ && debug_node_) {
    sensor_msgs::msg::JointState scalar_msg;
    scalar_msg.header.stamp = debug_node_->get_clock()->now();
    for (const auto& cfg : joint_configs_) {
      scalar_msg.name.push_back(cfg.name);
    }
    scalar_msg.position = id_ref_positions;
    scalar_msg.velocity = id_ref_velocities;
    scalar_msg.effort = id_ref_accelerations;
    scalar_path_debug_pub_->publish(scalar_msg);
  }

  std::vector<double> pinocchio_gravity_torques;
  if (pinocchio_initialized_ && (use_pinocchio_gravity_ || use_pinocchio_inverse_dynamics_)) {
    pinocchio_gravity_torques = computePinocchioGravity(hw_positions_);
  }

  std::vector<double> pinocchio_id_torques;
  if (use_pinocchio_inverse_dynamics_ && pinocchio_initialized_) {
    pinocchio_id_torques = computePinocchioInverseDynamics(
      id_ref_positions, id_ref_velocities, id_ref_accelerations);
  }

  if (last_tau_for_spike_check.size() != joint_configs_.size()) {
    last_tau_for_spike_check.assign(joint_configs_.size(), 0.0);
  }

  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    const auto& config = joint_configs_[i];
    auto params = getMotorParams(config.motor_type);

    double gravity_torque = 0.0;
    if (gravity_comp_enabled_ || (use_pinocchio_gravity_ && pinocchio_initialized_)) {
      if (use_pinocchio_gravity_ && pinocchio_initialized_ && i < pinocchio_gravity_torques.size()) {
        gravity_torque = pinocchio_gravity_torques[i] * gravity_feedforward_ratio_;
      } else {
        gravity_torque = computeGravityTorque(i, hw_positions_[i]) * gravity_feedforward_ratio_;
      }
    }

    double model_feedforward_torque = gravity_torque;
    if (use_pinocchio_inverse_dynamics_ && pinocchio_initialized_ && i < pinocchio_id_torques.size()) {
      model_feedforward_torque = pinocchio_id_torques[i];
      if (i < pinocchio_gravity_torques.size()) {
        model_feedforward_torque -= (1.0 - gravity_feedforward_ratio_) * pinocchio_gravity_torques[i];
      }
    }

    const double low_joint_kp = std::clamp(
      (config.low_stiffness_kp > 0.0) ? config.low_stiffness_kp : low_stiffness_kp_, 0.0, 500.0);
    const double low_joint_kd = std::clamp(
      (config.low_stiffness_kd > 0.0) ? config.low_stiffness_kd : low_stiffness_kd_, 0.0, 5.0);

    double motor_kp = 0.0;
    double motor_kd = 0.0;
    double cmd_torque = 0.0;
    double final_cmd_position = cmd_positions_motor[i];

    if (zero_torque_mode_) {
      motor_kp = 0.0;
      motor_kd = std::clamp(zero_torque_kd_, 0.0, 5.0);

      if (use_pinocchio_gravity_ && pinocchio_initialized_ && i < pinocchio_gravity_torques.size()) {
        cmd_torque = pinocchio_gravity_torques[i];
      } else {
        cmd_torque = computeGravityTorque(i, hw_positions_[i]);
      }

      final_cmd_position = hw_positions_[i] * config.direction + config.position_offset;
    } else if (low_stiffness_mode_) {
      motor_kp = low_joint_kp;
      motor_kd = low_joint_kd;
      cmd_torque = model_feedforward_torque + low_stiffness_torque_bias_;
    } else {
      const double joint_kp = (config.kp > 0.0) ? config.kp : position_kp_;
      const double joint_kd = (config.kd > 0.0) ? config.kd : position_kd_;
      motor_kp = std::clamp(joint_kp, 0.0, 500.0);
      motor_kd = std::clamp(joint_kd, 0.0, 5.0);
      cmd_torque = model_feedforward_torque;
    }

    cmd_torque = std::clamp(cmd_torque, params.t_min, params.t_max);

    const double tau_jump = std::abs(cmd_torque - last_tau_for_spike_check[i]);
    if (tau_jump > 4.0) {
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("RsA3HardwareInterface"),
        *debug_node_->get_clock(),
        2000,
        "关节 %s 力矩跳变较大：|Δtau|=%.4f",
        config.name.c_str(),
        tau_jump);
    }
    last_tau_for_spike_check[i] = cmd_torque;

    const double final_cmd_velocity = zero_torque_mode_ ? 0.0 : cmd_velocities_motor[i];

    final_cmd_positions_[i] = (final_cmd_position - config.position_offset) * config.direction;
    final_cmd_velocities_[i] = final_cmd_velocity * config.direction;
    final_cmd_efforts_[i] = cmd_torque;
    final_cmd_kps_[i] = motor_kp;
    final_cmd_kds_[i] = motor_kd;
    final_cmd_torque_ff_[i] = cmd_torque;

    if (can_ready && !can_driver_->sendMotionControl(
          config.motor_id,
          config.motor_type,
          final_cmd_position,
          final_cmd_velocity,
          motor_kp,
          motor_kd,
          cmd_torque)) {
      static int warn_counter = 0;
      if (warn_counter++ % 1000 == 0) {
        RCLCPP_WARN(rclcpp::get_logger("RsA3HardwareInterface"),
                    "向电机 %d 发送运控指令失败", config.motor_id);
      }
    }

    if (can_ready) {
      usleep(50);
    }
  }

  if (write_counter % 10 == 0 && debug_node_ && hw_cmd_pub_ && smoothed_cmd_pub_) {
    auto now = debug_node_->get_clock()->now();

    sensor_msgs::msg::JointState hw_cmd_msg;
    hw_cmd_msg.header.stamp = now;
    for (const auto& config : joint_configs_) {
      hw_cmd_msg.name.push_back(config.name);
    }
    hw_cmd_msg.position = hw_commands_positions_;
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
  }

  if (debug_node_ && final_cmd_pub_ && final_cmd_joint_frame_pub_ && final_pd_pub_ && final_torque_ff_pub_) {
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

    final_cmd_msg.position = final_cmd_positions_;
    final_cmd_msg.velocity = final_cmd_velocities_;
    final_cmd_msg.effort = final_cmd_efforts_;

    sensor_msgs::msg::JointState final_cmd_joint_frame_msg;
    final_cmd_joint_frame_msg.header.stamp = now;
    final_cmd_joint_frame_msg.name = final_cmd_msg.name;
    final_cmd_joint_frame_msg.position = final_cmd_positions_;
    final_cmd_joint_frame_msg.velocity = final_cmd_velocities_;
    for (size_t i = 0; i < final_cmd_efforts_.size() && i < joint_configs_.size(); ++i) {
      final_cmd_joint_frame_msg.effort.push_back(final_cmd_efforts_[i] * joint_configs_[i].direction);
    }

    final_pd_msg.position = final_cmd_kps_;
    final_pd_msg.velocity = final_cmd_kds_;

    for (size_t i = 0; i < final_cmd_torque_ff_.size() && i < joint_configs_.size(); ++i) {
      final_torque_ff_msg.effort.push_back(final_cmd_torque_ff_[i] * joint_configs_[i].direction);
    }

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

    pinocchio_initialized_ = true;
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

std::vector<double> RsA3HardwareInterface::computePinocchioGravity(
  const std::vector<double>& positions)
{
  std::vector<double> gravity_torques(joint_configs_.size(), 0.0);

  if (!pinocchio_initialized_ || !pinocchio_mapping_ready_ || positions.size() != joint_configs_.size()) {
    return gravity_torques;
  }

  try {
    Eigen::VectorXd q = Eigen::VectorXd::Zero(pinocchio_model_.nq);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(pinocchio_model_.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(pinocchio_model_.nv);

    for (size_t i = 0; i < joint_configs_.size(); ++i) {
      const int q_idx = pinocchio_q_index_map_[i];
      if (q_idx >= 0 && q_idx < q.size()) {
        q[q_idx] = positions[i];
      }
    }

    const Eigen::VectorXd tau = pinocchio::rnea(pinocchio_model_, pinocchio_data_, q, v, a);

    for (size_t i = 0; i < joint_configs_.size(); ++i) {
      const int v_idx = pinocchio_v_index_map_[i];
      if (v_idx < 0 || v_idx >= tau.size()) {
        continue;
      }

      double scale = 1.0;
      if (!use_calibrated_inertia_ && i < inertia_scale_params_.size()) {
        scale = inertia_scale_params_[i].mass_scale;
      }

      gravity_torques[i] = tau[v_idx] * scale * joint_configs_[i].direction;
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
  std::vector<double> id_torques(joint_configs_.size(), 0.0);

  if (!pinocchio_initialized_ || !pinocchio_mapping_ready_ ||
      positions.size() != joint_configs_.size() ||
      velocities.size() != joint_configs_.size() ||
      accelerations.size() != joint_configs_.size()) {
    return id_torques;
  }

  try {
    Eigen::VectorXd q = Eigen::VectorXd::Zero(pinocchio_model_.nq);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(pinocchio_model_.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(pinocchio_model_.nv);

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

    const Eigen::VectorXd tau = pinocchio::rnea(pinocchio_model_, pinocchio_data_, q, v, a);

    for (size_t i = 0; i < joint_configs_.size(); ++i) {
      const int v_idx = pinocchio_v_index_map_[i];
      if (v_idx < 0 || v_idx >= tau.size()) {
        continue;
      }

      double scale = 1.0;
      if (!use_calibrated_inertia_ && i < inertia_scale_params_.size()) {
        scale = inertia_scale_params_[i].mass_scale;
      }
      id_torques[i] = tau[v_idx] * scale * joint_configs_[i].direction;
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
  
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "Pinocchio 模型已更新（已应用标定惯量参数）");
}

}  // namespace rc_arm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rc_arm_hardware::RsA3HardwareInterface,
  hardware_interface::SystemInterface)

