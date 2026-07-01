/**
 * @file rc_arm_hardware.hpp
 * @brief EL-A3 机械臂的 ROS2 Control 硬件接口
 */

#ifndef RC_ARM_HARDWARE__RC_ARM_HARDWARE_HPP_
#define RC_ARM_HARDWARE__RC_ARM_HARDWARE_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "arm_msgs/msg/can_frame.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "dmbot_serial/dm_motor_driver.hpp"
#include "rc_arm_hardware/motor_params.hpp"

// Pinocchio：用于动力学计算
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace rc_arm_hardware
{

/**
 * @brief 关节配置
 */
struct JointConfig
{
  std::string name;
  uint8_t motor_id;
  MotorType motor_type;
  double position_offset;
  double direction;  // 1.0 or -1.0
  double lower_limit;  // Joint lower limit (rad)
  double upper_limit;  // Joint upper limit (rad)
  double unloaded_kp;           // 空载关节独立 Kp（0 表示使用全局默认值）
  double unloaded_kd;           // 空载关节独立 Kd（0 表示使用全局默认值）
  double unloaded_low_stiffness_kp;  // 空载低刚度模式关节独立 Kp（0 表示使用全局默认值）
  double unloaded_low_stiffness_kd;  // 空载低刚度模式关节独立 Kd（0 表示使用全局默认值）
  double payload_kp;            // 带载关节独立 Kp（0 表示使用全局默认值）
  double payload_kd;            // 带载关节独立 Kd（0 表示使用全局默认值）
  double payload_low_stiffness_kp;   // 带载低刚度模式关节独立 Kp（0 表示使用全局默认值）
  double payload_low_stiffness_kd;   // 带载低刚度模式关节独立 Kd（0 表示使用全局默认值）
  double velocity_limit;   // 关节速度上限（rad/s）
  bool is_continuous;      // 是否连续旋转关节（需做 unwrap）
};

struct ControlGainSet
{
  double position_kp{60.0};
  double position_kd{3.5};
  double low_stiffness_kp{20.0};
  double low_stiffness_kd{2.0};
  double low_stiffness_torque_bias{0.0};
};

/**
 * @brief EL-A3 的 ROS2 Control 硬件接口
 */
class RsA3HardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RsA3HardwareInterface)

  RsA3HardwareInterface();
  ~RsA3HardwareInterface() override;

  // SystemInterface 接口
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo& info) override;
  
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State& previous_state) override;
  
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State& previous_state) override;
  
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State& previous_state) override;
  
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State& previous_state) override;
  
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State& previous_state) override;
  
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;
  
  hardware_interface::return_type write(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  /**
   * @brief 从硬件信息中解析关节配置
   */
  bool parseJointConfig(const hardware_interface::HardwareInfo& info);

  enum class BackendMode
  {
    REAL,
    MUJOCO
  };
  
  // 后端模式
  BackendMode backend_mode_;
  std::string backend_name_;

  // dmbot_serial 实机后端
  std::unique_ptr<dmbot_serial::DmMotorDriver> dm_driver_;
  std::string dm_serial_number_;
  uint32_t dm_nominal_baud_;
  uint32_t dm_data_baud_;
  double dm_startup_delay_sec_;
  int dm_enable_retry_count_;
  double dm_enable_retry_interval_sec_;

  // MuJoCo topic 后端
  std::string mujoco_command_topic_;
  
  // 关节配置
  std::vector<JointConfig> joint_configs_;
  
  // 状态接口数据
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  
  // 指令接口数据
  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_commands_accelerations_;
  std::vector<double> hw_commands_efforts_;
  std::vector<double> final_cmd_positions_;   // 最终控制包（joint frame）中的目标位置
  std::vector<double> final_cmd_velocities_;  // 最终控制包（joint frame）中的目标速度
  std::vector<double> final_cmd_efforts_;     // 最终控制包（joint frame）中的目标力矩
  std::vector<double> final_cmd_kps_;         // 实际发送控制中的 Kp
  std::vector<double> final_cmd_kds_;         // 实际发送控制中的 Kd
  std::vector<double> final_cmd_torque_ff_;   // 最终前馈力矩（joint frame，供调试/桥接拆分）
  
  // 控制参数
  ControlGainSet unloaded_gains_;
  ControlGainSet payload_gains_;
  double velocity_limit_;
  std::string vacuum_activate_topic_;
  std::string payload_command_topic_;
  std::string payload_active_topic_;
  std::string j5_command_topic_;
  std::string j5_position_topic_;
  std::string camera_position_topic_;
  std::string dm_serial_rx_topic_;
  std::string dm_serial_tx_topic_;
  std::string payload_frame_;
  double payload_mass_;
  std::array<double, 3> payload_diaginertia_;
  std::array<double, 3> payload_com_offset_;
  std::array<double, 3> payload_box_size_;
  std::string mujoco_payload_body_name_;
  std::string mujoco_payload_site_name_;
  std::array<double, 3> mujoco_payload_initial_pos_;
  std::atomic<bool> payload_active_;
  double payload_blend_;
  double payload_blend_target_;
  double payload_blend_ramp_up_sec_;
  double payload_blend_ramp_down_sec_;
  bool payload_blend_enabled_;
  double j5_kp_;
  double j5_kd_;
  double latest_j5_command_;
  double latest_j5_position_;
  double latest_camera_position_;
  bool j5_command_received_;
  
  // 参考轨迹缓存
  std::vector<double> smoothed_positions_;     // 当前执行参考位置
  std::vector<double> smoothed_velocities_;    // 当前执行参考速度
  std::vector<double> smoothed_accelerations_; // 当前执行参考加速度
  
  // 速度前馈计算
  std::vector<double> filtered_cmd_velocities_;     // 一阶滤波后的指令速度
  std::vector<double> velocity_ff_stage2_;          // 二阶滤波中间量（最终发送的速度前馈）
  double max_acceleration_;                         // 最大加速度限制 (rad/s²)
  bool first_command_;                              // 首条指令标志
  double fallback_control_period_;                  // 异常周期回退值 (s)

  bool use_mock_hardware_;  // 是否使用仿真/假硬件

  // 外部反馈（如 MuJoCo）输入
  bool external_feedback_enabled_;                  // 是否启用外部关节状态反馈
  std::string external_feedback_topic_;             // 外部反馈话题
  double external_feedback_timeout_sec_;            // 外部反馈超时时间
  std::atomic<bool> external_feedback_received_;    // 是否收到过外部反馈
  std::chrono::steady_clock::time_point external_feedback_last_time_;  // 最近一次反馈时间
  std::vector<double> external_feedback_positions_;
  std::vector<double> external_feedback_velocities_;
  std::vector<double> external_feedback_efforts_;
  std::mutex external_feedback_mutex_;

  // ============ 零力矩模式与重力补偿 ============
  // 零力矩模式
  bool zero_torque_mode_;           // 是否启用零力矩模式
  double zero_torque_kd_;           // 零力矩模式阻尼系数

  // 低刚度位置 + 力矩调节模式
  bool low_stiffness_mode_;                  // 是否启用低刚度位置控制
  
  // 重力补偿参数（按关节：τ = sin_coeff * sin(θ) + cos_coeff * cos(θ) + offset）
  struct GravityCompParams {
    double sin_coeff;
    double cos_coeff;
    double offset;
  };
  std::vector<GravityCompParams> gravity_params_;
  bool gravity_comp_enabled_;       // 是否启用重力补偿
  double gravity_feedforward_ratio_;          // 重力补偿前馈比例 (0-1，默认 0.5=50%)
  
  // ============ Pinocchio 动力学模型 ============
  bool use_pinocchio_gravity_;      // 是否使用 Pinocchio 进行重力补偿
  bool use_pinocchio_inverse_dynamics_;  // 是否使用 Pinocchio 全逆动力学前馈

  std::string urdf_path_;           // URDF 文件路径
  pinocchio::Model pinocchio_model_;     // Pinocchio 模型
  pinocchio::Data pinocchio_data_;       // Pinocchio 数据
  pinocchio::Model pinocchio_loaded_model_;  // 带载 Pinocchio 模型
  pinocchio::Data pinocchio_loaded_data_;    // 带载 Pinocchio 数据
  bool pinocchio_initialized_;           // Pinocchio 是否初始化成功
  bool pinocchio_loaded_initialized_;    // 带载模型是否初始化成功
  std::vector<int> pinocchio_q_index_map_;  // 硬件关节 -> Pinocchio q 索引
  std::vector<int> pinocchio_v_index_map_;  // 硬件关节 -> Pinocchio v 索引
  bool pinocchio_mapping_ready_;            // 关节名映射是否就绪
  
  // 惯量参数（用于标定，完全替换 URDF 默认值）
  struct CalibratedInertiaParams {
    double mass;            // 质量 (kg)
    double com_x;           // 质心 x 坐标 (m)
    double com_y;           // 质心 y 坐标 (m)
    double com_z;           // 质心 z 坐标 (m)
  };
  std::vector<CalibratedInertiaParams> calibrated_inertia_params_;
  std::string inertia_config_path_;   // 惯量配置文件路径
  bool use_calibrated_inertia_;       // 是否使用标定后的惯量参数
  
  // 旧版缩放因子结构体（保留用于向后兼容）
  struct InertiaScaleParams {
    double mass_scale;      // 质量缩放系数
    double com_x_offset;    // 质心 x 偏移
    double com_y_offset;    // 质心 y 偏移
    double com_z_offset;    // 质心 z 偏移
  };
  std::vector<InertiaScaleParams> inertia_scale_params_;
  
  /**
   * @brief 初始化 Pinocchio 模型
   * @param urdf_path URDF 文件路径
   * @return 是否成功
   */
  bool initPinocchioModel(const std::string& urdf_path);
  bool buildPinocchioJointMapping();
  
  /**
   * @brief 从 YAML 配置文件加载标定后的惯量参数
   * @param config_path 配置文件路径
   * @return 是否成功
   */
  bool loadCalibratedInertia(const std::string& config_path);
  
  /**
   * @brief 将标定惯量参数应用到 Pinocchio 模型
   */
  void applyCalibratedInertiaToModel();
  
  /**
   * @brief 使用 Pinocchio 计算完整的重力补偿向量
   * @param positions 当前关节位置
   * @return 每个关节的重力补偿力矩
   */
  std::vector<double> computePinocchioGravity(const std::vector<double>& positions);
  std::vector<double> computePinocchioGravity(
    const std::vector<double>& positions, bool use_payload_model);

  /**
   * @brief 使用 Pinocchio 计算完整逆动力学力矩（RNEA）
   * @param positions 期望关节位置
   * @param velocities 期望关节速度
   * @param accelerations 期望关节加速度
   * @return 每个关节的逆动力学前馈力矩
   */
  std::vector<double> computePinocchioInverseDynamics(
    const std::vector<double>& positions,
    const std::vector<double>& velocities,
    const std::vector<double>& accelerations);
  std::vector<double> computePinocchioInverseDynamics(
    const std::vector<double>& positions,
    const std::vector<double>& velocities,
    const std::vector<double>& accelerations,
    bool use_payload_model);
  // 计算关节重力补偿力矩（简化模型：各关节相互独立）
  double computeGravityTorque(size_t joint_idx, double position);
  void publishPayloadActiveState();
  void vacuumActivateCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void payloadActiveCommandCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void j5CommandCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void dmSerialTxCallback(const arm_msgs::msg::CanFrame::SharedPtr msg);
  void configureLoadedPinocchioModel();
  
  // 零力矩模式服务回调
  void zeroTorqueModeCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  
  // 零力矩模式服务
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr zero_torque_srv_;
  
  // 调试发布器
  rclcpp::Node::SharedPtr debug_node_;
  std::thread spin_thread_;                 // 处理服务回调的线程
  std::atomic<bool> spin_thread_running_;   // 线程运行标志
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr hw_cmd_pub_;        // 控制器输出的指令
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr smoothed_cmd_pub_;  // 实际发送给电机的平滑指令
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gravity_torque_pub_; // 重力补偿力矩
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr velocity_ff_pub_;   // 速度前馈（发送给电机）
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr final_cmd_pub_;     // 最终下发控制包(position/velocity/torque，effort 为电机坐标系)
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr final_cmd_joint_frame_pub_;  // MuJoCo 使用的关节坐标系控制包
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr final_pd_pub_;      // 最终下发 PD 参数(kp/kd)
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr final_torque_ff_pub_; // 最终前馈力矩(joint frame)
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr j2_qd_ref_pub_;           // j2 当前参考速度
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr j2_qd_actual_pub_;        // j2 当前实际反馈速度
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr payload_blend_pub_;        // 负载渐变比例
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr laser_distance_pub_;       // 激光测距值
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr payload_active_pub_;         // 统一负载状态
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr j5_position_pub_;         // J5 实际位置
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr camera_position_pub_;     // Camera 实际位置
  rclcpp::Publisher<arm_msgs::msg::CanFrame>::SharedPtr dm_serial_rx_pub_;        // DM USB2CANFD 原始接收帧
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr mujoco_command_pub_; // MuJoCo 命令输出
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr external_feedback_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vacuum_activate_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr payload_command_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr j5_command_sub_;
  rclcpp::Subscription<arm_msgs::msg::CanFrame>::SharedPtr dm_serial_tx_sub_;

  // 外部反馈回调
  void externalFeedbackCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
};

}  // namespace rc_arm_hardware

#endif  // RC_ARM_HARDWARE__RC_ARM_HARDWARE_HPP_
