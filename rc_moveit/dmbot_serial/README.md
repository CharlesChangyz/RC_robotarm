# dmbot_serial - DM达妙电机串行通信驱动包

## 功能概述

`dmbot_serial` 是一个 ROS 2 驱动包，用于与达妙（DamaiO）电机进行通信和控制。该包提供了通过 USB-CAN-FD 适配器与多个 DM4310/DM4340 电机进行实时控制的功能。

### 核心功能

1. **多电机控制**：支持通过 CAN 总线同时控制多达 6 个以上的达妙电机
2. **多种控制模式**：支持 MIT 模式、位置-速度模式、速度模式、位置-力模式等
3. **实时反馈**：发布当前电机的关节状态（位置、速度、力矩）
4. **键盘调试**：提供交互式命令行界面用于测试和调试
5. **USB-CAN 通信**：通过 libusb 和 CAN-FD 协议实现与电机的通信

---

## 项目结构

```
dmbot_serial/
├── CMakeLists.txt              # 构建配置文件
├── package.xml                 # ROS 2 包定义
├── README.md                   # 本文件
├── include/dmbot_serial/       # 头文件目录
│   ├── debugger_node.hpp       # 调试命令发布器
│   ├── usb2canfd_dm_node.hpp   # 电机驱动主节点
│   └── protocol/               # 通信协议
│       ├── damiao.h            # 达妙电机协议定义
│       └── usb_class.h         # USB 通信类
├── src/                        # 源文件目录
│   ├── debugger_node.cpp       # 调试节点实现
│   ├── usb2canfd_dm_node.cpp   # 电机驱动节点实现
│   ├── dev_sn.cpp              # USB 设备序列号查询工具
│   ├── test.cpp                # 电机功能测试程序
│   └── protocol/               # 协议实现
│       └── damiao.cpp          # 达妙电机协议实现
├── launch/                     # ROS 2 启动文件
│   ├── dev_sn.launch.py        # 序列号查询启动脚本
│   ├── test_motor.launch.py    # 电机测试启动脚本
│   └── ...
└── lib/                        # 外部库
    └── libu2canfd.a            # USB-CAN-FD 通信库
```

---

## 组件说明

### 1. `usb2canfd_dm_node` - 电机驱动主节点

**功能**：核心驱动节点，负责与电机硬件通信

**工作流程**：
- 初始化 USB-CAN-FD 适配器和电机配置
- 订阅 ROS 2 话题 `robot_command`（接收控制命令）
- 将命令转换为 CAN 报文并发送给电机
- 定期发布电机的关节状态到 `joint_state` 话题

**可配置参数**（通过 ROS 2 参数）：
- `sn` (string): USB-CAN-FD 设备序列号（默认：9940F4E149D904A69924737E3DE6629F）
- `nom_baud` (int): CAN 标准帧波特率（默认：1000000）
- `dat_baud` (int): CAN 数据帧波特率（默认：2000000）
- `motor_ids` (list): 电机 CAN ID 列表（默认：[1, 2, 3, 4, 5, 6]）
- `master_ids` (list): 主机 ID 列表（对应各电机）
- `motor_types` (list): 电机类型（1=DM4310, 3=DM4340）
- `control_mode` (int): 控制模式（0=MIT, 1=位置-速度, 2=速度, 3=位置-力）

**话题接口**：
- 订阅：`robot_command` - 接收电机控制命令（arm_msgs::msg::RobotCommand）
- 发布：`joint_state` - 发送关节状态反馈（sensor_msgs::msg::JointState）

### 2. `debugger_node` - 调试命令发布器

**功能**：交互式键盘控制界面，用于测试和调试

**工作流程**：
- 启动时会显示可用命令列表
- 监听键盘输入（1-9, 0 分别对应 10 个预定义命令）
- 每个命令定义不同的电机目标位置和 PID 参数
- 以 100ms 的周期发布控制命令到 `robot_command` 话题

**预定义命令示例**：
| 命令 | 功能 |
|------|------|
| 1 | 所有电机回到零位，启用电机 |
| 2 | 电机 2 轻微旋转测试 |
| 3 | 电机 3 轻微旋转测试 |
| 4 | 电机 4 轻微旋转测试 |
| ... | ... |

**特点**：
- 包含安全的关节限位检查（防止电机超出安全范围）
- 实时调整的 PID 增益参数
- 实时摇摆运动控制（swing phase）

### 3. `dev_sn` - USB 设备查询工具

**功能**：扫描并列出所有已连接的 USB-CAN-FD 适配器

**使用场景**：
- 获取 USB-CAN-FD 设备的序列号
- 确认硬件是否正确连接
- 查看设备 VID/PID 等信息

**输出示例**：
```
U2CANFD_DEV 0:
  VID: 0x34b7
  PID: 0x6877
  SN: 9940F4E149D904A69924737E3DE6629F
```

### 4. `test_motor` - 电机功能测试程序

**功能**：独立的电机测试程序（不依赖 ROS 2）

**用途**：
- 验证电机硬件功能
- 测试通信协议
- 调试电机参数

---

## 使用方法

### 前置条件

1. **硬件要求**：
   - USB-CAN-FD 适配器（如 DM-CANFD）
   - 达妙电机（DM4310 或 DM4340）
   - Linux 系统

2. **软件依赖**：
   - ROS 2（Humble 或更新版本）
   - libusb-1.0-dev
   - arm_msgs 包

3. **驱动程序**：
   ```bash
   sudo apt-get install libusb-1.0-0-dev
   ```

### 安装和编译

1. **克隆包到 ROS 2 工作空间**：
   ```bash
   cd ~/ros2_ws/src
   # dmbot_serial 应该已在工作空间中
   ```

2. **编译包**：
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select dmbot_serial
   ```

3. **设置环境**：
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

### 基本使用

#### 1. 查询 USB 设备序列号

```bash
# 使用 launch 文件
ros2 launch dmbot_serial dev_sn.launch.py

# 或直接运行程序
ros2 run dmbot_serial dev_sn
```

运行后会显示所有连接的 USB-CAN-FD 设备及其序列号。

#### 2. 启动电机驱动和调试器

```bash
# 启动电机驱动节点
ros2 run dmbot_serial usb2canfd_dm_node_cpp

# 在另一个终端启动调试器
ros2 run dmbot_serial debugger_node_cpp
```

#### 3. 使用调试器控制

按键盘数字键 1-9, 0 来切换不同的控制命令：
- 按 `1`：启用电机，所有关节回零
- 按 `2-4`：测试各个关节的单独运动
- 按 `0`：发送第 10 个预定义命令
- 按 `Ctrl+C`：退出

#### 4. 使用自定义参数启动

```bash
ros2 run dmbot_serial usb2canfd_dm_node_cpp \
  --ros-args \
  -p sn:="YOUR_DEVICE_SN" \
  -p motor_ids:=[1,2,3,4,5,6] \
  -p control_mode:=0
```

### 高级配置

#### 修改电机参数

编辑源文件 [src/usb2canfd_dm_node.cpp](src/usb2canfd_dm_node.cpp) 中的初始化参数：

```cpp
this->declare_parameter<std::string>("sn", "YOUR_SERIAL_NUMBER");
this->declare_parameter<int64_t>("control_mode", 0);  // 0=MIT, 1=PosVel, 2=Vel, 3=PosForce
```

#### 自定义控制命令

编辑 [src/debugger_node.cpp](src/debugger_node.cpp) 中的 `create_commands()` 方法来定义新的控制序列。

#### PID 增益调整

在 [include/dmbot_serial/debugger_node.hpp](include/dmbot_serial/debugger_node.hpp) 中调整：

```cpp
motor_gains_ = {
    std::pair<float, float>{0.001F, 0.005F},  // 电机0的Kp, Kd
    {5.0F, 1.0F},                              // 电机1的Kp, Kd
    ...
};
```

---

## 通信协议

### ROS 2 消息格式

**控制命令格式** (`arm_msgs::msg::RobotCommand`)：
```
bool is_enable
MotorCommand[] motor_command (6 个元素)
  - float q      # 目标位置 (rad)
  - float dq     # 目标速度 (rad/s)
  - float tau    # 目标力矩 (Nm)
  - float kp     # 比例增益
  - float kd     # 微分增益
```

**关节状态反馈** (`sensor_msgs::msg::JointState`)：
```
string[] name            # 关节名称
float[] position         # 当前位置 (rad)
float[] velocity         # 当前速度 (rad/s)
float[] effort           # 当前力矩 (Nm)
```

### CAN 通信协议

- 使用大妙电机的 MIT 控制协议
- 标准帧波特率：1 Mbps
- 数据帧波特率：2 Mbps
- CAN FD 格式：12 字节数据帧

---

## 故障排除

### 问题 1：无法找到 USB 设备

**症状**：`Failed to initialize motor driver`

**解决方案**：
1. 检查 USB 设备是否连接：
   ```bash
   ros2 run dmbot_serial dev_sn
   ```
2. 检查设备权限：
   ```bash
   lsusb  # 查看是否列出设备（VID: 34b7, PID: 6877）
   ```
3. 添加 udev 规则允许用户访问设备（需要 root 权限）

### 问题 2：电机未响应

**症状**：电机不动或只有部分电机响应

**排查步骤**：
1. 验证 CAN ID 是否正确
2. 检查电机是否上电
3. 检查 CAN 线路连接
4. 查看 debug 日志：
   ```bash
   ros2 run dmbot_serial usb2canfd_dm_node_cpp --ros-args --log-level DEBUG
   ```

### 问题 3：键盘输入无响应

**症状**：按键无效

**原因**：调试器线程可能被阻塞

**解决方案**：
- 确保终端焦点在发布调试器的终端上
- 重新启动调试器节点

### 问题 4：波特率或协议错误

**症状**：`CAN communication error`

**排查**：
1. 确认电机型号（DM4310 vs DM4340）
2. 检查配置中的控制模式是否匹配
3. 验证 CAN 波特率设置

---

## 开发和扩展

### 添加新的电机类型

在 [src/usb2canfd_dm_node.cpp](src/usb2canfd_dm_node.cpp) 中添加新的电机类型判断：

```cpp
if (motor_type_val == 3) {
    motor_type = damiao::DM4340;
} else if (motor_type_val == 4) {
    // 新增电机类型
    motor_type = damiao::DM_NEW_TYPE;
}
```

### 自定义通信协议

扩展 [include/dmbot_serial/protocol/damiao.h](include/dmbot_serial/protocol/damiao.h) 中的电机通信接口。

### 添加新的控制话题

在 [src/usb2canfd_dm_node.cpp](src/usb2canfd_dm_node.cpp) 中添加新的订阅器：

```cpp
command_subscriber_v2_ = this->create_subscription<CustomMsg>(
    "robot_command_v2", 10,
    std::bind(&Usb2canfdDMNode::command_callback_v2, this, std::placeholders::_1));
```

---

## 性能指标

- **通信频率**：可达 100 Hz 以上（取决于电机数量）
- **控制延迟**：~10 ms（USB 通信 + CAN 传输）
- **最大电机数量**：8+ 个（受 CAN 总线带宽限制）
- **支持的数据速率**：500 kbps, 1 Mbps, 2 Mbps

---

## 相关资源

- **达妙电机官网**：https://www.damiao.com
- **ROS 2 文档**：https://docs.ros.org/
- **CAN-FD 规范**：ISO 11898-2

---

## 许可证

遵循项目统一的许可证（待定）

---

## 维护和支持

如有问题或建议，请联系项目维护者。

**最后更新**：2026年3月29日
