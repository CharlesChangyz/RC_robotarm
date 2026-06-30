# RC Arm 2 CAN 通信方案书

## 1. 背景与目标

本文说明当前工作空间中 `rc_arm_2` 实机任务的 CAN 通信设计，重点解释任务如何通过 DM serial/CAN 帧触发，以及动作完成后如何发出 ID 为 `0x500` 的 CAN 回执报文。

方案目标：

- 建立外部控制器、ROS 2 middleware、USB2CANFD 设备之间的通信边界。
- 明确 CAN 帧 ID、数据长度、数据内容和触发语义。
- 保持动作执行逻辑在 ROS 2 内部可维护，CAN 层只承担轻量命令和回执。
- 给出启动、调试和验证方法，便于实机联调。

## 2. 总体架构

当前链路分为三层：

```text
外部 CAN 控制器
    |
    | CAN/USB2CANFD 物理总线
    v
dmbot_serial::dm_serial_frame_bridge
    |
    | ROS 2 topic: /rc_arm_2/dm_serial_rx
    | ROS 2 topic: /rc_arm_2/dm_serial_tx
    v
rc_arm2_middleware::arm2_middleware
    |
    | action_sets.yaml / MoveIt / payload / vacuum / j5 / laser
    v
机械臂动作执行链路
```

各模块职责：

| 模块 | 职责 |
| --- | --- |
| `dm_serial_frame_bridge` | 连接 USB2CANFD 设备，把物理 CAN 帧转换为 ROS 2 `CanFrame` topic；同时把 ROS 2 `CanFrame` topic 写回 CAN 总线。 |
| `arm2_middleware` | 解析 CAN 命令帧，映射到动作集 ID，执行动作集，并在成功完成后发布完成回执帧。 |
| `DmSerialActionBridge` | 定义 CAN ID 到动作集的映射规则，以及完成回执帧的格式。 |
| `action_sets.yaml` | 定义动作集内部步骤，例如运动、吸附、延时、负载状态、传感器等待等。 |

## 3. CAN 通信协议

### 3.1 命令帧

外部控制器向机械臂发送标准 CAN 帧，使用 `0x4xx` 作为动作触发命令。

规则：

```text
action_set_id = can_id - 0x400
```

示例：

| CAN ID | 动作集 ID | 含义 |
| --- | ---: | --- |
| `0x401` | 1 | 触发动作集 1 |
| `0x402` | 2 | 触发动作集 2 |
| `0x40A` | 10 | 触发动作集 10 |

约束：

- 只接受标准帧，扩展帧会被忽略。
- 映射出的动作集 ID 必须存在于 `action_sets.yaml`。
- 如果配置了白名单 `dm_serial_allowed_action_set_ids`，则只有白名单内的动作集允许被 CAN 触发。
- middleware 忙碌时会忽略新的动作触发，避免动作集并发冲突。

当前代码位置：

- CAN ID 映射：`rc_moveit/rc_arm2_middleware/rc_arm2_middleware/dm_serial_action_bridge.py`
- CAN 接收处理：`rc_moveit/rc_arm2_middleware/rc_arm2_middleware/arm2_middleware_node.py`

### 3.2 完成回执帧

动作集由 CAN 命令触发，并且动作步骤全部执行完成后，middleware 会发出固定完成回执帧：

```text
CAN ID: 0x500
DLC:    8
Data:   00 00 00 00 00 00 00 00
Frame:  标准帧
FD:     true, 通过当前 CanFrame 字段标记为 CAN FD
```

该帧表示“动作集执行完成”。当前实现没有在 payload 中编码动作集 ID、错误码或状态码，所有 8 字节均为 `0x00`。

对应代码：

```python
def completion_frame(self) -> RawCanFrame:
    return RawCanFrame(can_id=self._complete_id, dlc=8, data=[0] * 8)
```

默认 `complete_id` 为 `0x500`，launch 中以十进制 `1280` 传入。

### 3.3 失败与异常语义

当前实现只有动作集执行到末尾时才发送 `0x500`。如果出现以下情况，不会发送完成回执：

- CAN 命令帧不是标准帧。
- `0x4xx` 映射出的动作集 ID 不存在。
- middleware 正在执行另一个动作集。
- 动作集执行失败并进入失败状态。
- `dm_serial_frame_bridge` 未启动，导致 `/rc_arm_2/dm_serial_tx` 没有被写到真实 CAN 总线。

如果比赛或上位机协议需要区分成功、失败、忙碌和未知命令，建议后续扩展为：

```text
CAN ID: 0x500
DLC:    8
Data[0]: action_set_id low byte
Data[1]: action_set_id high byte
Data[2]: status, 0=success, 1=failed, 2=busy, 3=unknown
Data[3]: error_code
Data[4..7]: reserved
```

当前版本为了兼容已有逻辑，仍保持全 0 完成帧。

## 4. ROS 2 消息格式

CAN 帧在 ROS 2 内部使用自定义消息 `arm_msgs/msg/CanFrame.msg` 表示：

```text
uint32 id
bool is_extended
bool is_remote
bool is_fd
uint8 dlc
uint8[64] data
```

Topic 约定：

| Topic | 方向 | 发布方 | 订阅方 | 用途 |
| --- | --- | --- | --- | --- |
| `/rc_arm_2/dm_serial_rx` | CAN -> ROS | `dm_serial_frame_bridge` | `arm2_middleware` | 接收外部 CAN 命令帧。 |
| `/rc_arm_2/dm_serial_tx` | ROS -> CAN | `arm2_middleware` | `dm_serial_frame_bridge` | 发布动作完成回执帧。 |

## 5. 发送流程

### 5.1 外部命令进入 ROS

1. 外部控制器在 CAN 总线上发送标准帧，例如 `0x401`。
2. USB2CANFD 设备接收该帧。
3. `dm_serial_frame_bridge` 通过 `usb_class` 回调拿到原始 CAN 帧。
4. `dm_serial_frame_bridge` 发布 `arm_msgs/msg/CanFrame` 到 `/rc_arm_2/dm_serial_rx`。
5. `arm2_middleware` 订阅 `/rc_arm_2/dm_serial_rx`，解析 `msg.id`。
6. `DmSerialActionBridge` 将 `0x401` 映射为动作集 `1`。
7. middleware 调用 `_try_start_action_set()` 执行动作集。

核心代码路径：

```text
dmbot_serial/src/dm_serial_frame_bridge.cpp
  publishRxFrame()

rc_arm2_middleware/arm2_middleware_node.py
  _on_dm_serial_frame()
  _try_start_action_set()

rc_arm2_middleware/dm_serial_action_bridge.py
  action_set_id_from_frame()
```

### 5.2 动作执行完成后发出 `0x500`

1. `arm2_middleware` 按 `action_sets.yaml` 执行动作集步骤。
2. 当 `run.step_index >= len(run.action_set.steps)` 时，动作集完成。
3. 如果该动作集的触发来源是 `dm_serial`，调用 `_send_dm_serial_completion()`。
4. `_send_dm_serial_completion()` 从 `DmSerialActionBridge` 获取完成帧。
5. `_publish_dm_serial_frame()` 构造 ROS 2 `CanFrame`：

   ```text
   id = 0x500
   dlc = 8
   data = [0,0,0,0,0,0,0,0]
   is_extended = false
   is_remote = false
   is_fd = true
   ```

6. middleware 发布该消息到 `/rc_arm_2/dm_serial_tx`。
7. `dm_serial_frame_bridge` 订阅 `/rc_arm_2/dm_serial_tx`。
8. `dm_serial_frame_bridge::onTxFrame()` 取出 `msg->id` 和 `msg->data`。
9. 调用 `usb_hw_->fdcanFrameSend(data, msg->id)` 写入 USB2CANFD 设备。
10. USB2CANFD 设备在 CAN 总线上发出 ID 为 `0x500` 的 CAN 帧。

核心代码路径：

```text
rc_arm2_middleware/arm2_middleware_node.py
  _send_dm_serial_completion()
  _publish_dm_serial_frame()

dmbot_serial/src/dm_serial_frame_bridge.cpp
  onTxFrame()
  usb_hw_->fdcanFrameSend(data, msg->id)
```

## 6. 启动配置

实机入口脚本：

```bash
./scripts/run_rc_arm_real.sh
```

当前默认值：

```bash
USE_ARM2_MIDDLEWARE=true
MIDDLEWARE_DM_SERIAL_BRIDGE_ENABLED=true
USE_DM_SERIAL_FRAME_BRIDGE=false
```

注意：`MIDDLEWARE_DM_SERIAL_BRIDGE_ENABLED=true` 只代表 middleware 会订阅和发布 DM serial 相关 ROS topic。真正把 `/rc_arm_2/dm_serial_tx` 写到 CAN 总线，需要启动 `dm_serial_frame_bridge`。

推荐实机联调启动方式：

```bash
USE_DM_SERIAL_FRAME_BRIDGE=true ./scripts/run_rc_arm_real.sh
```

可选参数：

```bash
./scripts/run_rc_arm_real.sh \
  use_dm_serial_frame_bridge:=true \
  dm_serial_bridge_sn:=9940F4E149D904A69924737E3DE6629F \
  dm_serial_bridge_nom_baud:=1000000 \
  dm_serial_bridge_dat_baud:=2000000 \
  middleware_dm_serial_command_base_id:=1024 \
  middleware_dm_serial_complete_id:=1280
```

参数说明：

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `use_dm_serial_frame_bridge` | `false` | 是否启动 USB2CANFD 原始帧桥。 |
| `middleware_dm_serial_bridge_enabled` | `true` | 是否启用 middleware 内部 CAN 触发逻辑。 |
| `middleware_dm_serial_command_base_id` | `1024` | 命令基 ID，即 `0x400`。 |
| `middleware_dm_serial_complete_id` | `1280` | 完成回执 ID，即 `0x500`。 |
| `dm_serial_bridge_nom_baud` | `1000000` | CAN 仲裁域波特率。 |
| `dm_serial_bridge_dat_baud` | `2000000` | CAN FD 数据域波特率。 |

## 7. 调试与验证

### 7.1 验证 ROS topic 是否发出 `0x500`

启动系统后，在另一个终端执行：

```bash
source /opt/ros/humble/setup.bash
source rc_moveit/install/setup.bash
source config/ros_domain.env

ros2 topic echo /rc_arm_2/dm_serial_tx
```

当 CAN 命令触发的动作集完成后，应看到类似：

```text
id: 1280
is_extended: false
is_remote: false
is_fd: true
dlc: 8
data:
- 0
- 0
- 0
- 0
- 0
- 0
- 0
- 0
```

### 7.2 不接 CAN 设备时模拟触发

可以直接向 `/rc_arm_2/dm_serial_rx` 发布一帧命令，模拟外部 CAN 输入：

```bash
ros2 topic pub --once /rc_arm_2/dm_serial_rx arm_msgs/msg/CanFrame \
"{id: 1025, is_extended: false, is_remote: false, is_fd: true, dlc: 8, data: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}"
```

其中 `id: 1025` 即 `0x401`，会触发动作集 `1`。

### 7.3 验证物理 CAN 总线是否发出

确保启动时带上：

```bash
USE_DM_SERIAL_FRAME_BRIDGE=true
```

观察日志：

```text
sent DM serial completion id=0x500 for action_set=...
sent dmserial frame id=0x500 dlc=8
```

第一行来自 `arm2_middleware`，表示 ROS topic 已发布；第二行来自 `dm_serial_frame_bridge`，表示已经调用 USB2CANFD 发送函数。

如果只有第一行，没有第二行，说明 middleware 发出了 topic，但物理 CAN 桥没有启动或没有订阅成功。

## 8. 实现细节

### 8.1 middleware 默认参数

`arm2_middleware` 默认声明：

```python
self.declare_parameter("dm_serial_rx_topic", "/rc_arm_2/dm_serial_rx")
self.declare_parameter("dm_serial_tx_topic", "/rc_arm_2/dm_serial_tx")
self.declare_parameter("dm_serial_command_base_id", 0x400)
self.declare_parameter("dm_serial_complete_id", 0x500)
```

### 8.2 完成帧只对 CAN 触发动作发送

动作集也可以通过 `/arm2/middleware/run_action_set` 这个 ROS topic 触发。为了避免普通 ROS 调用也向外部 CAN 控制器发回执，代码中只在触发来源为 `dm_serial` 时发送 `0x500`：

```python
if run.trigger_source == "dm_serial":
    self._send_dm_serial_completion(run.action_set.action_id)
```

### 8.3 USB2CANFD 发送边界

`dm_serial_frame_bridge` 的发送函数：

```cpp
void onTxFrame(const arm_msgs::msg::CanFrame::SharedPtr msg)
{
  const auto dlc = std::min<uint8_t>(msg->dlc, static_cast<uint8_t>(msg->data.size()));
  std::vector<uint8_t> data(msg->data.begin(), msg->data.begin() + dlc);
  usb_hw_->fdcanFrameSend(data, msg->id);
}
```

`fdcanFrameSend()` 的实现来自 `dmbot_serial` 链接的 `motor` 静态库依赖，当前工作区可见头文件和符号，但具体实现位于外部库 `rc_moveit/dmbot_serial/lib/libu2canfd.a` 链路中。

## 9. 风险与改进建议

当前方案已经能完成“CAN 命令触发动作集，成功后回发 `0x500`”的闭环，但仍有以下风险：

| 风险 | 影响 | 建议 |
| --- | --- | --- |
| `USE_DM_SERIAL_FRAME_BRIDGE` 默认关闭 | middleware 只发布 ROS topic，不会真正发到 CAN 总线 | 实机脚本可改为默认开启，或在启动日志中明确提示。 |
| `0x500` payload 全 0 | 外部控制器无法区分动作 ID、成功失败、忙碌等状态 | 后续扩展 Data 字段，加入 action ID 和 status。 |
| 忙碌或未知命令无回执 | 外部控制器可能一直等待 | 增加 `busy`、`unknown action`、`failed` 回执。 |
| 当前只过滤扩展帧 | 对 DLC、payload 内容没有协议校验 | 增加命令帧版本号或 magic byte，降低误触发概率。 |
| 完成回执只在动作走到末尾时发送 | 中途失败没有 CAN 层反馈 | `_fail_action_set()` 中增加失败回执。 |

建议的下一版状态帧：

```text
CAN ID: 0x500
DLC:    8
Data[0]: action_set_id & 0xFF
Data[1]: action_set_id >> 8
Data[2]: status
         0x00 success
         0x01 failed
         0x02 busy
         0x03 unknown_action
         0x04 rejected
Data[3]: error_code
Data[4]: trigger_can_id low byte
Data[5]: trigger_can_id high byte
Data[6]: sequence
Data[7]: checksum or reserved
```

## 10. 结论

当前 CAN 通信方案的核心是：

```text
外部发送 0x4xx -> middleware 执行动作集 xx -> 成功后回发 0x500
```

其中 `0x500` 完成回执由 `arm2_middleware` 生成，经 `/rc_arm_2/dm_serial_tx` 发布给 `dm_serial_frame_bridge`，最终由 `usb_hw_->fdcanFrameSend(data, msg->id)` 写入 USB2CANFD 设备并发到 CAN 总线。

实机联调时，务必确认 `dm_serial_frame_bridge` 已启动；否则只能在 ROS topic 层看到 `0x500`，物理 CAN 总线上不会出现该报文。
