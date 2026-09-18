# 项目删减与简化建议

目标：只讨论“删减、合并、去冗余、去闲置入口”。不把加锁、补状态位、补测试这类增强型修改列为主要建议。当前代码可用，所以每条都带“删除前确认点”。

审查依据：`rg` 引用搜索、`git ls-files` 跟踪文件检查、三个只读子代理分别审查 C++ 硬件链路、ROS launch/config、根 Python/MuJoCo/资源文件。

优先级说明：

- S0：明显可删或可合并，基本不改变主链路行为。
- S1：确认外部脚本/现场流程后删除，收益明显。
- S2：需要先定架构取舍，再整条删除或合并。
- HOLD：看起来冗余，但当前主链路仍在用，不建议直接删。

## S0：优先删减

| 位置 | 现状/含义 | 删减建议 | 删除前确认点 | 原因 |
|---|---|---|---|---|
| `rc_robotarm_mujoco/test.py:1-37` | 包内临时 viewer 脚本，硬编码 `/media/dust/...` 本机绝对路径，后半段是注释掉的旧实验代码 | 删除该文件 | 确认没人用 `python -m rc_robotarm_mujoco.test` | 会作为包模块安装；换机器必坏，不是正式测试 |
| `.marscode/deviceInfo.json:1` | 本机 IDE/设备标识文件被 Git 跟踪 | 从仓库删除，并忽略 `.marscode/` | 确认不作为团队共享配置 | 机器私有状态，不是源码 |
| `tmp/aha/dconf/user:1` | 本地 dconf 用户状态文件被 Git 跟踪 | 从仓库删除，并忽略 `tmp/` | 确认不是测试 fixture | 运行环境生成物，不应进仓库 |
| `rc_robotarm_mujoco/assets/robots/rc_arm/meshes/Link_3.STL.bak`，`rc_robotarm_mujoco/assets/robots/rc_arm_2/meshes/l1.STL.bak` | 两个 `.bak` mesh 被 Git 跟踪，大小约 13M 和 11M，搜索未见 XML/URDF 引用 | 删除 `.bak`；若是有效版本，应改正式文件名并被模型引用 | 确认不是人工保留的正确版本 | 备份文件占体积且语义不清 |
| `rc_moveit/rc_arm_description/urdf/rc_arm_2/rc_arm_2.pinocchio.urdf:1`，`:236`；`rc_moveit/rc_arm_description/CMakeLists.txt:20-52` | 源码树提交了生成 URDF，且含 `/home/dust/...` 绝对路径；CMake 已有生成和安装流程 | 删除源码树中的生成 URDF，只保留 xacro 源和 CMake 生成产物 | 确认没有未构建时直接读取 source tree 该 URDF 的流程 | 生成物会过期，绝对路径不可移植 |
| `requirements.txt:8`，`:11` | `empy==3.3.4` 和 `empy<4` 重复约束 | 保留一个约束 | 确认 ROS 2 Humble 当前是否必须固定 `3.3.4` | 依赖声明重复且容易漂移 |
| `.gitignore:40-41` | 忽略所有 `test/` 和 `**/test/` 目录，但仓库已有 ROS 包测试目录 | 删除这两条，或改成只忽略本地临时测试输出 | 确认不会把临时测试目录误加入 Git | 现在规则会让新测试默认不进版本控制 |
| `rc_moveit/dmbot_serial/include/dmbot_serial/protocol/damiao.h:261-269` | `current_motor_acc`、`desire_motor_*`、`min/max_motor_tor`、`mit_kp/kd` 基本无有效引用 | 删除这些 public 数组，仅保留实际读取的 `current_motor_pos/vel/tor` | 再跑一次 `rg "current_motor_acc|desire_motor_|min_motor_tor|max_motor_tor|mit_kp|mit_kd"` | public 状态面过大，维护者会误以为参与控制 |
| `rc_moveit/dmbot_serial/src/protocol/damiao.cpp:816-821`，`:837-843` | 反馈解析里有注释掉的旧逻辑；`auto m = motors[canID];` 未使用且会隐式插入 map | 删除注释块和未用变量 | 确认不需要恢复 `m->receive_data()` 老路径 | 局部死代码，且 `operator[]` 有副作用 |
| `rc_moveit/dmbot_serial/include/dmbot_serial/dm_motor_driver.hpp:35`，`rc_moveit/dmbot_serial/src/dm_motor_driver.cpp:223-236`，`rc_moveit/rc_arm_hardware/src/rc_arm_hardware.cpp:1054` | `MotorState.valid` 只要有 `motor_control_` 就固定设 true；硬件层再检查它 | 删除 `valid` 字段和检查；未连接时让 `readStates()` 返回空 vector | 确认 `rc_arm_hardware::read()` 能接受空状态列表 | 当前是“假有效性”标志，增加认知噪声 |
| `rc_moveit/rc_arm_hardware/include/rc_arm_hardware/rc_arm_hardware.hpp:212-219`，`rc_moveit/rc_arm_hardware/src/rc_arm_hardware.cpp:110` | `ControlMode` 枚举和 `control_mode_` 成员未参与行为 | 删除 | 确认不恢复 CSP/velocity/effort 模式 | 纯遗留状态 |
| `rc_moveit/rc_arm_hardware/include/rc_arm_hardware/rc_arm_hardware.hpp:140`，`rc_moveit/rc_arm_hardware/src/rc_arm_hardware.cpp:81`，`:235-236`，`:824-828`，`rc_moveit/rc_arm_description/config/rc_arm_2/rc_arm_2_hardware.real.yaml:37-39` | `dm_control_mode_` 读配置和打印，但 `DmMotorDriver::connect()` 固定 `damiao::MIT_MODE` | 删除成员、解析、日志和 YAML 配置项 | 确认不需要非 MIT 模式 | 这是无效果参数 |
done
| `rc_moveit/rc_arm_hardware/include/rc_arm_hardware/rc_arm_hardware.hpp:203-207`，`rc_moveit/rc_arm_hardware/src/rc_arm_hardware.cpp:339-345`，`:351-354`，`:1220-1221` | `last_cmd_positions_`、`cmd_velocities_` 只写不读；`max_velocity_` 实际不参与限速 | 删除这些成员和相关初始化/赋值/日志 | 确认没有计划把它们导出为调试接口 | 纯缓存冗余，不影响输出 |
| `rc_moveit/rc_arm_hardware/include/rc_arm_hardware/rc_arm_hardware.hpp:155`，`:357`，`rc_moveit/rc_arm_hardware/src/rc_arm_hardware.cpp:441`，`:938-941`，`:1060`，`:1115-1117` | 温度状态和 `/debug/motor_temperature` 目前固定写 25.0 | 若没有真实温度反馈消费，删除温度 state interface、缓存和 publisher | 确认外部没有订阅温度接口 | 固定假数据比没有数据更容易误导 |
| `rc_moveit/rc_arm_hardware/include/rc_arm_hardware/rc_arm_hardware.hpp:380-401`，`rc_moveit/rc_arm_hardware/src/rc_arm_hardware.cpp:372-386`，`:425-427`，`:1597-1687`，`rc_moveit/rc_arm_description/config/rc_arm_2/rc_arm_2_hardware.real.yaml:150-157` | 限位保护参数和函数存在，但 `write()` 中没有调用 | 删除限位保护成员、函数、参数读取、日志和 YAML `limit_*` | 确认不会马上把它接回控制路径 | 未接线代码提供不了保护，只增加配置噪声 |

## S1：确认现场流程后删除

| 位置 | 现状/含义 | 删减建议 | 删除前确认点 | 原因 |
|---|---|---|---|---|
| `rc_moveit/rc_arm_hardware/src/robstride_can_driver.cpp:1-465`，`rc_moveit/rc_arm_hardware/include/rc_arm_hardware/robstride_can_driver.hpp:38-295`，`rc_moveit/rc_arm_hardware/CMakeLists.txt:27-29` | Robstride 驱动源码存在，但库只编译 `rc_arm_hardware.cpp`；仓内只有 `MotorType/getMotorParams` 被用 | 删除 `.cpp` 和未用类；把 `MotorType/getMotorParams` 拆成小头文件 | 确认没有外部包 include/use `RobstrideCanDriver` | 不可达旧后端，误导维护 |
| `rc_moveit/dmbot_serial/src/usb2canfd_dm_node.cpp:10-314`，`rc_moveit/dmbot_serial/include/dmbot_serial/usb2canfd_dm_node.hpp:17-57`，`rc_moveit/dmbot_serial/CMakeLists.txt:70-80`，`:107` | 旧独立 USB2CAN 节点订阅 `/debug/final_*` 后下发；当前主链路由 `rc_arm_hardware` 直接 `writeCommands()` | 如果当前架构以 `rc_arm_hardware` 直连为准，删除该节点、头文件和 CMake target | 确认现场没有手动运行 `ros2 run dmbot_serial usb2canfd_dm_node_cpp` | 两条下发路径重复维护，误运行可能双发 |
done
| `rc_moveit/dmbot_serial/src/debugger_node.cpp:11-282`，`rc_moveit/dmbot_serial/include/dmbot_serial/debugger_node.hpp:13-37`，`rc_moveit/dmbot_serial/CMakeLists.txt:82-88` | `debugger_node_cpp` 发布旧 `robot_command`，仓内只有旧 `usb2canfd_dm_node_cpp` 消费 | 若删除旧 USB2CAN 节点，同步删除 debugger 节点 | 确认它不是保留的台架工具 | 调试链路依附旧入口 |
done
| `rc_moveit/dmbot_serial/src/test.cpp:13-143`，`rc_moveit/dmbot_serial/CMakeLists.txt:54-60` | `test_motor` 是硬编码 SN/CAN ID 的单电机测试程序，含大量实验注释 | 从默认构建/安装删除；必要时移入 `tools/` 或 `examples/` | 确认不作为硬件验收步骤 | 生产包混入一次性测试入口 |
done








| `rc_moveit/dmbot_serial/launch/dev_sn.launch:1-7`，`rc_moveit/dmbot_serial/launch/test_motor.launch:1-7` | ROS 1 风格 XML launch 与 `.launch.py` 重复 | 删除 XML 版本，只保留 ROS 2 Python launch | 确认没有脚本调用 XML launch | 同一工具两个入口 |

done

| `rc_moveit/dmbot_serial/include/dmbot_serial/protocol/damiao.h:216-218`，`rc_moveit/dmbot_serial/src/protocol/damiao.cpp:138-176` | `Motor_Control(..., simulation_only)` 构造函数仓内无调用 | 删除声明和实现 | 确认外部没有把 `motor` 库当 SDK 用 | 未用 API |
done

| `rc_moveit/dmbot_serial/include/dmbot_serial/protocol/damiao.h:221-239`，`rc_moveit/dmbot_serial/src/protocol/damiao.cpp:418-751` | 参数读写、切模式、单电机 MIT/POS/VEL 等旧通用 API 主要服务实验路径 | 若删除 `test_motor`，继续裁掉这些旧 API | 确认外部没有依赖通用达妙 SDK 能力 | 暴露面远大于当前生产路径 |
done

| `rc_moveit/arm_msgs/msg/MotorState.msg`，`MotorCommand.msg`，`RobotState.msg`，`RobotCommand.msg`，`MotorMitCmdTau.msg`，`RobotMitCmdTau.msg`，`PointCommand.msg`，`PathCommand.msg` | 多数旧消息只服务旧 `dmbot_serial` 独立节点或完全无仓内消费者 | 若删除旧 USB2CAN/debugger 链路，同步删除旧 msg，只保留 `CanFrame`、`Arm2TargetPoint`、`Arm2MotionExecution` 等活跃消息 | 确认没有外部包订阅/发布这些旧消息 | 消息包是公开接口，旧接口越多维护成本越高 |
done

| `rc_moveit/rc_arm_teleop/setup.py:33-39` | 注册了 `xbox_teleop_node`、`xbox_servo_node`、`joycon_*`、`master_slave_node`，但源码目录只有 `xbox_teleop_node_rc_arm_2.py` | 删除不存在模块对应 entry points，保留 `xbox_teleop_node_rc_arm_2` | 确认这些模块不是由 overlay 或生成步骤提供 | 安装后暴露坏命令 |
done

| `rc_moveit/rc_arm_description/launch/rc_arm_2_display.launch.py:8-20`，`rc_moveit/rc_arm_description/launch/rc_arm_2_control_main.launch.py:8-20` | 平铺 launch 只是 include 子目录入口 | 删除 wrapper，统一使用 `launch/rc_arm_2/display.launch.py` 和 `launch/rc_arm_2/control.launch.py` | 确认 README、脚本、外部命令不再调用旧名字 | 纯兼容别名 |
done

| `rc_moveit/rc_arm_moveit_config/launch/rc_arm_2_demo.launch.py:8-20` | 平铺 demo launch 只是 include `launch/rc_arm_2/demo.launch.py` | 删除 wrapper | 确认没有使用 `ros2 launch rc_arm_moveit_config rc_arm_2_demo.launch.py` 的流程 | 纯转发 |
done

| `rc_moveit/rc_arm_teleop/launch/rc_arm_2_sim_teleop.launch.py:8-20`，`rc_moveit/rc_arm_teleop/launch/rc_arm_2_real_teleop.launch.py:50-91` | 平铺 teleop launch 转发到子目录；real 版本还重复声明同一批参数 | 删除平铺 wrapper，统一使用 `launch/rc_arm_2/*.launch.py` | README 当前仍引用旧入口，需同步文档 | 两个入口维护同一功能 |
done

| `demo/rc_robotarm_demo.py:1`，`scripts/run_rc_arm_mujoco_bridge.sh:28` | 已把原 `rc_robotarm_demo_2.py` 实现合并到 `rc_robotarm_demo.py`；脚本入口保持不变 | 无需继续修改 | 若外部手动调用旧 `_2.py` 文件名需同步改命令 | 已处理：只保留一个真实入口 |
done

| `scripts/arm2_target_point_stdin_publisher.py` | stdin JSON 转 `Arm2TargetPoint` 的调试工具，仓内无主流程引用 | 无需继续修改 | 若外部还手动管道喂 JSON，改用 ROS 节点或恢复为本地工具 | 已处理：脚本已删除 |
done

| `rc_robotarm_mujoco/props/` | `Primitive` 抽象仓内未使用 | 无需继续修改 | 若外部 notebook/script 使用过 `rc_robotarm_mujoco.props`，需改为本地工具代码 | 已处理：`props` 包已删除 |
| `rc_robotarm_mujoco/assets/map/mocap_env.xml` | mocap 场景变体，默认 `StandardArena` 加载 `robocon2026.xml`，仓内未见入口 | 无需继续修改 | 若外部手动加载该 XML，需改用 `robocon2026.xml` 或恢复本地场景 | 已处理：未接入场景变体已删除 |
| `rc_robotarm_mujoco/assets/map/meshes/robocon2026.obj`，`robocon2026.mtl` | 完整导出 OBJ/MTL 与运行用拆分 mesh 并存；当前 `robocon2026.xml` 引用 `visual/` 和 `parts/` | 若拆分 mesh 是唯一运行资产，删除完整导出 OBJ/MTL | 确认建模流程不把它当源文件 | 避免源导出和运行资产双份维护 |
| `README.md:5-317`，`:324-600` | 中英文两套 README 内容基本重复，共约 600 行 | 单团队使用可删掉一套语言；若需要双语，拆成 `README.md` + `README.en.md` | 确认是否有对外英文文档需求 | 文档维护量翻倍 |

## S2：合并或按架构取舍

| 位置 | 现状/含义 | 简化建议 | 取舍点 | 原因 |
|---|---|---|---|---|
| `rc_moveit/dmbot_serial/src/dm_serial_frame_bridge.cpp:15-95`，`rc_moveit/rc_arm_hardware/src/rc_arm_hardware.cpp:800-806`，`:1023-1040`，`rc_moveit/rc_arm_moveit_config/launch/rc_arm_2_robot.launch.py:48`，`:150-163` | 独立 bridge 和硬件接口都实现 `/rc_arm_2/dm_serial_rx/tx` 原始帧桥；主 launch 默认关闭独立 bridge | 如果硬件接口总是运行，删除独立 `dm_serial_frame_bridge` target；如果需要无 ros2_control 桥接，则保留并明确文档 | 是否存在 `use_dm_serial_frame_bridge:=true` 且不启动硬件接口的模式 | 两套桥接会重复维护，且可能竞争同一 USB2CANFD |
| `rc_moveit/dmbot_serial/include/dmbot_serial/protocol/damiao.h:255-256`，`rc_moveit/dmbot_serial/src/protocol/damiao.cpp:382-395`，`rc_moveit/dmbot_serial/src/dm_motor_driver.cpp:88-99`，`:204-217`，`rc_moveit/rc_arm_hardware/src/rc_arm_hardware.cpp:452`，`:466-469`，`:800-806` | raw frame callback 链服务 DM serial action bridge | 如果不再用 DM 原始帧触发动作集，整条删除；如果还用，不建议局部删 | 当前 `arm2_middleware` 默认启用 `dm_serial_bridge_enabled`，需现场确认 | 这是跨包功能链，必须整条取舍 |
| `rc_moveit/rc_arm2_middleware/rc_arm2_middleware/arm2_middleware_node.py:119-123`，`:412-414`，`:519-532`；`rc_moveit/dmbot_serial/src/protocol/damiao.cpp:813-814`；`rc_moveit/rc_arm_hardware/src/rc_arm_hardware.cpp:1048`，`:1063-1066` | middleware 支持 laser wait，但当前 action_sets 只使用 `move_target_offset_noj5`；底层 laser 固定发布 1000 | 若不使用 `move_target_offset` 激光闭环，删除 laser topic、状态、等待逻辑和固定 1000 | 确认后续是否会恢复激光距离闭环 | 固定假数据和未用等待逻辑增加认知成本 |
| `rc_moveit/rc_arm_hardware/src/rc_arm_hardware.cpp:435-480`，`:1436-1511` | 硬件插件无条件创建大量 debug publisher；部分 topic 是 MuJoCo bridge 主链路输入 | 把“MuJoCo 必需 topic”和“纯 debug topic”分开；纯 debug topic 增加开关或删除 | `demo/rc_robotarm_demo.py:335-337` 依赖 `/debug/final_joint_command_joint_frame`、`/debug/final_pd_gains`、`/debug/final_joint_torque_ff` | debug 与功能桥接混在一起，导致不能直接删 |
| `rc_moveit/rc_arm_hardware/include/rc_arm_hardware/rc_arm_hardware.hpp:185-188`，`rc_moveit/rc_arm_hardware/src/rc_arm_hardware.cpp:198-212`；`rc_moveit/rc_arm_moveit_config/launch/payload_scene_sync.py` | `payload_box_size`、MuJoCo payload body/site/initial pos 在硬件插件内只读入不使用，场景同步脚本才关心 | 从 C++ 硬件插件 schema 中删除；这些字段留给 scene sync 配置 | 确认 xacro 不强制每个字段都传给硬件插件 | 硬件接口混入场景参数 |
| `rc_moveit/rc_arm_description/config/rc_arm_2/rc_arm_2_hardware.real.yaml:8-18`，`rc_moveit/rc_arm_description/config/rc_arm_2/rc_arm_2_hardware.mujoco.yaml:26-35` | real 配置保留 MuJoCo 字段，MuJoCo 配置保留 CAN 字段 | 拆成公共配置 + backend 专属配置，删除各后端无效字段 | xacro 和硬件插件当前是否要求统一 schema | 配置文件承担兼容职责，参数越写越多 |
| `rc_moveit/rc_arm_moveit_config/launch/rc_arm_2_robot.launch.py:49`，`rc_moveit/rc_arm_description/config/rc_arm_2/rc_arm_2_hardware.real.yaml:29`，`rc_moveit/rc_arm_description/config/rc_arm_2/rc_arm_2_hardware.mujoco.yaml:28` | USB2CANFD SN 默认值重复三处 | 保留一个权威位置，其余只透传或读取配置 | 独立 bridge 是否需要同一配置源 | 换设备时容易漏改 |
| `config/ros_domain.env:2`，`rc_moveit/rc_arm_moveit_config/launch/rc_arm_2_robot.launch.py:43` | `ROS_DOMAIN_ID=55` 默认值重复 | 保留 env 文件为权威来源；launch 不再硬编码 fallback，或反过来删除 env 文件 | 手动 `ros2 launch` 不 source env 时是否仍需要默认值 | 默认值重复会漂移 |
| `setup.py:9-15`，`requirements.txt:1-11` | Python 依赖在两个文件中维护，且不完全一致 | 保留一个权威依赖源；另一个引用或精简为说明用途 | 需要支持 `pip install .` 还是 `pip install -r requirements.txt` | 依赖漂移 |
| `scripts/run_rc_arm_mujoco.sh:20-90`，`scripts/run_rc_arm_real.sh:20-81` | 两个启动脚本大量重复 workspace 初始化、变量、echo、launch args | 合并公共 shell 片段，或做一个 `mode=mujoco/real` 的统一脚本，旧脚本变薄 wrapper | README 和 GUI 是否需要稳定三个按钮入口 | 参数一边改一边漏 |
| `rc_moveit/rc_arm_moveit_config/launch/rc_arm_2_robot.launch.py:30-108` | 总入口声明 70+ 个 launch 参数，许多是可选工具和调试开关 | 拆成基础 robot launch、middleware launch、debug/tools launch；不用的工具入口从默认总入口移出 | 当前一键启动是否必须包含所有功能 | 总入口过重，难判断主链路 |
| `rc_moveit/rc_arm_description/urdf/rc_arm_2/rc_arm_2.urdf.xacro`，`rc_moveit/rc_arm_description/urdf/rc_arm_2/rc_arm_2_ros2_control.xacro`，`rc_moveit/rc_arm_moveit_config/config/rc_arm_2/joint_limits.yaml` | 关节 position/velocity/effort 限位在 URDF、ros2_control、MoveIt 多处重复 | 建立一个权威来源，删除重复硬编码或由生成脚本产出 | MoveIt 是否仍需额外 acceleration/jerk 字段 | 关节限制是关键合同，重复维护风险高 |
| `rc_moveit/rc_arm_moveit_config/config/rc_arm_2/moveit_controllers.yaml`，`rc_moveit/rc_arm_description/config/rc_arm_2/rc_arm_2_controllers.yaml` | `arm_controller` 关节列表重复 | 合并为同一关节列表来源或生成其中一份 | MoveIt simple controller manager 的配置格式限制 | 关节顺序重复维护 |
| `rc_moveit/rc_arm_moveit_config/config/rc_arm_2/rc_arm_2.srdf:7-19` | `home` 与 `zero` group_state 都是 4 个关节 0.0；但 teleop 代码里 `home` 语义实际不是全零 | 不要直接删；先统一 “home” 和 “zero” 的真实含义，再删除重复状态 | `xbox_teleop_node_rc_arm_2.py` 明确有 home/zero 两套动作 | 名称重复会误导，但当前代码语义已经分叉 |

## HOLD：暂不建议删除

| 位置 | 现状/含义 | 结论 | 原因 |
|---|---|---|---|
| `rc_robotarm_mujoco/assets/map/meshes/kfs/**`，`parts/**`，`visual/robocon2026_*.obj` | `robocon2026.xml` 明确 include `kfs.xml` 并引用大量 visual/parts mesh | 暂不删 | 这些是当前场景运行资产 |
| `rc_moveit/rc_arm2_middleware/rc_arm2_middleware/dm_serial_action_bridge.py`，`rc_moveit/rc_arm2_middleware/rc_arm2_middleware/arm2_middleware_node.py:185-187` | DM serial action bridge 当前默认启用，服务动作集 CAN 触发 | 不单独删 | 必须和 raw frame topic 链一起取舍 |
| `rc_moveit/rc_arm_hardware/src/rc_arm_hardware.cpp:588-607`，`:1325-1337`，`:1529-1547` | zero torque 服务被 teleop 使用 | 暂不删 | `xbox_teleop_node_rc_arm_2.py` 多处调用 `/rc_arm/set_zero_torque_mode` |
| `rc_moveit/rc_arm_hardware/src/rc_arm_hardware.cpp:442-445`，`:1475-1511` | `/debug/final_joint_command_joint_frame`、`/debug/final_pd_gains`、`/debug/final_joint_torque_ff` 名字像 debug，但 MuJoCo demo 订阅它们 | 不按 debug 直接删 | 它们实际承担仿真桥接功能 |
| `rc_moveit/rc_arm2_middleware/config/action_sets.yaml` | 当前动作集中 `set_vacuum`、`set_payload_active`、`move_target_offset_noj5` 都被 middleware 支持 | 暂不删 | 是当前任务流程配置，不是闲置代码 |

## 建议删减顺序

1. 先删仓库垃圾和明显生成物：`.marscode/deviceInfo.json`、`tmp/aha/dconf/user`、`.bak` mesh、源码树 `rc_arm_2.pinocchio.urdf`、`rc_robotarm_mujoco/test.py`。
2. 再删局部无行为代码：`damiao.h` 闲置数组、`damiao.cpp` 未用变量/注释块、`MotorState.valid` 假标志、`dm_control_mode_`、`ControlMode`、未接线限位保护、固定 25 度温度。
3. 然后决定旧 DM 独立节点是否保留：若不保留，一次性删除 `usb2canfd_dm_node_cpp`、`debugger_node_cpp`、旧 `arm_msgs`、相关 README 说明。
4. 最后做结构合并：launch wrapper、real/mujoco 配置拆分、脚本合并、README 双语拆分。
