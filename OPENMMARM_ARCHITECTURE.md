# OpenMMArm 架构详解

本文档只描述 `openmmarm` 相关项目结构与运行机制。

## 1. 总体目标

OpenMMArm 软件栈将"规划、硬件通信"解耦，核心目标是：

- 上层（MoveIt / 自定义程序 / SDK）统一输出关节目标
- `openmmarm_hw` 直接持有 IO 驱动，在 `ros2_control` 的更新循环中与仿真或真机通信（频率由 `update_rate` 配置）
- 执行层支持 MuJoCo 仿真（进程内）和真机 MCU（UDP）两套链路

## 2. 包级架构

| 包名 | 角色 | 关键输入 | 关键输出 |
| --- | --- | --- | --- |
| `openmmarm_bringup` | 系统编排 | launch 参数 | 拉起整套节点 |
| `openmmarm_description` | 模型与 ros2_control 定义 | URDF/Xacro | `/robot_description` |
| `openmmarm_interfaces` | 消息定义 | - | `MotorCmd`/`MotorState` |
| `openmmarm_moveit_config` | 规划配置 | SRDF/YAML | `move_group` 规划服务 |
| `openmmarm_hw` | ros2_control 硬件插件（内嵌 IO） | 轨迹控制器命令 | 直接驱动仿真/真机 |

## 3. 分层与数据流

```text
[应用层]
MoveIt / 用户节点 / SDK程序
        |
        v
[接入层]
openmmarm_hw (ros2_control)      openmmarm_sdk
        |                              |
        v  IOInterface::sendRecv()     v  UDP ArmCmd/ArmState
[执行层]                          [执行层]
IOMujoco (进程内仿真)             MCU (真机 UDP)
IOUDP   (真机 UDP)
```

### 3.1 仿真链路

1. `move_group` 生成轨迹，交给 `JointTrajectoryController`
2. `openmmarm_hw::write()` 将目标关节角 + PD 增益填入 `LowLevelCmd`
3. `IOMujoco::sendRecv()` 在进程内执行 MuJoCo 单步仿真
4. `openmmarm_hw::read()` 从 `LowLevelState` 读取关节状态

### 3.2 真机链路

1. `openmmarm_hw::write()` 同样填充 `LowLevelCmd`
2. `IOUDP::sendRecv()` 通过 UDP 向 MCU 发送控制包，接收状态包
3. `openmmarm_hw::read()` 从 `LowLevelState` 读取关节状态

## 4. openmmarm_hw 内部架构

`openmmarm_hw` 是 `hardware_interface::SystemInterface` 插件，直接持有 `IOInterface`：

- 插件名：`openmmarm_hw/OpenMMArmHW`
- 由 `ros2_control_node` 加载
- 命令接口：`position`、`velocity`
- 状态接口：`position`/`velocity`/`effort`

### 4.1 IO 驱动层

- `IOInterface`：IO 抽象基类（`init()`、`sendRecv()`、`isConnected()`）
- `IOMujoco`：MuJoCo 仿真（进程内单步、URDF 预处理、可选 GLFW viewer）
- `IOUDP`：真机 UDP 通信（CRC32 校验）

### 4.2 生命周期行为

- `on_init`：从 `control_config.yaml` 读取通信模式、PD 增益、仿真/UDP 参数
- `on_configure`：根据 `communication` 创建 `IOMujoco` 或 `IOUDP`，调用 `init()`，首次 `sendRecv` 获取初始位置
- `on_activate`：用当前位置 + PD 增益填充 `LowLevelCmd`
- `on_deactivate`：置零 kp/kd/tau，保持当前位置
- `write`：填充 `LowLevelCmd`（q/dq/tau/kp/kd），调用 `ioInter_->sendRecv()`
- `read`：从 `LowLevelState` 读取 q/dq/tau_est

## 5. 通信接口与端口

### 5.1 hw ↔ MCU 通道（真机）

- 协议：UDP 二进制包 + CRC32
- 参数：`control_config.yaml` 中的 `udp.mcu_ip`、`udp.mcu_port`、`udp.local_port`
- 实现：`openmmarm_hw/src/io_udp.cpp`

### 5.2 hw ↔ MuJoCo（仿真）

- 协议：进程内直接调用
- 参数：`control_config.yaml` 中的 `sim.model_path`、`sim.viewer`
- 实现：`openmmarm_hw/src/io_mujoco.cpp`

### 5.3 SDK ↔ MCU 通道

- 协议：UDP + CRC32
- 默认端口：`8871`
- 数据结构定义：`openmmarm_sdk/include/openmmarm_sdk/openmmarm_arm_common.h`
  - `ArmCmd`：`mode/Kp/Kd/q_d/dq_d/tau_d/crc`
  - `ArmState`：`mode/q/dq/tau/temperature/errors/crc`

## 6. 配置入口与所有权

| 配置文件 | 作用 |
| --- | --- |
| `src/openmmarm_hw/config/control_config.yaml` | 通信模式、PD 增益、仿真/UDP 参数 |
| `src/openmmarm_description/urdf/openmmarm.ros2_control.xacro` | `openmmarm_hw` 插件和硬件参数 |
| `src/openmmarm_hw/config/openmmarm_hw_config.yaml` | `controller_manager` 与控制器插件配置 |
| `src/openmmarm_moveit_config/config/*.yaml` | MoveIt 规划/运动学/控制器映射参数 |

## 7. 启动时序

### 7.1 仿真（推荐）

```bash
ros2 launch openmmarm_bringup sim_arm.launch.py
```

时序：

1. 启动 `robot_state_publisher`
2. 启动 `openmmarm_hw`（内嵌 MuJoCo 仿真）
3. 延时启动 `move_group`（可选）
4. 启动 `rviz2`（可选）

### 7.2 真机

将 `control_config.yaml` 中 `communication` 改为 `"UDP"` 并配置 MCU 地址，然后：

```bash
ros2 launch openmmarm_bringup real_arm.launch.py
```
