# openmmarm_hw

OpenMMArm 的 `ros2_control` 硬件接口插件。内嵌 IO 驱动层（MuJoCo 仿真 / UDP 真机），在 `ros2_control` 更新循环中直接与执行层通信，无需额外的中间节点。控制频率由 `update_rate` 配置（默认 250Hz，impedance 模式建议更高）。

---

## 1. 整体架构

```text
┌──────────────────────────────────────────────────────────┐
│  MoveIt / 用户节点                                       │
│  (FollowJointTrajectory Action)                          │
└───────────────────────┬──────────────────────────────────┘
                        │ position / velocity 命令
                        v
┌──────────────────────────────────────────────────────────┐
│  ros2_control (controller_manager)                        │
│  ┌──────────────────────────────────────────────────┐    │
│  │  JointTrajectoryController                       │    │
│  │  插值并输出每周期的 q_d / dq_d                    │    │
│  └──────────────────────┬───────────────────────────┘    │
│                         │                                │
│  ┌──────────────────────v───────────────────────────┐    │
│  │  OpenMMArmHW  (hardware_interface::SystemInterface)   │
│  │                                                  │    │
│  │  write():                                        │    │
│  │    LowLevelCmd ← { q_d, dq_d, 0, Kp, Kd }      │    │
│  │    ioInter_->sendRecv(&lowCmd_, &lowState_)      │    │
│  │                                                  │    │
│  │  read():                                         │    │
│  │    hw_states ← { lowState_.q, .dq, .tau_est }    │    │
│  └──────────────────────┬───────────────────────────┘    │
└─────────────────────────┼────────────────────────────────┘
                          │ IOInterface::sendRecv()
              ┌───────────┴───────────┐
              v                       v
┌─────────────────────┐  ┌─────────────────────┐
│  IOMujoco            │  │  IOUDP               │
│  进程内 MuJoCo 仿真  │  │  UDP 二进制包 + CRC32 │
│  (GLFW 可视化)       │  │  与真机 MCU 通信      │
└─────────────────────┘  └─────────────────────┘
```

### 1.1 关键设计决策

- **无中间节点**：`OpenMMArmHW` 直接持有 `IOInterface`，不经过额外的 UDP 转发或 FSM 节点。MuJoCo 仿真运行在 `ros2_control_node` 进程内，消除了序列化 + 反序列化 + CRC + FSM 透传的开销。
- **统一控制律**：无论仿真还是真机，底层执行的控制律完全一致（见第 2 节）。
- **配置驱动**：通信模式（SIM/UDP）、PD 增益、仿真参数全部从 `control_config.yaml` 读取，切换仿真/真机只需改一行配置。

---

## 2. 控制模型

### 2.1 关节力矩控制律

每个控制周期（$dt = 1/f_{ctrl}$），`write()` 将以下参数打包为 `LowLevelCmd` 发往执行层：

| 字段 | 含义 | 单位 | 来源 |
| --- | --- | --- | --- |
| $q_d^{(i)}$ | 期望关节位置 | rad | `JointTrajectoryController` 输出 |
| $\dot{q}_d^{(i)}$ | 期望关节速度 | rad/s | `JointTrajectoryController` 输出 |
| $\tau_d^{(i)}$ | 前馈力矩 | N·m | 当前固定为 0（预留接口） |
| $K_p^{(i)}$ | 位置刚度 | N·m/rad | `control_config.yaml` |
| $K_d^{(i)}$ | 速度阻尼 | N·m·s/rad | `control_config.yaml` |

执行层（MCU 或 MuJoCo）按以下控制律计算关节力矩：

$$ \tau_i = K_p^{(i)} \left( q_d^{(i)} - q^{(i)} \right) + K_d^{(i)} \left( \dot{q}_d^{(i)} - \dot{q}^{(i)} \right) + \tau_d^{(i)} $$

其中 $q^{(i)}$、$\dot{q}^{(i)}$ 为关节实际位置/速度（编码器或仿真读数）。该控制律与真机 MCU 固件完全一致。

### 2.2 Position 模式——高刚度位置跟踪

Position 模式下，$\tau_d = 0$，控制律简化为：

$$\tau = K_p (q_d - q) + K_d (\dot{q}_d - \dot{q})$$

使用高 $K_p$（高刚度）+ 匹配的 $K_d$（接近临界阻尼），关节表现为强力弹簧，被牢牢"拉"到 $q_d$，外力难以推动关节偏离轨迹。适合 MoveIt 轨迹执行、精密定位等无接触运动场景。


### 2.3 Impedance 模式——低刚度柔顺控制

完整的关节空间阻抗控制律为：

$$\tau = \underbrace{M(q)\ddot{q}_d + C(q, \dot{q})\dot{q}_d + g(q)}_{\text{前馈/补偿}} + \underbrace{K_d(q_d - q) + D_d(\dot{q}_d - \dot{q})}_{\text{阻抗项}}$$

其中：

- $M(q)$ — 关节空间惯量矩阵
- $C(q, \dot{q})$ — 科里奥利/离心力矩阵
- $g(q)$ — 重力补偿向量
- $K_d$ — 期望刚度矩阵（对角，N·m/rad）
- $D_d$ — 期望阻尼矩阵（对角，N·m·s/rad）

前馈/补偿项通过动力学模型抵消惯性、科里奥利和重力的影响，使关节对外力的响应**仅由阻抗项决定**——即呈现用户期望的刚度 $K_d$ 和阻尼 $D_d$。

映射到 2.1 节的通用控制律：

$$\tau_d = M(q)\ddot{q}_d + C(q, \dot{q})\dot{q}_d + g(q), \quad K_p = K_d, \quad K_d^{ctrl} = D_d$$

使用低 $K_d$（低刚度）+ 适当 $D_d$（保证稳定），关节表现为软弹簧。人可以用手轻松推动机械臂，适合接触操作、碰撞安全、拖动示教。

**典型参数**：

| 关节 | $K_d$ (N·m/rad) | $D_d$ (N·m·s/rad) |
| --- | --- | --- |
| J1-J4（肩/肘） | 25 | 10 |
| J5-J6（腕） | 15 | 5 |

### 2.4 两种模式的对比总结

```text
                Position                    Impedance
  ┌─────────────────────────────┬─────────────────────────────┐
  │  高 Kp，高 Kd               │  低 Kd，适中 Dd              │
  │                             │  + 动力学前馈补偿             │
  │                             │                             │
  │  q ≈ q_d                    │  q = q_d + τ_ext/Kd         │
  │  (精确跟踪)                  │  (柔顺偏移)                  │
  │                             │                             │
  │  外力难以推动                 │  外力可轻松推动              │
  │  碰撞力大                    │  碰撞力小                    │
  │                             │                             │
  │  适合:                      │  适合:                       │
  │  - MoveIt 轨迹执行           │  - 拖动示教                  │
  │  - 精密定位                  │  - 接触操作 (擦拭/装配)       │
  │  - 无接触运动                │  - 人机协作                  │
  │                             │  - 碰撞安全                  │
  └─────────────────────────────┴─────────────────────────────┘
```

### 2.5 MuJoCo 仿真中的隐式 PD 控制

#### 2.5.1 General Actuator 与隐式积分

MuJoCo 仿真中使用 `<general>` actuator 配合 `biastype="affine"` 实现 PD 控制。每个关节对应的 actuator 定义为：

```xml
<general name="actuator_J1" joint="J1" gear="1"
         gaintype="fixed" gainprm="1"
         biastype="affine" biasprm="0 -Kp -Kd"
         ctrllimited="true" ctrlrange="-200 200"/>
```

MuJoCo 的 `<general>` actuator 力公式：

$$\tau = \underbrace{g \cdot u}_{\text{gain} \times \text{ctrl}} + \underbrace{b_0 + b_1 \cdot q + b_2 \cdot \dot{q}}_{\text{affine bias}}$$

代入参数（$g = 1$，$b_0 = 0$，$b_1 = -K_p$，$b_2 = -K_d$，每步动态更新）：

$$u_i = K_p q_d + K_d \dot{q}_d + \tau_d \qquad \text{(常数驱动项)}$$

展开：

$$\tau = 1 \cdot (K_p q_d + K_d \dot{q}_d + \tau_d) + 0 + (-K_p) \cdot q + (-K_d) \cdot \dot{q}$$

$$= K_p (q_d - q) + K_d (\dot{q}_d - \dot{q}) + \tau_d$$

与 2.1 节的控制律数学完全等价。

#### 2.5.2 为什么不直接计算力矩

如果在 C++ 中手动计算 $\tau = K_p(q_d - q) + K_d(\dot{q}_d - \dot{q}) + \tau_d$ 然后写入 `ctrl`（即使用 `<motor>` actuator），MuJoCo 将其视为**外部常量力**，采用**显式积分**。此时的数值稳定性条件为：

$$dt < \frac{2}{\sqrt{K_p / J}}$$

| 示例 | $K_p$ | $J$ | 最大 $dt$ | 最低频率 |
| --- | --- | --- | --- | --- |
| 大惯量关节 | 100 | 0.01 | 0.02 s | 50 Hz |
| 小惯量关节 | 100 | 0.001 | 0.0063 s | 159 Hz |

小惯量关节的稳定性条件更苛刻。超过临界步长时会产生**数值振荡**（关节疯狂抖动）。

而使用 `<general biastype="affine">`，$b_1 \cdot q$ 和 $b_2 \cdot \dot{q}$ 被 MuJoCo 的 `implicitfast` 积分器识别并做**隐式积分**——即在求解下一时刻的 $q$、$\dot{q}$ 时，已经考虑了 $K_p$、$K_d$ 对状态变化的反馈：

$$\left(J + K_d \cdot dt + K_p \cdot dt^2\right) \dot{q}(t+dt) = J \cdot \dot{q}(t) + \left(\tau_{ext} - K_p \cdot q(t)\right) dt$$

$K_p$、$K_d$ 参与方程左侧的系数矩阵求解，从根本上消除了数值振荡，允许使用任意步长。

#### 2.5.3 仿真流程

`IOMujoco::sendRecv()` 每次调用执行以下步骤：

1. **检测有效指令**：首次收到非零 $K_p / K_d / \tau_d$ 时标记仿真开始
2. **更新 actuator 参数**：
   - `model->actuator_biasprm[i*mjNBIAS + 1]` $= -K_p$（隐式刚度）
   - `model->actuator_biasprm[i*mjNBIAS + 2]` $= -K_d$（隐式阻尼）
   - `data->ctrl[i]` $= K_p q_d + K_d \dot{q}_d + \tau_d$（常数驱动项）
3. **执行 `mj_step()`**：单步仿真，步长 $= 1 / f_{ctrl}$
4. **状态读取**：从 `data->qpos/qvel/qacc/actuator_force` 提取状态

```text
ros2_control 循环 (周期 = 1/update_rate)
┌──────────────────────────────────────────────────┐
│  write():                                        │
│    biasprm[1] = −Kp, biasprm[2] = −Kd           │
│    ctrl[i] = Kp·q_d + Kd·dq_d + τ_d             │
│    mj_step(model, data)              ← 单步      │
│                                                  │
│  read():                                         │
│    states ← qpos, qvel, actuator_force           │
└──────────────────────────────────────────────────┘
```

### 2.6 URDF 预处理与模型加载

`IOMujoco::init()` 执行两步加载策略：

1. **URDF 预处理**：收集所有 `package://` 格式的 mesh 引用，通过 `ament_index` 解析为绝对路径，创建符号链接到模型目录，将 URDF 中的 mesh filename 简化为 basename
2. **两步加载**：先以 URDF 加载（此时 `nu=0`，无 actuator），导出为原生 MJCF，在 MJCF 中注入 `<actuator>` 段（每个非 fixed joint 一个 `<general biastype="affine">` actuator），重新加载

### 2.7 UDP 真机通信协议

`IOUDP` 使用与 MCU 固件约定的二进制协议：

**发送包 (UDPSendCmd)**

```text
偏移   字段                 类型           说明
0x00   head[2]              uint8[2]       固定 {0xFE, 0xEF}
0x02   mode[6]              uint8[6]       关节控制模式
0x08   q[6]                 float[6]       期望位置 (rad)
0x20   dq[6]                float[6]       期望速度 (rad/s)
0x38   tau[6]               float[6]       前馈力矩 (N·m)
0x50   kp[6]                float[6]       位置增益
0x68   kd[6]                float[6]       速度增益
0x80   crc                  uint32         CRC-32 (IEEE 802.3)
```

**接收包 (UDPRecvState)**

```text
偏移   字段                 类型           说明
0x00   head[2]              uint8[2]       固定 {0xFE, 0xEF}
0x02   mode[6]              uint8[6]       关节当前模式
0x08   q[6]                 float[6]       实际位置 (rad)
0x20   dq[6]                float[6]       实际速度 (rad/s)
0x38   ddq[6]               float[6]       实际加速度 (rad/s²)
0x50   tau_est[6]           float[6]       力矩估计 (N·m)
0x68   temperature[6]       int8[6]        电机温度 (°C)
0x6E   error_state[6]       uint8[6]       错误状态
0x74   crc                  uint32         CRC-32 (IEEE 802.3)
```

初始化时发送空指令并等待 MCU 响应，最多重试 10 次（每次 200ms 间隔）。

---

## 3. 生命周期

`OpenMMArmHW` 遵循 `ros2_control` 的 Managed Node 生命周期：

```text
  on_init()         on_configure()        on_activate()
[UNCONFIGURED] ────> [INACTIVE] ────────> [ACTIVE]
                         ^                    │
                         │    on_deactivate() │
                         └────────────────────┘
```

### 3.1 on_init()

1. 从 URDF 读取 `has_gripper` 参数，确定关节数量（6 或 7）
2. 从 `control_config.yaml` 读取：
   - `communication`：通信模式 (SIM / UDP)
   - `control_mode`：控制模式 (position / impedance)
   - `pd_gains`：对应模式下的 Kp/Kd 数组
   - `sim.*`：仿真配置
   - `udp.*`：UDP 配置
3. 初始化状态和命令向量 (position/velocity/effort)

### 3.2 on_configure()

1. 根据 `communication` 创建 IO 驱动实例：
   - `"SIM"` → `IOMujoco(model_path, 0.004, viewer)`
   - `"UDP"` → `IOUDP(mcu_ip, mcu_port, local_port)`
2. 调用 `ioInter_->init()` 初始化驱动
3. 首次 `sendRecv()` 获取初始关节位置，同步到命令缓冲区

### 3.3 on_activate()

1. 将当前位置写入命令缓冲区（避免激活瞬间的位置跳变）
2. 用当前位置 + 配置的 PD 增益填充 `LowLevelCmd`
3. 执行一次 `sendRecv()` 开始控制

### 3.4 on_deactivate()

1. 将 Kp/Kd/tau 全部置零
2. 保持 q 为当前实际位置
3. 执行 `sendRecv()` 使机械臂进入零阻尼松弛状态

### 3.5 read() / write() 循环

每个控制周期按以下顺序执行：

```text
read()  → 从 lowState_ 复制 q/dq/tau_est 到 hw_states_*
          (供 JointStateBroadcaster 发布到 /joint_states)
          ↓
[controller_manager 运行 JointTrajectoryController]
          ↓
write() → 从 hw_commands_* 复制 q_d/dq_d 到 lowCmd_
          填入 Kp/Kd (来自配置)，tau=0
          调用 ioInter_->sendRecv(&lowCmd_, &lowState_)
          (sendRecv 内部完成一轮发送+接收，lowState_ 被更新)
```

---

## 4. 接口定义

### 4.1 命令接口 (Command Interfaces)

| 接口 | 类型 | 说明 |
| --- | --- | --- |
| `<joint>/position` | `double` | 期望关节位置 (rad)，由 `JointTrajectoryController` 写入 |
| `<joint>/velocity` | `double` | 期望关节速度 (rad/s)，由 `JointTrajectoryController` 写入 |

### 4.2 状态接口 (State Interfaces)

| 接口 | 类型 | 说明 |
| --- | --- | --- |
| `<joint>/position` | `double` | 实际关节位置 (rad) |
| `<joint>/velocity` | `double` | 实际关节速度 (rad/s) |
| `<joint>/effort` | `double` | 力矩估计 (N·m) |

### 4.3 关节名称

默认 6 个旋转关节：`J1`, `J2`, `J3`, `J4`, `J5`, `J6`。启用夹爪时追加第 7 关节。

---

## 5. 文件结构

```text
openmmarm_hw/
├── include/openmmarm_hw/
│   ├── openmmarm_hw.hpp       # 硬件插件主类
│   ├── io_interface.h         # IO 抽象基类
│   ├── io_mujoco.h            # MuJoCo 仿真驱动
│   ├── io_udp.h               # UDP 真机驱动
│   └── low_level_types.h      # LowLevelCmd / LowLevelState 数据结构
├── src/
│   ├── openmmarm_hw.cpp       # 插件生命周期实现
│   ├── io_mujoco.cpp          # MuJoCo 仿真实现 (URDF预处理/viewer)
│   └── io_udp.cpp             # UDP 通信实现 (CRC32/socket)
├── config/
│   ├── control_config.yaml    # 通信模式、PD增益、仿真/UDP 参数
│   └── openmmarm_hw_config.yaml  # controller_manager 和控制器插件配置
├── launch/
│   └── openmmarm_hw.launch.py
├── openmmarm_hw.xml           # pluginlib 插件描述
├── CMakeLists.txt
└── package.xml
```

---

## 6. 配置参考

### 6.1 control_config.yaml

```yaml
# 通信模式: "SIM" (MuJoCo 仿真) 或 "UDP" (真机)
communication: "SIM"

# 控制频率 (Hz)
# 同时决定 ros2_control 更新频率和 MuJoCo 仿真步长 (timestep = 1/update_rate)
# MuJoCo 使用隐式积分，数值稳定性不受频率限制
# 频率影响的是控制精度和力交互带宽:
#   position 模式: 250Hz 通常足够
#   impedance 模式: 建议 500-1000Hz 以获得良好的力交互体验
update_rate: 250

# 仿真配置 (communication=SIM 时使用)
sim:
  model_path: ""   # 留空则自动使用 openmmarm_description 中的 URDF
  viewer: true     # 是否启用 MuJoCo GLFW 可视化窗口

# UDP 通信配置 (communication=UDP 时使用)
udp:
  mcu_ip: "192.168.123.110"
  mcu_port: 8881
  local_port: 8871

# 控制模式: "position" 或 "impedance"
control_mode: "position"

# PD 增益 (按关节顺序 J1-J6)
pd_gains:
  position:
    kp: [100.0, 100.0, 100.0, 100.0, 50.0, 50.0]
    kd: [10.0, 10.0, 10.0, 10.0, 5.0, 5.0]
  impedance:
    kp: [25.0, 25.0, 25.0, 25.0, 15.0, 15.0]
    kd: [10.0, 10.0, 10.0, 10.0, 5.0, 5.0]
```

### 6.2 openmmarm_hw_config.yaml

```yaml
controller_manager:
  ros__parameters:
    update_rate: 250  # Hz — 控制频率，与 MCU 固件保持一致

openmmarm_arm_controller:
  ros__parameters:
    joints: [J1, J2, J3, J4, J5, J6]
    command_interfaces: [position, velocity]
    state_interfaces: [position, velocity]
    open_loop_control: true   # hw 自身执行 PD，不需要 controller 闭环
```

### 6.3 URDF 硬件参数 (openmmarm.ros2_control.xacro)

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `has_gripper` | `false` | 是否启用第 7 个夹爪关节 |

---

## 7. 使用方法

### 7.1 仿真模式（推荐入口）

```bash
colcon build
source install/setup.bash
ros2 launch openmmarm_bringup sim_arm.launch.py
```

启动后 MuJoCo viewer 自动弹出，可用鼠标交互：

- **左键拖动**：旋转视角
- **右键拖动**：缩放
- **中键拖动**：平移

### 7.2 真机模式

1. 修改 `config/control_config.yaml`：

```yaml
communication: "UDP"
udp:
  mcu_ip: "192.168.123.110"  # 根据实际 MCU 地址修改
  mcu_port: 8881
  local_port: 8871
```

2. 启动：

```bash
ros2 launch openmmarm_bringup real_arm.launch.py
```

### 7.3 单独启动

需要先启动 `robot_state_publisher` 提供 `/robot_description`：

```bash
ros2 launch openmmarm_description display.launch.py gui:=false
ros2 launch openmmarm_hw openmmarm_hw.launch.py
```

### 7.4 运行后检查

```bash
# 查看硬件接口状态
ros2 control list_hardware_interfaces

# 查看控制器状态
ros2 control list_controllers

# 检查关节状态发布
ros2 topic echo /joint_states
```

---

## 8. 依赖项

| 依赖 | 用途 |
| --- | --- |
| `rclcpp` / `rclcpp_lifecycle` | ROS 2 核心和生命周期管理 |
| `hardware_interface` / `pluginlib` | ros2_control 插件框架 |
| `rclcpp_action` / `control_msgs` | Gripper Action Server |
| `ament_index_cpp` | 包路径解析（用于找 URDF 和配置文件） |
| `yaml-cpp` | 解析 control_config.yaml |
| **MuJoCo** (>= 3.0) | 物理仿真引擎 |
| **GLFW** (可选) | MuJoCo 可视化窗口 |

---

## 9. 常见问题排查

| 现象 | 可能原因 | 解决方法 |
| --- | --- | --- |
| MuJoCo viewer 不弹出 | 未安装 GLFW 或编译时未检测到 | `sudo apt install libglfw3-dev` 后重新编译 |
| 仿真中机械臂抖动 | Kp 过大或 Kd 不足 | 降低 `pd_gains.position.kp` 或增大 `kd` |
| 仿真中机械臂松软无力 | 控制指令未到达 | 检查 `ros2 control list_controllers` 是否 active |
| UDP 模式连接超时 | MCU 地址/端口不对或网络不通 | `ping` 确认网络，检查 `control_config.yaml` 中 udp 配置 |
| 模型加载失败 | URDF/mesh 路径无效 | 确认 `openmmarm_description` 已编译安装，或在 `sim.model_path` 中指定绝对路径 |
| `ros2_control_node` 找不到 `robot_description` | 未启动 `robot_state_publisher` | 使用 `openmmarm_bringup` 启动或先手动启动 `robot_state_publisher` |
| 激活瞬间机械臂跳动 | 初始位置未同步 | 正常情况下 `on_configure` 已同步；若 MCU 未响应，检查通信状态 |
