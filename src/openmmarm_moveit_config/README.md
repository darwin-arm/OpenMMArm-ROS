# openmmarm_moveit_config

OpenMMArm 的 MoveIt 2 配置包，提供规划组、运动学、关节限制、控制器映射和启动文件。

## 架构简述

本包位于“规划层配置”，不直接控制硬件：

- `demo.launch.py`：Fake Controller 演示环境
- `move_group.launch.py`：只启动 `move_group`，供实际系统集成
- `config/*.yaml`：规划和控制行为参数

## 使用方法

### 1. Demo 模式（最快验证）

```bash
colcon build --packages-select openmmarm_moveit_config --symlink-install
source install/setup.bash
ros2 launch openmmarm_moveit_config demo.launch.py
```

常用参数：

```bash
ros2 launch openmmarm_moveit_config demo.launch.py use_rviz:=true use_sim_time:=false
```

### 2. 接入真实控制链路

推荐通过 bringup 启动（会按顺序拉起控制链路）：

```bash
ros2 launch openmmarm_bringup sim_arm.launch.py
# 或
ros2 launch openmmarm_bringup real_arm.launch.py
```

### 3. 仅启动 move_group

适合你已自行启动 `openmmarm_hw`、`robot_state_publisher` 的场景：

```bash
ros2 launch openmmarm_moveit_config move_group.launch.py use_sim_time:=false
```

## 核心配置文件

- `config/openmmarm.srdf`：规划组与语义定义
- `config/kinematics.yaml`：运动学求解配置
- `config/joint_limits.yaml`：关节速度/加速度限制
- `config/ompl_planning.yaml`：OMPL 规划器配置
- `config/moveit_controllers.yaml`：MoveIt 控制器映射

## 常见自定义

- 修改关节速度/加速度限制：`config/joint_limits.yaml`
- 调整规划算法参数：`config/ompl_planning.yaml`
- 调整运动学超时和精度：`config/kinematics.yaml`
