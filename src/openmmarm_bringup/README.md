# openmmarm_bringup

OpenMMArm 的统一启动包，提供仿真与真机两种启动入口。

## 架构简述

`openmmarm_bringup` 不实现控制算法，它负责把以下模块按顺序拉起：

- `openmmarm_description`：机器人模型与 `robot_state_publisher`
- `openmmarm_hw`：`ros2_control` 硬件插件（内嵌 MuJoCo 仿真 / UDP 真机通信）
- `openmmarm_moveit_config`：`move_group` 规划服务
- `rviz2`：可视化（可选）

## 使用方法

### 1. 前置条件

```bash
colcon build --symlink-install
source install/setup.bash
```

### 2. 仿真模式

```bash
ros2 launch openmmarm_bringup sim_arm.launch.py
```

常用参数：

```bash
ros2 launch openmmarm_bringup sim_arm.launch.py \
  use_rviz:=true \
  use_sim_time:=false \
  use_moveit:=true
```

### 3. 真机模式

将 `src/openmmarm_hw/config/control_config.yaml` 中 `communication` 改为 `"UDP"` 并配置 MCU 地址，然后：

```bash
ros2 launch openmmarm_bringup real_arm.launch.py use_rviz:=true use_moveit:=true
```

### 4. 运行后检查

```bash
ros2 control list_controllers
ros2 topic echo /joint_states
```

如果需要规划，再确认：

```bash
ros2 node list | rg move_group
```

## 参数说明

### sim_arm.launch.py

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `use_rviz` | `true` | 是否启动 RViz2 |
| `use_sim_time` | `false` | 是否使用仿真时间 |
| `use_moveit` | `true` | 是否启动 `move_group` |

### real_arm.launch.py

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `use_rviz` | `true` | 是否启动 RViz2 |
| `use_moveit` | `true` | 是否启动 `move_group` |
