# OpenMMArm-ROS

OpenMMArm 的 ROS 2 工作空间，包含 `ros2_control` 硬件接口、MoveIt 配置、SDK 和一键启动包。

## 架构简述

项目分为三层：

1. 应用层：MoveIt / 自定义节点 / SDK 用户程序
2. 接入层：`openmmarm_hw`（`ros2_control`，内嵌 IO 驱动）和 `openmmarm_sdk`（UDP 客户端）
3. 执行层：MuJoCo 仿真（进程内）或真机 MCU（UDP）

详细架构请看：`OPENMMARM_ARCHITECTURE.md`

## 快速使用

### 1. 编译

```bash
colcon build
source install/setup.bash
```

### 2. 仿真一键启动

```bash
ros2 launch openmmarm_bringup sim_arm.launch.py
```

常用参数：

```bash
ros2 launch openmmarm_bringup sim_arm.launch.py use_rviz:=true use_moveit:=true
```

### 3. 真机启动

将 `control_config.yaml` 中 `communication` 改为 `"UDP"` 并配置 MCU 地址，然后：

```bash
ros2 launch openmmarm_bringup real_arm.launch.py use_rviz:=true
```

### 4. 运行 SDK 示例

```bash
./install/openmmarm_sdk/lib/openmmarm_sdk/example_joint_ctrl
./install/openmmarm_sdk/lib/openmmarm_sdk/example_lowcmd
```

## 各包入口

- `src/openmmarm_bringup/README.md`：统一启动（仿真/真机）
- `src/openmmarm_hw/README.md`：`ros2_control` 硬件接口使用
- `src/openmmarm_moveit_config/README.md`：MoveIt 配置与启动
- `src/openmmarm_sdk/README.md`：SDK API 与示例
