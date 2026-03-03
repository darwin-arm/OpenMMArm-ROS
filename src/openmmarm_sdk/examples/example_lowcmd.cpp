/**
 * @example example_lowcmd.cpp
 * @brief 底层控制（LowCmd）模式示例
 *
 * 演示如何使用 OpenMMArmSdk 进行电机级别的直接控制。
 * LowCmd 模式下用户需要自行设置 Kp/Kd 增益和位置/速度/力矩目标。
 *
 * 控制律: tau = Kp * (q_d - q) + Kd * (dq_d - dq) + tau_d
 *
 * 需要先启动仿真或真机环境：
 *   ros2 launch openmmarm_bringup sim_arm.launch.py
 *
 * 然后运行本程序：
 *   ./example_lowcmd [server_ip]
 */

#include "openmmarm_sdk/openmmarm_arm.h"

int main(int argc, char **argv) {
  // 连接 controller
  std::string controller_ip = argc > 1 ? argv[1] : "127.0.0.1";
  OPENMMARM_SDK::OpenMMArmSdk arm(controller_ip);
  arm.init();

  // 进入 LowCmd 模式前，机械臂应处于 PASSIVE 状态
  if (arm.armState.mode !=
      static_cast<uint8_t>(OPENMMARM_SDK::ArmMode::PASSIVE)) {
    std::cerr << "[ERROR] 请确保机械臂处于 PASSIVE 模式\n";
    return -1;
  }

  // 切换到 LowCmd 模式
  arm.armCmd.mode = static_cast<uint8_t>(OPENMMARM_SDK::ArmMode::LOW_CMD);

  // 同步当前位置
  arm.armCmd.q_d = arm.armState.q;
  arm.armCmd.dq_d.fill(0);
  arm.armCmd.tau_d.fill(0);

  // 设置 PD 增益
  arm.armCmd.Kp = {500, 600, 500, 400, 300, 200};
  arm.armCmd.Kd = {5, 5, 5, 5, 5, 5};
  arm.sendRecv();

  OPENMMARM_SDK::Timer timer(arm.dt);

  // 缓慢移动第一个关节
  double joint_speed = 0.2; // rad/s
  std::cout << "LowCmd 模式：移动第一个关节...\n";
  for (size_t i = 0; i < 300; ++i) {
    arm.armCmd.q_d[0] += joint_speed * arm.dt;
    arm.armCmd.setTau(Vec6::Zero());
    arm.sendRecv();
    timer.sleep();
  }
  std::cout << "LowCmd 控制完成\n";

  // 切回 PASSIVE 模式
  arm.armCmd.mode = static_cast<uint8_t>(OPENMMARM_SDK::ArmMode::PASSIVE);
  arm.sendRecv();

  return 0;
}
