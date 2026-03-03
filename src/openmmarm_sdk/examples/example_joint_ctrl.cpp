/**
 * @example example_joint_ctrl.cpp
 * @brief 关节速度控制和位置控制示例
 *
 * 演示如何使用 OpenMMArmSdk 进行关节速度控制和位置控制。
 * 需要先启动仿真或真机环境：
 *   ros2 launch openmmarm_bringup sim_arm.launch.py
 *
 * 然后运行本程序：
 *   ./example_joint_ctrl [server_ip]
 */

#include "openmmarm_sdk/openmmarm_arm.h"

int main(int argc, char **argv) {
  // 连接 controller
  std::string controller_ip = argc > 1 ? argv[1] : "127.0.0.1";
  OPENMMARM_SDK::OpenMMArmSdk arm(controller_ip);
  arm.init();

  OPENMMARM_SDK::Timer timer(arm.dt);

  // ===== 阶段 1: 关节速度控制 =====
  std::cout << "开始关节速度控制...\n";
  double joint_speed = 0.3; // rad/s
  for (size_t i = 0; i < 400; ++i) {
    arm.armCmd.mode = static_cast<uint8_t>(OPENMMARM_SDK::ArmMode::JOINT_CTRL);
    arm.armCmd.dq_d[0] = joint_speed; // 第一个关节正向转动
    arm.sendRecv();
    timer.sleep();
  }
  std::cout << "关节速度控制完成\n";

  // ===== 阶段 2: 关节位置控制 =====
  std::cout << "开始关节位置控制...\n";
  // 获取当前位置作为起点
  Vec6 q = arm.armState.getQ();
  for (size_t i = 0; i < 400; ++i) {
    arm.armCmd.mode = static_cast<uint8_t>(OPENMMARM_SDK::ArmMode::JOINT_CTRL);
    q(0) -= joint_speed * arm.dt; // 第一个关节反向移动
    arm.armCmd.setQ(q);
    arm.armCmd.dq_d[0] = -joint_speed;
    arm.sendRecv();
    timer.sleep();
  }
  std::cout << "关节位置控制完成\n";

  // 切换到 PASSIVE 模式
  arm.armCmd.mode = static_cast<uint8_t>(OPENMMARM_SDK::ArmMode::PASSIVE);
  arm.sendRecv();

  return 0;
}
