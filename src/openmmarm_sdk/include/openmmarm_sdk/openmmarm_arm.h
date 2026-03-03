#pragma once

/**
 * @file openmmarm_arm.h
 * @brief OpenMMARM 机械臂 SDK 控制接口
 *
 * 提供与机械臂服务端通信的高层接口。
 * 使用方法：
 *   1. 创建 OpenMMArmSdk 对象
 *   2. 调用 init() 等待连接
 *   3. 在循环中设置 armCmd 的字段，调用 sendRecv()
 *
 * @code
 *   OPENMMARM_SDK::OpenMMArmSdk arm("127.0.0.1");
 *   arm.init();
 *
 *   Timer timer(arm.dt);
 *   while (running) {
 *     arm.armCmd.mode = (uint8_t)ArmMode::JOINT_CTRL;
 *     arm.armCmd.q_d[0] += 0.001;
 *     arm.sendRecv();
 *     timer.sleep();
 *   }
 * @endcode
 */

#include <iostream>
#include <thread>

#include "openmmarm_sdk/openmmarm_arm_common.h"
#include "openmmarm_sdk/timer.h"
#include "openmmarm_sdk/udp_port.h"

namespace OPENMMARM_SDK {

class OpenMMArmSdk {
public:
  /**
   * @brief 构造 SDK 实例
   *
   * @param controllerIP  机械臂服务端的 IP 地址
   * @param ownPort       本机绑定的 UDP 端口（0 = 系统分配）
   * @param toPort        服务端的 SDK 监听端口（默认 8871）
   */
  OpenMMArmSdk(std::string controllerIP, unsigned int ownPort = 0,
               unsigned int toPort = 8871) {
    // 设置版本号
    armCmd.version = {1, 0, 0};
    udp_ = std::make_shared<UdpPort>("openmmarm", ownPort, controllerIP, toPort,
                                     20);
    udp_->setCRC32(true);
  }

  /**
   * @brief 初始化连接
   *
   * 等待直到 controller 回传有效的 ArmState（mode != INVALID）。
   * 超时（5 秒）则退出程序。
   */
  void init() {
    std::cout << "等待连接机械臂服务端...\n";

    Timer timer(5.0);
    while (armState.mode == 0) {
      armCmd.mode = 0; // INVALID - 握手阶段
      udp_->send(reinterpret_cast<uint8_t *>(&armCmd), ARM_CMD_LENGTH);
      if (IOPortStatus::OK == udp_->recv(reinterpret_cast<uint8_t *>(&armState),
                                         ARM_STATE_LENGTH)) {
        // 检查版本兼容性
        if (armState.version[0] != armCmd.version[0]) {
          std::cerr << "[ERROR] 版本不兼容。Controller 版本: "
                    << static_cast<int>(armState.version[0]) << "."
                    << static_cast<int>(armState.version[1]) << "."
                    << static_cast<int>(armState.version[2]) << "\n";
          exit(-1);
        }
      }
      if (timer.waitTime() < 0) {
        std::cerr << "[ERROR] 连接机械臂服务端超时失败\n";
        exit(-1);
      }
    }

    // 同步当前位置作为初始目标
    armCmd.q_d = armState.q;
    std::cout << "已连接到机械臂服务端\n";
  }

  /**
   * @brief 发送指令并接收状态
   *
   * 应在每个控制周期调用一次。
   */
  void sendRecv() {
    udp_->send(reinterpret_cast<uint8_t *>(&armCmd), ARM_CMD_LENGTH);
    udp_->recv(reinterpret_cast<uint8_t *>(&armState), ARM_STATE_LENGTH);
  }

  /**
   * @brief 清除错误状态
   *
   * 发送 CLEAR_ERROR 指令后切换回 PASSIVE 模式。
   */
  void clearErrors() {
    armCmd.mode = static_cast<uint8_t>(ArmMode::CLEAR_ERROR);
    sendRecv();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    armCmd.mode = static_cast<uint8_t>(ArmMode::PASSIVE);
    sendRecv();
  }

  /**
   * @brief 打印错误状态日志
   */
  void printLog() {
    if (armState.errors[static_cast<int>(ArmError::JOINT_POSITION_LIMIT)]) {
      std::cout << "[WARNING] 关节位置超限\n";
    }
    if (armState.errors[static_cast<int>(ArmError::JOINT_VELOCITY_LIMIT)]) {
      std::cout << "[WARNING] 关节速度超限\n";
    }
    if (armState.errors[static_cast<int>(ArmError::COLLISION_DETECTED)]) {
      std::cout << "[WARNING] 碰撞检测触发\n";
    }
    if (armState.errors[static_cast<int>(ArmError::COMMUNICATION_LOST)]) {
      std::cout << "[ERROR] 通信丢失\n";
    }
  }

  /// 指令包（用户直接设置字段）
  ArmCmd armCmd{};

  /// 状态包（由 sendRecv 自动更新）
  ArmState armState{};

  /// 控制周期（秒），250Hz
  static constexpr double dt = 0.004;

private:
  UdpPortPtr udp_;
};

} // namespace OPENMMARM_SDK
