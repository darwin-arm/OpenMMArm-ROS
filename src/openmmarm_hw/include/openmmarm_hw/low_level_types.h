#pragma once

#include <cstdint>

namespace openmmarm_hw {

/**
 * @brief 底层控制指令 (发给电机)
 */
struct LowLevelCmd {
  uint8_t mode[6]{};
  float q[6]{};
  float dq[6]{};
  float tau[6]{};
  float kp[6]{};
  float kd[6]{};
};

/**
 * @brief 底层状态反馈 (从电机获取)
 */
struct LowLevelState {
  uint8_t mode[6]{};
  float q[6]{};
  float dq[6]{};
  float ddq[6]{};
  float tau_est[6]{};
  int8_t temperature[6]{};
};

} // namespace openmmarm_hw
