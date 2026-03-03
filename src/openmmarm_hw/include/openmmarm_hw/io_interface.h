#pragma once

#include "openmmarm_hw/low_level_types.h"

namespace openmmarm_hw {

/**
 * @brief 硬件抽象接口基类
 *
 * 用于统一真机 (UDP) 和仿真 (MuJoCo) 的通信接口。
 */
class IOInterface {
public:
  IOInterface() = default;
  virtual ~IOInterface() = default;

  virtual bool init() = 0;
  virtual bool sendRecv(const LowLevelCmd *cmd, LowLevelState *state) = 0;
  virtual bool isConnected() = 0;

  bool initialized() const { return initialized_; }

protected:
  bool initialized_ = false;
};

} // namespace openmmarm_hw
