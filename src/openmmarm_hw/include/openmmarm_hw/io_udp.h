#pragma once

#include "openmmarm_hw/io_interface.h"
#include <arpa/inet.h>
#include <atomic>
#include <cstring>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

namespace openmmarm_hw {

/**
 * @brief UDP 通信接口实现
 *
 * 通过 UDP Socket 与真机 MCU 通信。
 */
class IOUDP : public IOInterface {
public:
  IOUDP(const std::string &mcu_ip, int mcu_port, int local_port = 0);
  ~IOUDP() override;

  bool init() override;
  bool sendRecv(const LowLevelCmd *cmd, LowLevelState *state) override;
  bool isConnected() override;

private:
  void sendCmd(const LowLevelCmd *cmd);
  bool recvState(LowLevelState *state);

  int socket_fd_ = -1;
  struct sockaddr_in mcu_addr_;
  struct sockaddr_in local_addr_;

  std::string mcu_ip_;
  int mcu_port_;
  int local_port_;

  std::atomic<bool> is_connected_{false};
  int recv_timeout_ms_ = 100;

#pragma pack(push, 1)
  struct UDPSendCmd {
    uint8_t head[2] = {0xFE, 0xEF};
    uint8_t mode[6];
    float q[6];
    float dq[6];
    float tau[6];
    float kp[6];
    float kd[6];
    uint32_t crc;
  };

  struct UDPRecvState {
    uint8_t head[2];
    uint8_t mode[6];
    float q[6];
    float dq[6];
    float ddq[6];
    float tau_est[6];
    int8_t temperature[6];
    uint8_t error_state[6];
    uint32_t crc;
  };
#pragma pack(pop)

  uint32_t calculateCRC32(const uint8_t *data, size_t len);
};

} // namespace openmmarm_hw
