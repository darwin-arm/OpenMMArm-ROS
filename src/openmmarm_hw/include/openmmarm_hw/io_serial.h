#pragma once

#include "openmmarm_hw/io_interface.h"
#include <atomic>
#include <cstdint>
#include <libserial/SerialPort.h>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace openmmarm_hw {

/**
 * @brief 串口通信接口实现
 *
 * 通过 RS-485/USB 串口与 MCU 通信，复用 MIT 协议帧格式。
 * 帧结构: [HEAD 0xAA][LEN 59][DATA 59B][CRC16 2B][TAIL 0x55]  共 64 字节
 */
class IOSerial : public IOInterface {
public:
  IOSerial(const std::string &port_name, int baud_rate = 921600,
           int link_timeout_ms = 200);
  ~IOSerial() override;

  bool init() override;
  bool sendRecv(const LowLevelCmd *cmd, LowLevelState *state) override;
  bool isConnected() override;

private:
  // ------ 帧协议常量 ------
  static constexpr uint8_t FRAME_HEAD = 0xAA;
  static constexpr uint8_t FRAME_TAIL = 0x55;
  static constexpr uint16_t FRAME_DATA_LENGTH = 59;
  static constexpr uint16_t FRAME_SIZE = 1 + 1 + FRAME_DATA_LENGTH + 2 + 1; // 64

  // ------ MIT 协议物理量范围 (标准电机, J4-J6) ------
  static constexpr float P_MIN = -12.5f;
  static constexpr float P_MAX = 12.5f;
  static constexpr float V_MIN = -30.0f;
  static constexpr float V_MAX = 30.0f;
  static constexpr float KP_MIN = 0.0f;
  static constexpr float KP_MAX = 500.0f;
  static constexpr float KD_MIN = 0.0f;
  static constexpr float KD_MAX = 5.0f;
  static constexpr float T_MIN = -10.0f;
  static constexpr float T_MAX = 10.0f;

  // ------ MIT 协议物理量范围 (4340 电机, J1-J3) ------
  static constexpr float P_MIN_4340 = -12.5f;
  static constexpr float P_MAX_4340 = 12.5f;
  static constexpr float V_MIN_4340 = -10.0f;
  static constexpr float V_MAX_4340 = 10.0f;
  static constexpr float KP_MIN_4340 = 0.0f;
  static constexpr float KP_MAX_4340 = 500.0f;
  static constexpr float KD_MIN_4340 = 0.0f;
  static constexpr float KD_MAX_4340 = 5.0f;
  static constexpr float T_MIN_4340 = -28.0f;
  static constexpr float T_MAX_4340 = 28.0f;

  // ------ 串口配置 ------
  std::string port_name_;
  int baud_rate_;
  int link_timeout_ms_;
  LibSerial::SerialPort serial_port_;

  // ------ 异步接收线程 ------
  std::atomic<bool> running_{false};
  std::thread receive_thread_;
  std::mutex data_mutex_;
  std::vector<uint8_t> received_motor_data_; // 解析后的 59 字节有效载荷
  std::atomic<bool> is_connected_{false};
  std::atomic<int64_t> last_rx_time_ms_{0};
  bool last_reported_connected_{false};

  // ------ 使能/失能指令 ------
  static constexpr uint8_t ENABLE_MOTOR_DATA[8] = {0xff, 0xff, 0xff, 0xff,
                                                    0xff, 0xff, 0xff, 0xfc};
  static constexpr uint8_t DISABLE_MOTOR_DATA[8] = {0xff, 0xff, 0xff, 0xff,
                                                     0xff, 0xff, 0xff, 0xfd};

  // ------ 内部方法 ------
  void receiveLoop();
  std::vector<uint8_t> receiveRawFrame();

  void packFrame(const std::vector<uint8_t> &data, std::vector<uint8_t> &frame);
  bool parseFrame(const std::vector<uint8_t> &frame, std::vector<uint8_t> &data);
  void sendData(const std::vector<uint8_t> &buffer);

  uint16_t calculateCRC16(const uint8_t *data, uint16_t length);

  // MIT 编解码
  static int floatToUint(float value, float min_val, float max_val, int bits);
  static float uintToFloat(int value, float min_val, float max_val, int bits);

  void encodeMitCmd(float pos, float vel, float kp, float kd, float tau,
                    uint8_t data[8], bool is_4340);
  void decodeMitState(const uint8_t data[8], float &pos, float &vel,
                      float &tau, bool is_4340);

  void sendEnableMotors();
  void sendDisableMotors();

  // CRC16 查找表
  static const uint16_t CRC16_TABLE[256];
};

} // namespace openmmarm_hw
