#include "openmmarm_hw/io_serial.h"
#include <chrono>
#include <iostream>
#include <thread>

namespace openmmarm_hw {
namespace {
inline int64_t nowSteadyMs() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
} // namespace

// CRC16 查找表 (与 ros2_ws 中 serial_port.cpp 完全一致)
const uint16_t IOSerial::CRC16_TABLE[256] = {
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011, 0x8033,
    0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022, 0x8063, 0x0066,
    0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072, 0x0050, 0x8055, 0x805F,
    0x005A, 0x804B, 0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
    0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB,
    0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE,
    0x00B4, 0x80B1, 0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087,
    0x0082, 0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1, 0x01E0,
    0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6,
    0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
    0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176, 0x017C, 0x8179,
    0x0168, 0x816D, 0x8167, 0x0162, 0x8123, 0x0126, 0x012C, 0x8129, 0x0138,
    0x813D, 0x8137, 0x0132, 0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E,
    0x0104, 0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317,
    0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371, 0x8353,
    0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
    0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3, 0x03F6, 0x03FC,
    0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2, 0x83A3, 0x03A6, 0x03AC, 0x83A9,
    0x03B8, 0x83BD, 0x83B7, 0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B,
    0x038E, 0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E,
    0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7,
    0x02A2, 0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
    0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252, 0x0270, 0x8275,
    0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261, 0x0220, 0x8225, 0x822F,
    0x022A, 0x823B, 0x023E, 0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219,
    0x0208, 0x820D, 0x8207, 0x0202};

// ==================== 构造 / 析构 ====================

IOSerial::IOSerial(const std::string &port_name, int baud_rate,
                   int link_timeout_ms)
    : port_name_(port_name),
      baud_rate_(baud_rate),
      link_timeout_ms_(link_timeout_ms) {
  received_motor_data_.resize(FRAME_DATA_LENGTH, 0);
  std::cout << "[IOSerial] 初始化串口接口: " << port_name << " @ " << baud_rate
            << " bps" << std::endl;
}

IOSerial::~IOSerial() {
  // 失能电机
  if (serial_port_.IsOpen()) {
    try {
      sendDisableMotors();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      sendDisableMotors();
      std::cout << "[IOSerial] 已发送失能指令" << std::endl;
    } catch (...) {
      std::cerr << "[IOSerial] 发送失能指令异常" << std::endl;
    }
  }

  // 停止接收线程，并通过关闭串口打断可能阻塞的 Read()
  running_ = false;
  if (serial_port_.IsOpen()) {
    try {
      serial_port_.Close();
    } catch (const std::exception &e) {
      std::cerr << "[IOSerial] 关闭串口异常: " << e.what() << std::endl;
    }
  }

  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
}

// ==================== IOInterface 实现 ====================

bool IOSerial::init() {
  is_connected_ = false;
  last_rx_time_ms_ = 0;
  last_reported_connected_ = false;

  try {
    serial_port_.Open(port_name_);

    // 设置波特率
    using namespace LibSerial;
    // 将整型波特率映射到 LibSerial 枚举
    BaudRate br;
    switch (baud_rate_) {
    case 9600:
      br = BaudRate::BAUD_9600;
      break;
    case 19200:
      br = BaudRate::BAUD_19200;
      break;
    case 38400:
      br = BaudRate::BAUD_38400;
      break;
    case 57600:
      br = BaudRate::BAUD_57600;
      break;
    case 115200:
      br = BaudRate::BAUD_115200;
      break;
    case 230400:
      br = BaudRate::BAUD_230400;
      break;
    case 460800:
      br = BaudRate::BAUD_460800;
      break;
    case 921600:
      br = BaudRate::BAUD_921600;
      break;
    default:
      std::cerr << "[IOSerial] 不支持的波特率: " << baud_rate_
                << "，使用默认 921600" << std::endl;
      br = BaudRate::BAUD_921600;
      break;
    }

    serial_port_.SetBaudRate(br);
    serial_port_.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
    serial_port_.SetParity(Parity::PARITY_NONE);
    serial_port_.SetStopBits(StopBits::STOP_BITS_1);
    serial_port_.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);

    std::cout << "[IOSerial] 串口已打开: " << port_name_ << std::endl;
  } catch (const LibSerial::OpenFailed &) {
    std::cerr << "[IOSerial] 无法打开串口: " << port_name_ << std::endl;
    return false;
  } catch (const std::exception &e) {
    std::cerr << "[IOSerial] 串口初始化异常: " << e.what() << std::endl;
    return false;
  }

  // 启动接收线程
  running_ = true;
  receive_thread_ = std::thread(&IOSerial::receiveLoop, this);

  // 等待接收线程就绪
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // 使能电机 (与 ros2_ws 一致: 发送两次，间隔 1 秒)
  sendEnableMotors();
  std::cerr << "[IOSerial] ENABLE MOTORS >>>>>>>>>>>>>>>>>>>>>>>>" << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(1));
  sendEnableMotors();

  // 等待首次有效数据
  for (int i = 0; i < 20; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (isConnected()) {
      initialized_ = true;
      std::cout << "[IOSerial] 成功连接到 MCU" << std::endl;
      return true;
    }
    std::cout << "[IOSerial] 等待 MCU 响应... (" << i + 1 << "/20)"
              << std::endl;
  }

  std::cerr << "[IOSerial] 警告：未收到 MCU 响应，但串口已打开" << std::endl;
  initialized_ = true; // 允许继续，接收线程会持续尝试
  return true;
}

bool IOSerial::sendRecv(const LowLevelCmd *cmd, LowLevelState *state) {
  if (!serial_port_.IsOpen()) {
    is_connected_ = false;
    return false;
  }

  // ====== 发送: LowLevelCmd → MIT 编码 → 帧封装 → 串口写入 ======
  std::vector<uint8_t> payload(FRAME_DATA_LENGTH, 0);

  for (int i = 0; i < 6; ++i) {
    bool is_4340 = (i < 3);
    uint8_t motor_data[8] = {0};
    encodeMitCmd(cmd->q[i], cmd->dq[i], cmd->kp[i], cmd->kd[i], cmd->tau[i],
                 motor_data, is_4340);
    // 每个电机占 8 字节，偏移 = i * 8
    for (int j = 0; j < 8; ++j) {
      payload[i * 8 + j] = motor_data[j];
    }
  }

  std::vector<uint8_t> frame;
  packFrame(payload, frame);
  sendData(frame);

  const bool connected = isConnected();
  if (connected != last_reported_connected_) {
    if (connected) {
      std::cout << "[IOSerial] 通信恢复" << std::endl;
    } else {
      std::cerr << "[IOSerial] 通信超时/掉线 (超过 " << link_timeout_ms_
                << " ms 未收到有效帧)" << std::endl;
    }
    last_reported_connected_ = connected;
  }

  if (!connected) {
    return false;
  }

  // ====== 接收: 从共享缓冲区读取最新的已解析电机状态 ======
  std::vector<uint8_t> motor_data;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    motor_data = received_motor_data_;
  }

  // MIT 解码: 每个电机 8 字节 → pos, vel, tau
  for (int i = 0; i < 6; ++i) {
    bool is_4340 = (i < 3);
    float pos = 0, vel = 0, tau = 0;
    decodeMitState(&motor_data[i * 8], pos, vel, tau, is_4340);
    state->q[i] = pos;
    state->dq[i] = vel;
    state->tau_est[i] = tau;
    state->ddq[i] = 0.0f; // 串口协议不传加速度
  }

  return true;
}

bool IOSerial::isConnected() {
  const int64_t last_rx_ms = last_rx_time_ms_.load(std::memory_order_relaxed);
  const int64_t now_ms = nowSteadyMs();
  const bool connected =
      (last_rx_ms > 0) && ((now_ms - last_rx_ms) <= link_timeout_ms_);
  is_connected_.store(connected, std::memory_order_relaxed);
  return connected;
}

// ==================== 接收线程 ====================

void IOSerial::receiveLoop() {
  while (running_) {
    auto raw = receiveRawFrame();
    if (raw.size() == FRAME_SIZE) {
      std::vector<uint8_t> parsed_data;
      if (parseFrame(raw, parsed_data)) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        received_motor_data_ = parsed_data;
        last_rx_time_ms_.store(nowSteadyMs(), std::memory_order_relaxed);
        is_connected_.store(true, std::memory_order_relaxed);
      }
    }
    std::this_thread::sleep_for(std::chrono::microseconds(500));
  }
}

std::vector<uint8_t> IOSerial::receiveRawFrame() {
  std::vector<uint8_t> buffer(FRAME_SIZE, 0);
  constexpr size_t kReadTimeoutMs = 10;

  try {
    size_t bytes_read = 0;
    while (bytes_read < FRAME_SIZE && running_) {
      LibSerial::DataBuffer byte_buf;
      serial_port_.Read(byte_buf, 1, kReadTimeoutMs);
      if (!byte_buf.empty()) {
        buffer[bytes_read++] = byte_buf[0];
      }
    }
  } catch (const LibSerial::ReadTimeout &) {
    // 超时，返回空
    return {};
  } catch (const std::exception &e) {
    if (running_) {
      std::cerr << "[IOSerial] 接收异常: " << e.what() << std::endl;
    }
    return {};
  }
  return buffer;
}

// ==================== 帧协议 ====================

void IOSerial::packFrame(const std::vector<uint8_t> &data,
                         std::vector<uint8_t> &frame) {
  if (data.size() != FRAME_DATA_LENGTH) {
    std::cerr << "[IOSerial] packFrame: 数据长度错误 (" << data.size()
              << " != " << FRAME_DATA_LENGTH << ")" << std::endl;
    return;
  }

  frame.resize(FRAME_SIZE);
  frame[0] = FRAME_HEAD;
  frame[1] = FRAME_DATA_LENGTH;

  for (size_t i = 0; i < FRAME_DATA_LENGTH; ++i) {
    frame[2 + i] = data[i];
  }

  uint16_t crc = calculateCRC16(data.data(), FRAME_DATA_LENGTH);
  frame[2 + FRAME_DATA_LENGTH] = crc & 0xFF;
  frame[2 + FRAME_DATA_LENGTH + 1] = (crc >> 8) & 0xFF;

  frame[FRAME_SIZE - 1] = FRAME_TAIL;
}

bool IOSerial::parseFrame(const std::vector<uint8_t> &frame,
                          std::vector<uint8_t> &data) {
  if (frame.size() != FRAME_SIZE || frame[0] != FRAME_HEAD ||
      frame[FRAME_SIZE - 1] != FRAME_TAIL) {
    return false;
  }

  uint16_t received_crc =
      frame[2 + FRAME_DATA_LENGTH] | (frame[2 + FRAME_DATA_LENGTH + 1] << 8);
  uint16_t calculated_crc = calculateCRC16(frame.data() + 2, FRAME_DATA_LENGTH);

  if (received_crc != calculated_crc) {
    return false;
  }

  data.assign(frame.begin() + 2, frame.begin() + 2 + FRAME_DATA_LENGTH);
  return true;
}

void IOSerial::sendData(const std::vector<uint8_t> &buffer) {
  if (buffer.size() != FRAME_SIZE) {
    std::cerr << "[IOSerial] sendData: 帧大小错误" << std::endl;
    return;
  }
  try {
    serial_port_.Write(buffer);
  } catch (const std::exception &e) {
    std::cerr << "[IOSerial] 发送异常: " << e.what() << std::endl;
  }
}

uint16_t IOSerial::calculateCRC16(const uint8_t *data, uint16_t length) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < length; ++i) {
    uint8_t index = (crc >> 8) ^ data[i];
    crc = (crc << 8) ^ CRC16_TABLE[index];
  }
  return crc;
}

// ==================== MIT 编解码 ====================

int IOSerial::floatToUint(float value, float min_val, float max_val, int bits) {
  float span = max_val - min_val;
  return static_cast<int>((value - min_val) * ((1 << bits) - 1) / span);
}

float IOSerial::uintToFloat(int value, float min_val, float max_val, int bits) {
  float span = max_val - min_val;
  return value * span / ((1 << bits) - 1) + min_val;
}

void IOSerial::encodeMitCmd(float pos, float vel, float kp, float kd, float tau,
                            uint8_t data[8], bool is_4340) {
  int pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

  if (is_4340) {
    pos_tmp = floatToUint(pos, P_MIN_4340, P_MAX_4340, 16);
    vel_tmp = floatToUint(vel, V_MIN_4340, V_MAX_4340, 12);
    kp_tmp = floatToUint(kp, KP_MIN_4340, KP_MAX_4340, 12);
    kd_tmp = floatToUint(kd, KD_MIN_4340, KD_MAX_4340, 12);
    tor_tmp = floatToUint(tau, T_MIN_4340, T_MAX_4340, 12);
  } else {
    pos_tmp = floatToUint(pos, P_MIN, P_MAX, 16);
    vel_tmp = floatToUint(vel, V_MIN, V_MAX, 12);
    kp_tmp = floatToUint(kp, KP_MIN, KP_MAX, 12);
    kd_tmp = floatToUint(kd, KD_MIN, KD_MAX, 12);
    tor_tmp = floatToUint(tau, T_MIN, T_MAX, 12);
  }

  data[0] = (pos_tmp >> 8);
  data[1] = pos_tmp;
  data[2] = (vel_tmp >> 4);
  data[3] = (((vel_tmp & 0xF) << 4) | (kp_tmp >> 8));
  data[4] = kp_tmp;
  data[5] = (kd_tmp >> 4);
  data[6] = (((kd_tmp & 0xF) << 4) | (tor_tmp >> 8));
  data[7] = tor_tmp;
}

void IOSerial::decodeMitState(const uint8_t data[8], float &pos, float &vel,
                              float &tau, bool is_4340) {
  int m1_int = (data[1] << 8) | data[2];
  int m2_int = (data[3] << 4) | (data[4] >> 4);
  int m3_int = ((data[4] & 0xF) << 8) | data[5];

  if (is_4340) {
    pos = uintToFloat(m1_int, P_MIN_4340, P_MAX_4340, 16);
    vel = uintToFloat(m2_int, V_MIN_4340, V_MAX_4340, 12);
    tau = uintToFloat(m3_int, T_MIN_4340, T_MAX_4340, 12);
  } else {
    pos = uintToFloat(m1_int, P_MIN, P_MAX, 16);
    vel = uintToFloat(m2_int, V_MIN, V_MAX, 12);
    tau = uintToFloat(m3_int, T_MIN, T_MAX, 12);
  }
}

// ==================== 使能/失能 ====================

void IOSerial::sendEnableMotors() {
  std::vector<uint8_t> payload(FRAME_DATA_LENGTH, 0);
  // 将使能数据填充到每个电机的 8 字节槽
  for (int motor = 0; motor < 6; ++motor) {
    for (int j = 0; j < 8; ++j) {
      payload[motor * 8 + j] = ENABLE_MOTOR_DATA[j];
    }
  }
  std::vector<uint8_t> frame;
  packFrame(payload, frame);
  sendData(frame);
}

void IOSerial::sendDisableMotors() {
  std::vector<uint8_t> payload(FRAME_DATA_LENGTH, 0);
  for (int motor = 0; motor < 6; ++motor) {
    for (int j = 0; j < 8; ++j) {
      payload[motor * 8 + j] = DISABLE_MOTOR_DATA[j];
    }
  }
  std::vector<uint8_t> frame;
  packFrame(payload, frame);
  sendData(frame);
}

} // namespace openmmarm_hw
