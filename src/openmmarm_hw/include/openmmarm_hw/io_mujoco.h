#pragma once

#include "openmmarm_hw/io_interface.h"
#include <atomic>
#include <condition_variable>
#include <filesystem>
#include <mujoco/mujoco.h>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct GLFWwindow;

namespace openmmarm_hw {

/**
 * @brief MuJoCo 仿真接口实现
 *
 * 在进程内嵌入 MuJoCo 物理引擎，直接步进仿真。
 * 每次 sendRecv 调用会将控制指令写入 MuJoCo，
 * 执行一步仿真，然后读回关节状态。
 */
class IOMujoco : public IOInterface {
public:
  explicit IOMujoco(const std::string &model_path, double timestep = 0.004,
                    bool enable_viewer = false);
  ~IOMujoco() override;

  bool init() override;
  bool sendRecv(const LowLevelCmd *cmd, LowLevelState *state) override;
  bool isConnected() override;

private:
  bool initViewer();
  void viewerLoop();
  void closeViewer();

  static void mouseButtonCallback(GLFWwindow *w, int button, int action,
                                  int mods);
  static void cursorPosCallback(GLFWwindow *w, double xpos, double ypos);
  static void scrollCallback(GLFWwindow *w, double xoffset, double yoffset);

  std::string model_path_;
  std::string resolved_model_path_;
  double timestep_;
  bool enable_viewer_ = false;
  std::atomic<bool> viewer_initialized_{false};
  std::atomic<bool> viewer_stop_requested_{false};
  std::thread viewer_thread_;
  std::mutex sim_mutex_;
  std::mutex viewer_state_mutex_;
  std::condition_variable viewer_state_cv_;
  bool viewer_init_done_ = false;
  bool viewer_init_ok_ = false;

  mjModel *model_ = nullptr;
  mjData *data_ = nullptr;

  GLFWwindow *window_ = nullptr;
  mjvCamera camera_;
  mjvOption option_;
  mjvScene scene_;
  mjrContext context_;

  bool is_connected_ = false;
  bool sim_started_ = false;

  std::vector<std::filesystem::path> created_symlinks_;

  bool mouse_button_left_ = false;
  bool mouse_button_middle_ = false;
  bool mouse_button_right_ = false;
  double mouse_last_x_ = 0.0;
  double mouse_last_y_ = 0.0;

  static constexpr int NUM_JOINTS = 6;
};

} // namespace openmmarm_hw
