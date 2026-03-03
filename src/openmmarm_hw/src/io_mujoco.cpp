#include "openmmarm_hw/io_mujoco.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <regex>
#include <set>
#include <sstream>

#ifdef OPENMMARM_HAS_GLFW
#include <GLFW/glfw3.h>
#endif

namespace fs = std::filesystem;

namespace openmmarm_hw {

static std::vector<fs::path> collectMeshPaths(const std::string &xml_content) {
  std::vector<fs::path> mesh_paths;

  std::regex mesh_re(R"RE(filename="package://([a-zA-Z0-9_]+)/([^"]+)")RE");
  auto begin =
      std::sregex_iterator(xml_content.begin(), xml_content.end(), mesh_re);
  auto end = std::sregex_iterator();

  std::map<std::string, fs::path> pkg_cache;

  for (auto it = begin; it != end; ++it) {
    std::string pkg_name = (*it)[1].str();
    std::string sub_path = (*it)[2].str();

    if (pkg_cache.find(pkg_name) == pkg_cache.end()) {
      try {
        std::string share_dir =
            ament_index_cpp::get_package_share_directory(pkg_name);
        pkg_cache[pkg_name] = fs::path(share_dir);
      } catch (const std::exception &e) {
        std::cerr << "[IOMujoco] 无法解析包 " << pkg_name << ": " << e.what()
                  << std::endl;
        continue;
      }
    }

    fs::path full_path = pkg_cache[pkg_name] / sub_path;
    mesh_paths.push_back(full_path);
  }

  std::sort(mesh_paths.begin(), mesh_paths.end());
  mesh_paths.erase(std::unique(mesh_paths.begin(), mesh_paths.end()),
                   mesh_paths.end());

  return mesh_paths;
}

static std::vector<fs::path>
linkMeshesToDirectory(const std::vector<fs::path> &mesh_paths,
                      const fs::path &target_dir) {
  std::vector<fs::path> created_links;

  for (const auto &mesh_path : mesh_paths) {
    if (!fs::exists(mesh_path)) {
      std::cerr << "[IOMujoco] 警告: mesh 文件不存在: " << mesh_path
                << std::endl;
      continue;
    }

    fs::path link_path = target_dir / mesh_path.filename();

    if (fs::exists(link_path) || fs::is_symlink(link_path)) {
      if (fs::is_symlink(link_path) &&
          fs::read_symlink(link_path) == mesh_path) {
        continue;
      }
      fs::remove(link_path);
    }

    try {
      fs::create_symlink(mesh_path, link_path);
      created_links.push_back(link_path);
      std::cout << "[IOMujoco]   链接: " << mesh_path.filename() << " -> "
                << mesh_path << std::endl;
    } catch (const std::exception &e) {
      std::cerr << "[IOMujoco] 无法创建符号链接 " << link_path << ": "
                << e.what() << std::endl;
    }
  }

  return created_links;
}

static std::string simplifyMeshFilenames(const std::string &xml_content) {
  std::regex mesh_re(
      R"RE(filename="package://[a-zA-Z0-9_]+/(?:[^"/]+/)*([^"]+)")RE");
  return std::regex_replace(xml_content, mesh_re, R"RE(filename="$1")RE");
}

IOMujoco::IOMujoco(const std::string &model_path, double timestep,
                   bool enable_viewer)
    : model_path_(model_path), timestep_(timestep),
      enable_viewer_(enable_viewer) {}

IOMujoco::~IOMujoco() {
  closeViewer();

  if (data_) {
    mj_deleteData(data_);
    data_ = nullptr;
  }
  if (model_) {
    mj_deleteModel(model_);
    model_ = nullptr;
  }

  for (const auto &link : created_symlinks_) {
    if (fs::is_symlink(link)) {
      fs::remove(link);
    }
  }

  if (!resolved_model_path_.empty()) {
    std::remove(resolved_model_path_.c_str());
  }
}

bool IOMujoco::init() {
  fs::path model_fs_path(model_path_);
  if (!fs::exists(model_fs_path)) {
    std::cerr << "[IOMujoco] 模型文件不存在: " << model_path_ << std::endl;
    return false;
  }

  fs::path model_dir = fs::canonical(model_fs_path.parent_path());

  std::ifstream ifs(model_path_);
  if (!ifs.is_open()) {
    std::cerr << "[IOMujoco] 无法打开模型文件: " << model_path_ << std::endl;
    return false;
  }
  std::stringstream ss;
  ss << ifs.rdbuf();
  ifs.close();
  std::string original_content = ss.str();

  std::vector<fs::path> mesh_paths = collectMeshPaths(original_content);
  std::cout << "[IOMujoco] 发现 " << mesh_paths.size() << " 个 mesh 文件"
            << std::endl;

  created_symlinks_ = linkMeshesToDirectory(mesh_paths, model_dir);
  std::cout << "[IOMujoco] 创建了 " << created_symlinks_.size() << " 个符号链接"
            << std::endl;

  std::string xml_content = simplifyMeshFilenames(original_content);

  resolved_model_path_ =
      (model_dir / (model_fs_path.stem().string() + "_mujoco.xml")).string();

  std::ofstream ofs(resolved_model_path_);
  if (!ofs.is_open()) {
    std::cerr << "[IOMujoco] 无法写入临时文件: " << resolved_model_path_
              << std::endl;
    return false;
  }
  ofs << xml_content;
  ofs.close();

  std::cout << "[IOMujoco] 已生成预处理模型: " << resolved_model_path_
            << std::endl;

  char error[1000] = "";
  mjModel *urdf_model =
      mj_loadXML(resolved_model_path_.c_str(), nullptr, error, sizeof(error));

  if (!urdf_model) {
    std::cerr << "[IOMujoco] URDF 模型加载失败: " << error << std::endl;
    return false;
  }

  if (urdf_model->nu == 0) {
    std::cout
        << "[IOMujoco] URDF 模型中 nu=0，正在通过 MJCF 中间格式注入 actuator..."
        << std::endl;

    std::string mjcf_path =
        (model_dir / (model_fs_path.stem().string() + "_mjcf.xml")).string();

    if (!mj_saveLastXML(mjcf_path.c_str(), urdf_model, error, sizeof(error))) {
      std::cerr << "[IOMujoco] 导出 MJCF 失败: " << error << std::endl;
      mj_deleteModel(urdf_model);
      return false;
    }

    std::ifstream mjcf_ifs(mjcf_path);
    if (!mjcf_ifs.is_open()) {
      std::cerr << "[IOMujoco] 无法打开导出的 MJCF: " << mjcf_path << std::endl;
      mj_deleteModel(urdf_model);
      return false;
    }
    std::stringstream mjcf_ss;
    mjcf_ss << mjcf_ifs.rdbuf();
    mjcf_ifs.close();
    std::string mjcf_content = mjcf_ss.str();

    std::vector<std::string> joint_names;
    for (int i = 0; i < urdf_model->njnt; ++i) {
      std::string jname(mj_id2name(urdf_model, mjOBJ_JOINT, i));
      if (urdf_model->jnt_type[i] != mjJNT_FREE) {
        joint_names.push_back(jname);
      }
    }

    if (!joint_names.empty()) {
      std::stringstream actuator_block;
      actuator_block << "\n  <actuator>\n";
      for (const auto &jname : joint_names) {
        // 使用 general actuator + biastype="affine"，让 implicitfast 积分器
        // 对 PD 增益做隐式积分，从根本上消除显式积分导致的数值振荡。
        // force = gain * ctrl + biasprm[0] + biasprm[1]*q + biasprm[2]*dq
        //       = 1*ctrl + 0 + (-Kp)*q + (-Kd)*dq
        // 运行时通过 model_->actuator_biasprm 动态更新 Kp/Kd，
        // ctrl[i] = Kp*q_d + Kd*dq_d + tau_d (常数项)
        actuator_block << "    <general name=\"actuator_" << jname
                       << "\" joint=\"" << jname << "\" gear=\"1\""
                       << " gaintype=\"fixed\" gainprm=\"1\""
                       << " biastype=\"affine\" biasprm=\"0 0 0\""
                       << " ctrllimited=\"true\" ctrlrange=\"-200 200\"/>\n";
      }
      actuator_block << "  </actuator>\n";

      size_t pos = mjcf_content.rfind("</mujoco>");
      if (pos != std::string::npos) {
        mjcf_content.insert(pos, actuator_block.str());
      }
    }

    std::ofstream mjcf_ofs(mjcf_path);
    mjcf_ofs << mjcf_content;
    mjcf_ofs.close();

    mj_deleteModel(urdf_model);
    urdf_model = nullptr;

    model_ = mj_loadXML(mjcf_path.c_str(), nullptr, error, sizeof(error));
    if (!model_) {
      std::cerr << "[IOMujoco] MJCF 模型加载失败: " << error << std::endl;
      return false;
    }

    std::cout << "[IOMujoco] 已通过 MJCF 中间格式成功注入 "
              << joint_names.size() << " 个 actuator" << std::endl;
  } else {
    model_ = urdf_model;
  }

  model_->opt.timestep = timestep_;

  // implicitfast 积分器对 actuator 的刚度/阻尼做隐式积分，
  // 即使在较大步长下也能保证 PD 控制的数值稳定性。
  model_->opt.integrator = mjINT_IMPLICITFAST;

  data_ = mj_makeData(model_);
  if (!data_) {
    std::cerr << "[IOMujoco] 仿真数据创建失败" << std::endl;
    mj_deleteModel(model_);
    model_ = nullptr;
    return false;
  }

  is_connected_ = true;
  initialized_ = true;

  if (enable_viewer_ && !initViewer()) {
    std::cerr
        << "[IOMujoco] 警告: MuJoCo 可视化窗口初始化失败，将继续无窗口运行"
        << std::endl;
  }

  std::cout << "[IOMujoco] 初始化完成 - 模型: " << model_path_ << std::endl;
  std::cout << "[IOMujoco]   关节数 (nq): " << model_->nq
            << ", 自由度 (nv): " << model_->nv
            << ", 执行器数 (nu): " << model_->nu << std::endl;
  std::cout << "[IOMujoco]   仿真步长: " << model_->opt.timestep << "s"
            << std::endl;

  return true;
}

bool IOMujoco::sendRecv(const LowLevelCmd *cmd, LowLevelState *state) {
  if (!model_ || !data_) {
    return false;
  }

  std::lock_guard<std::mutex> lock(sim_mutex_);

  int n = std::min(NUM_JOINTS, model_->nv);

  if (!sim_started_) {
    for (int i = 0; i < n; ++i) {
      if (std::abs(cmd->kp[i]) > 0.0f || std::abs(cmd->kd[i]) > 0.0f ||
          std::abs(cmd->tau[i]) > 0.0f) {
        sim_started_ = true;
        std::cout << "[IOMujoco] 收到有效控制指令，仿真开始推进" << std::endl;
        break;
      }
    }
  }

  if (!sim_started_) {
    for (int i = 0; i < n; ++i) {
      state->mode[i] = cmd->mode[i];
      state->q[i] = static_cast<float>(data_->qpos[i]);
      state->dq[i] = static_cast<float>(data_->qvel[i]);
      state->ddq[i] = 0.0f;
      state->tau_est[i] = 0.0f;
      state->temperature[i] = 25;
    }
    return true;
  }

  // 隐式 PD 控制：通过 general actuator 的 biastype="affine" 实现
  //
  // MuJoCo actuator 力公式:
  //   force = gainprm[0]*ctrl + biasprm[0] + biasprm[1]*q + biasprm[2]*dq
  //
  // 设置:
  //   gainprm[0] = 1 (在 MJCF 中固定)
  //   biasprm[0] = 0, biasprm[1] = -Kp, biasprm[2] = -Kd (每步动态更新)
  //   ctrl[i] = Kp*q_d + Kd*dq_d + tau_d
  //
  // 展开:
  //   force = (Kp*q_d + Kd*dq_d + τ_d) + 0 + (-Kp)*q + (-Kd)*dq
  //         = Kp*(q_d - q) + Kd*(dq_d - dq) + τ_d
  //
  // 与显式 PD 数学等价，但 implicitfast 积分器会对 biasprm 中的
  // 刚度/阻尼项做隐式积分，从根本上保证数值稳定性。
  if (model_->nu >= n) {
    for (int i = 0; i < n; ++i) {
      double kp = static_cast<double>(cmd->kp[i]);
      double kd = static_cast<double>(cmd->kd[i]);

      // 动态更新 actuator 的 affine bias 参数
      model_->actuator_biasprm[i * mjNBIAS + 0] = 0.0;   // 常数项
      model_->actuator_biasprm[i * mjNBIAS + 1] = -kp;    // 位置项 (隐式刚度)
      model_->actuator_biasprm[i * mjNBIAS + 2] = -kd;    // 速度项 (隐式阻尼)

      // ctrl = 常数驱动项 (不含 q/dq，由 MuJoCo 内部隐式处理)
      data_->ctrl[i] = kp * static_cast<double>(cmd->q[i])
                      + kd * static_cast<double>(cmd->dq[i])
                      + static_cast<double>(cmd->tau[i]);
    }
  } else {
    // fallback: 无 actuator 时直接施加力矩 (显式，仅作兜底)
    for (int i = 0; i < n; ++i) {
      double q_err = static_cast<double>(cmd->q[i]) - data_->qpos[i];
      double dq_err = static_cast<double>(cmd->dq[i]) - data_->qvel[i];
      data_->qfrc_applied[i] = cmd->kp[i] * q_err + cmd->kd[i] * dq_err +
                                static_cast<double>(cmd->tau[i]);
    }
  }
  mj_step(model_, data_);

  for (int i = 0; i < n; ++i) {
    state->mode[i] = cmd->mode[i];
    state->q[i] = static_cast<float>(data_->qpos[i]);
    state->dq[i] = static_cast<float>(data_->qvel[i]);
    state->ddq[i] = static_cast<float>(data_->qacc[i]);

    if (model_->nu >= n) {
      state->tau_est[i] = static_cast<float>(data_->actuator_force[i]);
    } else {
      state->tau_est[i] = static_cast<float>(data_->qfrc_applied[i]);
    }

    state->temperature[i] = 25;
  }

  return true;
}

bool IOMujoco::isConnected() { return is_connected_; }

bool IOMujoco::initViewer() {
  if (!enable_viewer_) {
    return true;
  }

#ifdef OPENMMARM_HAS_GLFW
  viewer_stop_requested_ = false;
  {
    std::lock_guard<std::mutex> lk(viewer_state_mutex_);
    viewer_init_done_ = false;
    viewer_init_ok_ = false;
  }

  viewer_thread_ = std::thread(&IOMujoco::viewerLoop, this);

  std::unique_lock<std::mutex> lk(viewer_state_mutex_);
  bool signaled = viewer_state_cv_.wait_for(
      lk, std::chrono::seconds(2), [this]() { return viewer_init_done_; });
  if (!signaled) {
    std::cerr << "[IOMujoco] 等待 MuJoCo viewer 线程启动超时" << std::endl;
    viewer_stop_requested_ = true;
    lk.unlock();
    if (viewer_thread_.joinable()) {
      viewer_thread_.join();
    }
    return false;
  }
  return viewer_init_ok_;
#else
  std::cerr << "[IOMujoco] 当前构建未启用 GLFW，无法打开 MuJoCo 可视化窗口"
            << std::endl;
  return false;
#endif
}

void IOMujoco::viewerLoop() {
#ifdef OPENMMARM_HAS_GLFW
  auto signalInit = [this](bool ok) {
    {
      std::lock_guard<std::mutex> lk(viewer_state_mutex_);
      viewer_init_ok_ = ok;
      viewer_init_done_ = true;
    }
    viewer_state_cv_.notify_one();
  };

  if (!glfwInit()) {
    std::cerr << "[IOMujoco] GLFW 初始化失败" << std::endl;
    signalInit(false);
    return;
  }

  window_ =
      glfwCreateWindow(1280, 800, "OpenMMArm MuJoCo Viewer", nullptr, nullptr);
  if (!window_) {
    std::cerr << "[IOMujoco] 无法创建 MuJoCo 可视化窗口" << std::endl;
    glfwTerminate();
    signalInit(false);
    return;
  }

  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1);

  glfwSetWindowUserPointer(window_, this);
  glfwSetMouseButtonCallback(window_, &IOMujoco::mouseButtonCallback);
  glfwSetCursorPosCallback(window_, &IOMujoco::cursorPosCallback);
  glfwSetScrollCallback(window_, &IOMujoco::scrollCallback);

  mjv_defaultCamera(&camera_);
  mjv_defaultOption(&option_);
  mjv_defaultScene(&scene_);
  mjr_defaultContext(&context_);

  {
    std::lock_guard<std::mutex> lk(sim_mutex_);
    mjv_makeScene(model_, &scene_, 2000);
    mjr_makeContext(model_, &context_, mjFONTSCALE_150);
    camera_.type = mjCAMERA_FREE;
    camera_.lookat[0] = model_->stat.center[0];
    camera_.lookat[1] = model_->stat.center[1];
    camera_.lookat[2] = model_->stat.center[2];
    camera_.distance = model_->stat.extent * 2.0;
  }

  viewer_initialized_ = true;
  std::cout << "[IOMujoco] MuJoCo 可视化窗口已开启" << std::endl;
  signalInit(true);

  while (!viewer_stop_requested_) {
    if (glfwWindowShouldClose(window_)) {
      break;
    }

    {
      std::lock_guard<std::mutex> lk(sim_mutex_);
      mjv_updateScene(model_, data_, &option_, nullptr, &camera_, mjCAT_ALL,
                      &scene_);
    }

    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(window_, &width, &height);
    mjrRect viewport = {0, 0, width, height};
    mjr_render(viewport, &scene_, &context_);

    glfwSwapBuffers(window_);
    glfwPollEvents();
  }

  viewer_initialized_ = false;
  mjr_freeContext(&context_);
  mjv_freeScene(&scene_);
  if (window_) {
    glfwDestroyWindow(window_);
    window_ = nullptr;
  }
  glfwTerminate();
#else
  (void)this;
#endif
}

void IOMujoco::mouseButtonCallback(GLFWwindow *w, int button, int action,
                                   int /*mods*/) {
  auto *self = static_cast<IOMujoco *>(glfwGetWindowUserPointer(w));
  if (!self)
    return;

  bool pressed = (action == GLFW_PRESS);
  if (button == GLFW_MOUSE_BUTTON_LEFT)
    self->mouse_button_left_ = pressed;
  if (button == GLFW_MOUSE_BUTTON_MIDDLE)
    self->mouse_button_middle_ = pressed;
  if (button == GLFW_MOUSE_BUTTON_RIGHT)
    self->mouse_button_right_ = pressed;

  glfwGetCursorPos(w, &self->mouse_last_x_, &self->mouse_last_y_);
}

void IOMujoco::cursorPosCallback(GLFWwindow *w, double xpos, double ypos) {
  auto *self = static_cast<IOMujoco *>(glfwGetWindowUserPointer(w));
  if (!self)
    return;

  if (!self->mouse_button_left_ && !self->mouse_button_middle_ &&
      !self->mouse_button_right_) {
    self->mouse_last_x_ = xpos;
    self->mouse_last_y_ = ypos;
    return;
  }

  double dx = xpos - self->mouse_last_x_;
  double dy = ypos - self->mouse_last_y_;
  self->mouse_last_x_ = xpos;
  self->mouse_last_y_ = ypos;

  int width, height;
  glfwGetWindowSize(w, &width, &height);

  mjtMouse action = mjMOUSE_NONE;
  if (self->mouse_button_left_)
    action = mjMOUSE_ROTATE_V;
  else if (self->mouse_button_middle_)
    action = mjMOUSE_MOVE_V;
  else if (self->mouse_button_right_)
    action = mjMOUSE_ZOOM;

  mjv_moveCamera(self->model_, action, dx / width, dy / height, &self->scene_,
                 &self->camera_);
}

void IOMujoco::scrollCallback(GLFWwindow *w, double /*xoffset*/,
                              double yoffset) {
  auto *self = static_cast<IOMujoco *>(glfwGetWindowUserPointer(w));
  if (!self)
    return;

  mjv_moveCamera(self->model_, mjMOUSE_ZOOM, 0.0, -0.05 * yoffset,
                 &self->scene_, &self->camera_);
}

void IOMujoco::closeViewer() {
#ifdef OPENMMARM_HAS_GLFW
  viewer_stop_requested_ = true;
  if (viewer_initialized_) {
    glfwPostEmptyEvent();
  }
  if (viewer_thread_.joinable()) {
    viewer_thread_.join();
  }
#endif
}

} // namespace openmmarm_hw
