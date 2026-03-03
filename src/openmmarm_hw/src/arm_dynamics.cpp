#include "openmmarm_hw/arm_dynamics.h"

#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>

namespace openmmarm_hw {

ArmDynamics::~ArmDynamics() {
  if (data_) {
    mj_deleteData(data_);
    data_ = nullptr;
  }
  if (model_) {
    mj_deleteModel(model_);
    model_ = nullptr;
  }
  if (!tmp_urdf_path_.empty()) {
    std::remove(tmp_urdf_path_.c_str());
  }
}

std::string ArmDynamics::stripVisualCollision(const std::string &urdf_content) {
  // 剥离 <visual>...</visual> 和 <collision>...</collision> 块
  // 动力学计算只需惯性参数和运动学链，不需要 mesh
  static const std::regex visual_re(
      R"(<visual\b[^>]*>[\s\S]*?</visual>)", std::regex::optimize);
  static const std::regex collision_re(
      R"(<collision\b[^>]*>[\s\S]*?</collision>)", std::regex::optimize);

  std::string result = std::regex_replace(urdf_content, visual_re, "");
  result = std::regex_replace(result, collision_re, "");
  return result;
}

bool ArmDynamics::init(const std::string &urdf_path) {
  namespace fs = std::filesystem;

  if (!fs::exists(urdf_path)) {
    std::cerr << "[ArmDynamics] URDF 文件不存在: " << urdf_path << std::endl;
    return false;
  }

  // 读取 URDF 内容
  std::ifstream ifs(urdf_path);
  if (!ifs.is_open()) {
    std::cerr << "[ArmDynamics] 无法打开 URDF: " << urdf_path << std::endl;
    return false;
  }
  std::stringstream ss;
  ss << ifs.rdbuf();
  ifs.close();

  // 剥离 visual/collision，避免 mesh 依赖
  std::string stripped = stripVisualCollision(ss.str());

  // 写入临时文件
  fs::path urdf_dir = fs::canonical(fs::path(urdf_path).parent_path());
  tmp_urdf_path_ = (urdf_dir / "openmmarm_dynamics_stripped.urdf").string();

  std::ofstream ofs(tmp_urdf_path_);
  if (!ofs.is_open()) {
    std::cerr << "[ArmDynamics] 无法写入临时文件: " << tmp_urdf_path_
              << std::endl;
    return false;
  }
  ofs << stripped;
  ofs.close();

  // 加载 MuJoCo 模型
  char error[1000] = "";
  model_ = mj_loadXML(tmp_urdf_path_.c_str(), nullptr, error, sizeof(error));
  if (!model_) {
    std::cerr << "[ArmDynamics] MuJoCo 模型加载失败: " << error << std::endl;
    return false;
  }

  data_ = mj_makeData(model_);
  if (!data_) {
    std::cerr << "[ArmDynamics] mjData 创建失败" << std::endl;
    mj_deleteModel(model_);
    model_ = nullptr;
    return false;
  }

  std::cout << "[ArmDynamics] 初始化完成 - nq=" << model_->nq
            << ", nv=" << model_->nv << std::endl;

  if (model_->nv < ARM_DOF) {
    std::cerr << "[ArmDynamics] 警告: 模型自由度 nv=" << model_->nv
              << " < ARM_DOF=" << ARM_DOF << std::endl;
  }

  return true;
}

void ArmDynamics::computeFeedforward(const double q[ARM_DOF],
                                     const double dq[ARM_DOF],
                                     const double ddq_d[ARM_DOF],
                                     double tau_ff[ARM_DOF]) {
  if (!model_ || !data_) {
    std::memset(tau_ff, 0, ARM_DOF * sizeof(double));
    return;
  }

  int nv = std::min(model_->nv, ARM_DOF);

  // 1. 设置状态
  for (int i = 0; i < nv; ++i) {
    data_->qpos[i] = q[i];
    data_->qvel[i] = dq[i];
    data_->qacc[i] = ddq_d[i];
  }

  // 2. 正运动学（mj_rne 的前置条件）
  mj_kinematics(model_, data_);
  mj_comPos(model_, data_);
  mj_comVel(model_, data_);

  // 3. 递归牛顿-欧拉法: result = M(q)*ddq_d + C(q,dq)*dq + g(q)
  mjtNum result[ARM_DOF] = {};
  mj_rne(model_, data_, /*flg_acc=*/1, result);

  // 4. 输出
  for (int i = 0; i < ARM_DOF; ++i) {
    tau_ff[i] = (i < nv) ? static_cast<double>(result[i]) : 0.0;
  }
}

} // namespace openmmarm_hw
