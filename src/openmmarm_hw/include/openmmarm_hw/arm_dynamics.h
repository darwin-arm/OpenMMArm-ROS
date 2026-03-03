#pragma once

#include <mujoco/mujoco.h>
#include <string>

namespace openmmarm_hw {

static constexpr int ARM_DOF = 6;

/**
 * @brief 基于 MuJoCo 逆动力学的前馈力矩计算
 *
 * 加载 URDF（剥离 visual/collision 避免 mesh 依赖），
 * 持有独立的 mjModel/mjData（scratch data），
 * 用 mj_rne() 计算 M(q)*ddq_d + C(q,dq)*dq + g(q)。
 */
class ArmDynamics {
public:
  ArmDynamics() = default;
  ~ArmDynamics();

  ArmDynamics(const ArmDynamics &) = delete;
  ArmDynamics &operator=(const ArmDynamics &) = delete;

  /**
   * @brief 初始化：加载 URDF 并构建 MuJoCo 模型
   * @param urdf_path URDF 文件路径
   * @return true 成功
   */
  bool init(const std::string &urdf_path);

  /**
   * @brief 计算逆动力学前馈力矩
   * @param q      实际关节位置 [ARM_DOF]
   * @param dq     实际关节速度 [ARM_DOF]
   * @param ddq_d  期望关节加速度 [ARM_DOF]（初期设为 0）
   * @param tau_ff  输出前馈力矩 [ARM_DOF]
   */
  void computeFeedforward(const double q[ARM_DOF], const double dq[ARM_DOF],
                          const double ddq_d[ARM_DOF],
                          double tau_ff[ARM_DOF]);

private:
  /**
   * @brief 从 URDF 内容中剥离 <visual> 和 <collision> 标签块
   */
  static std::string stripVisualCollision(const std::string &urdf_content);

  mjModel *model_ = nullptr;
  mjData *data_ = nullptr;
  std::string tmp_urdf_path_;
};

} // namespace openmmarm_hw
