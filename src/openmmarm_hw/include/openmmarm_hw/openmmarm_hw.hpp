#ifndef OPENMMARM_HW_HPP
#define OPENMMARM_HW_HPP

#include <memory>
#include <string>
#include <vector>

#include "control_msgs/action/gripper_command.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "openmmarm_hw/arm_dynamics.h"
#include "openmmarm_hw/io_interface.h"
#include "openmmarm_hw/low_level_types.h"

namespace openmmarm_hw {

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using GripperAction = control_msgs::action::GripperCommand;
using GoalHandleGripper = rclcpp_action::ServerGoalHandle<GripperAction>;

/**
 * @brief OpenMMARM 硬件接口类
 *
 * 基于 ros2_control 的 SystemInterface，直接持有 IOInterface
 * 与仿真 (MuJoCo) 或真机 (UDP) 通信。
 */
class OpenMMArmHW : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(OpenMMArmHW)

  CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

  CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;

  hardware_interface::return_type
  write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  // 关节数量（默认 6，有夹爪则 7）
  size_t num_joints_{6};
  bool has_gripper_{false};

  // 状态变量
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;
  std::vector<double> hw_states_effort_;

  // 命令变量
  std::vector<double> hw_commands_position_;
  std::vector<double> hw_commands_velocity_;

  // 控制模式: "position" 或 "impedance"
  std::string control_mode_ = "position";

  // 关节 PD 增益
  std::array<double, 6> default_kp_ = {100.0, 100.0, 100.0, 100.0, 50.0, 50.0};
  std::array<double, 6> default_kd_ = {5.0, 5.0, 5.0, 5.0, 2.0, 2.0};

  // IO 接口（直接持有，取代 ArmSdkClient）
  std::unique_ptr<IOInterface> ioInter_;
  LowLevelCmd lowCmd_{};
  LowLevelState lowState_{};

  // 动力学前馈补偿（impedance 模式下使用）
  std::unique_ptr<ArmDynamics> dynamics_;

  // 通信配置
  std::string communication_mode_ = "SIM";
  std::string sim_model_path_;
  bool sim_viewer_ = true;
  std::string mcu_ip_ = "192.168.123.110";
  int mcu_port_ = 8881;
  int mcu_local_port_ = 8871;
  double update_rate_ = 250.0;

  // Gripper Action Server
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<GripperAction>::SharedPtr gripper_action_server_;

  // Gripper 状态
  double gripper_position_{0.0};
  double gripper_velocity_{0.0};
  double gripper_effort_{0.0};
  double gripper_command_{0.0};

  // Gripper Action 回调
  rclcpp_action::GoalResponse
  handle_gripper_goal(const rclcpp_action::GoalUUID &uuid,
                      std::shared_ptr<const GripperAction::Goal> goal);

  rclcpp_action::CancelResponse
  handle_gripper_cancel(const std::shared_ptr<GoalHandleGripper> goal_handle);

  void
  handle_gripper_accepted(const std::shared_ptr<GoalHandleGripper> goal_handle);
};

} // namespace openmmarm_hw

#endif // OPENMMARM_HW_HPP
