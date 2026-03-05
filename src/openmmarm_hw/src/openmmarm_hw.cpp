#include "openmmarm_hw/openmmarm_hw.hpp"
#include "openmmarm_hw/arm_dynamics.h"
#include "openmmarm_hw/io_mujoco.h"
#include "openmmarm_hw/io_serial.h"
#include "openmmarm_hw/io_udp.h"

#include <limits>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace openmmarm_hw {

CallbackReturn
OpenMMArmHW::on_init(const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // 读取 URDF 参数
  if (info_.hardware_parameters.find("has_gripper") !=
      info_.hardware_parameters.end()) {
    has_gripper_ = info_.hardware_parameters["has_gripper"] == "true";
  }

  num_joints_ = has_gripper_ ? 7 : 6;

  // 从 control_config.yaml 读取控制模式、PD 增益和通信配置
  try {
    std::string pkg_path =
        ament_index_cpp::get_package_share_directory("openmmarm_hw");
    std::string config_path = pkg_path + "/config/control_config.yaml";
    YAML::Node config = YAML::LoadFile(config_path);

    if (config["control_mode"]) {
      control_mode_ = config["control_mode"].as<std::string>();
    }

    if (config["pd_gains"] && config["pd_gains"][control_mode_]) {
      auto mode_gains = config["pd_gains"][control_mode_];
      if (mode_gains["kp"]) {
        auto kp_vec = mode_gains["kp"].as<std::vector<double>>();
        for (size_t i = 0; i < 6 && i < kp_vec.size(); ++i) {
          default_kp_[i] = kp_vec[i];
        }
      }
      if (mode_gains["kd"]) {
        auto kd_vec = mode_gains["kd"].as<std::vector<double>>();
        for (size_t i = 0; i < 6 && i < kd_vec.size(); ++i) {
          default_kd_[i] = kd_vec[i];
        }
      }
    } else {
      if (control_mode_ == "impedance") {
        default_kp_ = {25.0, 25.0, 25.0, 25.0, 15.0, 15.0};
        default_kd_ = {10.0, 10.0, 10.0, 10.0, 5.0, 5.0};
      } else {
        default_kp_ = {100.0, 100.0, 100.0, 100.0, 50.0, 50.0};
        default_kd_ = {5.0, 5.0, 5.0, 5.0, 2.0, 2.0};
      }
    }

    // 通信配置
    if (config["communication"]) {
      communication_mode_ = config["communication"].as<std::string>();
    }

    if (config["update_rate"]) {
      update_rate_ = config["update_rate"].as<double>();
    }

    if (config["sim"]) {
      if (config["sim"]["model_path"]) {
        sim_model_path_ = config["sim"]["model_path"].as<std::string>();
      }
      if (config["sim"]["viewer"]) {
        sim_viewer_ = config["sim"]["viewer"].as<bool>();
      }
    }

    if (config["udp"]) {
      if (config["udp"]["mcu_ip"]) {
        mcu_ip_ = config["udp"]["mcu_ip"].as<std::string>();
      }
      if (config["udp"]["mcu_port"]) {
        mcu_port_ = config["udp"]["mcu_port"].as<int>();
      }
      if (config["udp"]["local_port"]) {
        mcu_local_port_ = config["udp"]["local_port"].as<int>();
      }
    }

    if (config["serial"]) {
      if (config["serial"]["port"]) {
        serial_port_ = config["serial"]["port"].as<std::string>();
      }
      if (config["serial"]["baud_rate"]) {
        serial_baud_rate_ = config["serial"]["baud_rate"].as<int>();
      }
      if (config["serial"]["link_timeout_ms"]) {
        serial_link_timeout_ms_ = config["serial"]["link_timeout_ms"].as<int>();
      }
    }

    RCLCPP_INFO(rclcpp::get_logger("OpenMMArmHW"),
                "从 YAML 加载配置: communication=%s, mode=%s, "
                "Kp=[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f], "
                "Kd=[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]",
                communication_mode_.c_str(), control_mode_.c_str(),
                default_kp_[0], default_kp_[1], default_kp_[2], default_kp_[3],
                default_kp_[4], default_kp_[5], default_kd_[0], default_kd_[1],
                default_kd_[2], default_kd_[3], default_kd_[4], default_kd_[5]);
  } catch (const std::exception &e) {
    RCLCPP_WARN(rclcpp::get_logger("OpenMMArmHW"),
                "无法从 YAML 加载配置: %s，使用默认值", e.what());
  }

  // 初始化状态和命令向量
  hw_states_position_.resize(num_joints_, 0.0);
  hw_states_velocity_.resize(num_joints_, 0.0);
  hw_states_effort_.resize(num_joints_, 0.0);
  hw_commands_position_.resize(num_joints_, 0.0);
  hw_commands_velocity_.resize(num_joints_, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("OpenMMArmHW"),
              "初始化完成 - 通信: %s, 关节数: %zu, 控制模式: %s",
              communication_mode_.c_str(), num_joints_, control_mode_.c_str());

  return CallbackReturn::SUCCESS;
}

CallbackReturn
OpenMMArmHW::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenMMArmHW"), "配置硬件接口，通信模式: %s",
              communication_mode_.c_str());

  // 根据通信模式创建 IOInterface
  if (communication_mode_ == "SIM") {
    // 如果未指定模型路径，自动使用 openmmarm_description 中的 URDF
    std::string model_path = sim_model_path_;
    if (model_path.empty()) {
      try {
        std::string desc_pkg = ament_index_cpp::get_package_share_directory(
            "openmmarm_description");
        model_path = desc_pkg + "/urdf/openmmarm.urdf.xacro";

        // 如果 xacro 不存在，尝试 urdf
        if (!std::filesystem::exists(model_path)) {
          model_path = desc_pkg + "/urdf/openmmarm.urdf";
        }
      } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("OpenMMArmHW"),
                     "无法获取 openmmarm_description 路径: %s", e.what());
        return CallbackReturn::ERROR;
      }
    }

    RCLCPP_INFO(rclcpp::get_logger("OpenMMArmHW"),
                "创建 IOMujoco: model=%s, viewer=%s", model_path.c_str(),
                sim_viewer_ ? "true" : "false");

    ioInter_ =
        std::make_unique<IOMujoco>(model_path, 1.0 / update_rate_, sim_viewer_);
  } else if (communication_mode_ == "SERIAL") {
    RCLCPP_INFO(
        rclcpp::get_logger("OpenMMArmHW"),
        "创建 IOSerial: %s @ %d bps (link_timeout=%d ms)",
        serial_port_.c_str(), serial_baud_rate_, serial_link_timeout_ms_);

    ioInter_ = std::make_unique<IOSerial>(serial_port_, serial_baud_rate_,
                                          serial_link_timeout_ms_);
  } else {
    RCLCPP_INFO(rclcpp::get_logger("OpenMMArmHW"),
                "创建 IOUDP: %s:%d (本地端口: %d)", mcu_ip_.c_str(), mcu_port_,
                mcu_local_port_);

    ioInter_ = std::make_unique<IOUDP>(mcu_ip_, mcu_port_, mcu_local_port_);
  }

  if (!ioInter_->init()) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenMMArmHW"), "IOInterface 初始化失败");
    return CallbackReturn::ERROR;
  }

  // 首次 sendRecv，获取初始关节位置
  ioInter_->sendRecv(&lowCmd_, &lowState_);

  if (ioInter_->isConnected()) {
    for (size_t i = 0; i < 6; ++i) {
      hw_states_position_[i] = lowState_.q[i];
      hw_commands_position_[i] = hw_states_position_[i];
    }
    RCLCPP_INFO(rclcpp::get_logger("OpenMMArmHW"),
                "IOInterface 已连接，读取了初始位置");
  } else {
    RCLCPP_WARN(rclcpp::get_logger("OpenMMArmHW"),
                "IOInterface 尚未响应，将在激活后重试");
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenMMArmHW"), "硬件配置完成");

  // 阻抗模式下初始化动力学前馈补偿
  if (control_mode_ == "impedance") {
    std::string dynamics_urdf;
    try {
      std::string desc_pkg =
          ament_index_cpp::get_package_share_directory("openmmarm_description");
      dynamics_urdf = desc_pkg + "/urdf/openmmarm.urdf";
      if (!std::filesystem::exists(dynamics_urdf)) {
        dynamics_urdf = desc_pkg + "/urdf/openmmarm.urdf.xacro";
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN(rclcpp::get_logger("OpenMMArmHW"),
                  "无法获取 URDF 路径用于动力学计算: %s", e.what());
    }

    if (!dynamics_urdf.empty()) {
      dynamics_ = std::make_unique<ArmDynamics>();
      if (dynamics_->init(dynamics_urdf)) {
        RCLCPP_INFO(rclcpp::get_logger("OpenMMArmHW"),
                    "ArmDynamics 初始化成功，阻抗模式前馈补偿已启用");
      } else {
        RCLCPP_WARN(rclcpp::get_logger("OpenMMArmHW"),
                    "ArmDynamics 初始化失败，将不使用前馈补偿");
        dynamics_.reset();
      }
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn
OpenMMArmHW::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenMMArmHW"), "激活硬件接口...");

  // 读取当前位置作为初始命令
  for (size_t i = 0; i < num_joints_; ++i) {
    hw_commands_position_[i] = hw_states_position_[i];
  }

  // 用当前位置 + PD 增益填充 lowCmd
  for (size_t i = 0; i < 6; ++i) {
    lowCmd_.q[i] = static_cast<float>(hw_commands_position_[i]);
    lowCmd_.dq[i] = 0.0f;
    lowCmd_.tau[i] = 0.0f;
    lowCmd_.kp[i] = static_cast<float>(default_kp_[i]);
    lowCmd_.kd[i] = static_cast<float>(default_kd_[i]);
  }

  ioInter_->sendRecv(&lowCmd_, &lowState_);

  RCLCPP_INFO(rclcpp::get_logger("OpenMMArmHW"), "硬件已激活");
  return CallbackReturn::SUCCESS;
}

CallbackReturn
OpenMMArmHW::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenMMArmHW"), "停用硬件接口...");

  // 置零 kp/kd/tau，保持当前位置
  for (size_t i = 0; i < 6; ++i) {
    lowCmd_.q[i] = lowState_.q[i];
    lowCmd_.dq[i] = 0.0f;
    lowCmd_.tau[i] = 0.0f;
    lowCmd_.kp[i] = 0.0f;
    lowCmd_.kd[i] = 0.0f;
  }

  ioInter_->sendRecv(&lowCmd_, &lowState_);

  RCLCPP_INFO(rclcpp::get_logger("OpenMMArmHW"), "硬件已停用");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
OpenMMArmHW::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_states_position_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &hw_states_velocity_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
        &hw_states_effort_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
OpenMMArmHW::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_commands_position_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &hw_commands_velocity_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type
OpenMMArmHW::read(const rclcpp::Time & /*time*/,
                  const rclcpp::Duration & /*period*/) {
  // 从 lowState_ 读取 q/dq/tau_est
  for (size_t i = 0; i < 6; ++i) {
    hw_states_position_[i] = lowState_.q[i];
    hw_states_velocity_[i] = lowState_.dq[i];
    hw_states_effort_[i] = lowState_.tau_est[i];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
OpenMMArmHW::write(const rclcpp::Time & /*time*/,
                   const rclcpp::Duration & /*period*/) {
  // 填充 lowCmd_ (q/dq/kp/kd)
  for (size_t i = 0; i < 6; ++i) {
    lowCmd_.q[i] = static_cast<float>(hw_commands_position_[i]);
    lowCmd_.dq[i] = static_cast<float>(hw_commands_velocity_[i]);
    lowCmd_.kp[i] = static_cast<float>(default_kp_[i]);
    lowCmd_.kd[i] = static_cast<float>(default_kd_[i]);
  }

  // 阻抗模式：计算逆动力学前馈力矩
  if (control_mode_ == "impedance" && dynamics_) {
    double q[6], dq[6], ddq_d[6] = {}, tau_ff[6];
    for (size_t i = 0; i < 6; ++i) {
      q[i] = hw_states_position_[i];
      dq[i] = hw_states_velocity_[i];
    }
    dynamics_->computeFeedforward(q, dq, ddq_d, tau_ff);
    for (size_t i = 0; i < 6; ++i) {
      lowCmd_.tau[i] = static_cast<float>(tau_ff[i]);
    }
  } else {
    for (size_t i = 0; i < 6; ++i) {
      lowCmd_.tau[i] = 0.0f;
    }
  }

  // 发送指令，接收状态
  const bool comm_ok = ioInter_->sendRecv(&lowCmd_, &lowState_);
  if (!comm_ok) {
    ++comm_fail_count_;
    if (comm_healthy_last_cycle_ || (comm_fail_count_ % 500 == 0)) {
      RCLCPP_WARN(rclcpp::get_logger("OpenMMArmHW"),
                  "底层通信失败/超时 (连续失败: %zu)", comm_fail_count_);
    }
    comm_healthy_last_cycle_ = false;
    return hardware_interface::return_type::OK;
  }

  if (!comm_healthy_last_cycle_) {
    RCLCPP_INFO(rclcpp::get_logger("OpenMMArmHW"),
                "底层通信恢复 (此前连续失败: %zu)", comm_fail_count_);
  }
  comm_healthy_last_cycle_ = true;
  comm_fail_count_ = 0;

  return hardware_interface::return_type::OK;
}

// Gripper Action 回调实现
rclcpp_action::GoalResponse OpenMMArmHW::handle_gripper_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const GripperAction::Goal> /*goal*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenMMArmHW"), "收到夹爪控制请求");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse OpenMMArmHW::handle_gripper_cancel(
    const std::shared_ptr<GoalHandleGripper> /*goal_handle*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenMMArmHW"), "收到夹爪取消请求");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void OpenMMArmHW::handle_gripper_accepted(
    const std::shared_ptr<GoalHandleGripper> goal_handle) {
  std::thread{[this, goal_handle]() {
    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<GripperAction::Feedback>();
    auto result = std::make_shared<GripperAction::Result>();

    gripper_command_ = goal->command.position;

    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
      if (std::abs(gripper_position_ - gripper_command_) < 0.01) {
        result->position = gripper_position_;
        result->reached_goal = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(rclcpp::get_logger("OpenMMArmHW"), "夹爪到达目标位置");
        return;
      }

      feedback->position = gripper_position_;
      goal_handle->publish_feedback(feedback);

      rate.sleep();
    }
  }}.detach();
}

} // namespace openmmarm_hw

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(openmmarm_hw::OpenMMArmHW,
                       hardware_interface::SystemInterface)
