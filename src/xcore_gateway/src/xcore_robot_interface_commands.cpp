// 外部命令入口：robot_command_servo_j/p 和 ArmServoCommand。

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include "tf2/LinearMath/Matrix3x3.h"
#include "xcore_gateway/xcore_robot_interface.hpp"
#include "xcore_gateway/xcore_robot_interface_utils.hpp"

#include "rokae/exception.h"

using namespace std::chrono_literals;

namespace xcore_gateway
{
void XcoreRobotInterface::on_servo_j(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->position.size() != kDof || !finite_values(msg->position)) {
      set_status("rejected robot_command_servo_j: expected 7 finite positions");
      return;
    }
    std::vector<double> target = msg->position;
    // 与 JAKA 参考保持兼容：name 为空表示绝对关节位姿，非空表示增量。
    if (!msg->name.empty()) {
      const auto current = current_joint_positions_or_sdk();
      for (std::size_t i = 0; i < kDof; ++i) {
        target[i] += current[i];
      }
    }
    accept_servo_target(target, "robot_command_servo_j");
  }
  // 末端位姿伺服
void XcoreRobotInterface::on_servo_p(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if ((msg->position.size() != 6 && msg->position.size() != 7) || !finite_values(msg->position)) {
      set_status("rejected robot_command_servo_p: expected pose [x,y,z,rx,ry,rz,(elbow)]");
      return;
    }
    std::string message;
    if (!require_robot(message)) {
      return;
    }
    const auto pose = convert_input_pose(msg->position, cartesian_input_unit_);

    std::error_code ec;
    // 先用 SDK calcIk 保持原有失败语义，再用 all IK 从上一目标附近选解，减少冗余轴跳解。
    auto model = robot_->model();
    const auto cartesian = to_cartesian(pose);
    const auto joints = model.calcIk(cartesian, ec);
    if (ec) {
      handle_ik_failure(ec, "calcIk for servo_p");
      return;
    }
    std::vector<double> target = array_to_vector(joints);
    std::vector<double> reference;
    {
      std::lock_guard<std::mutex> lock(servo_mutex_);
      reference = latest_servo_target_;
    }
    if (reference.size() != kDof || !finite_values(reference)) {
      reference = current_joint_positions_or_sdk();
    }
    std::vector<std::vector<int>> calc_confs;
    std::error_code all_ik_ec;
    const auto all_solutions = model.calcAllIkSolutions(cartesian, calc_confs, all_ik_ec);
    if (!all_ik_ec) {
      const auto selected = select_closest_ik_solution(
        all_solutions, reference, joint_limits_min_, joint_limits_max_);
      if (!selected.empty()) {
        target = selected;
      }
    }

    sensor_msgs::msg::JointState raw_msg;
    raw_msg.header.stamp = get_clock()->now();
    raw_msg.name = joint_names_;
    raw_msg.position = target;
    debug_raw_ik_target_pub_->publish(raw_msg);

    reset_ik_fail_state();
    accept_servo_target(target, "robot_command_servo_p");
  }
  // 接受 /ArmServoCommand
void XcoreRobotInterface::on_arm_servo_command(const vr_robot_interfaces::msg::ArmServoCommand::SharedPtr msg)
  {
    // 单臂允许空 arm_name；双臂 launch 会关闭该通配并通过 remap 隔离左右臂命令。
    if (msg->arm_name.empty() && !accept_empty_arm_name_) {
      return;
    }
    if (!msg->arm_name.empty() && msg->arm_name != arm_name_) {
      return;
    }
    if (!msg->enabled) {
      stop_servo_session();
      set_status("ArmServoCommand enabled=false; servo disabled");
      return;
    }
    // JOINT 可直接进入统一安全入口；POSE 先由 xCore 模型 IK 转成七轴目标。
    if (msg->mode == vr_robot_interfaces::msg::ArmServoCommand::MODE_JOINT) {
      if (msg->joint_positions.size() != kDof || !finite_values(msg->joint_positions)) {
        set_status("rejected ArmServoCommand MODE_JOINT: expected 7 finite positions");
        return;
      }
      accept_servo_target(
        std::vector<double>(msg->joint_positions.begin(), msg->joint_positions.end()),
        "ArmServoCommand");
      return;
    }

    if (msg->mode == vr_robot_interfaces::msg::ArmServoCommand::MODE_POSE) {
      handle_pose_command(msg->target_pose);
      return;
    }

    set_status("rejected ArmServoCommand: unsupported mode");
  }
  // 锁定 VR 初始位姿和机械臂当前 TCP 位姿，作为相对运动参考

}  // namespace xcore_gateway
