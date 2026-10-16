// ROS 服务回调：connect/stop/info/power/move/FK/IK。

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
void XcoreRobotInterface::on_connect(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    response->success = connect_robot(response->message);
  }

void XcoreRobotInterface::on_stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::string message;
    if (!require_robot(message)) {
      response->success = false;
      response->message = message;
      return;
    }

    stop_servo_session();
    std::error_code ec;
    robot_->stop(ec);
    if (ec) {
      response->success = false;
      response->message = error_message("stop", ec);
      set_status(response->message);
      return;
    }
    response->success = true;
    response->message = "stop requested";
    set_status(response->message);
  }
  // 读取机器人型号、版本、SDK 版本、关节数
void XcoreRobotInterface::on_get_robot_info(
    const std::shared_ptr<vr_srv::GetRobotInfo::Request> /*request*/,
    std::shared_ptr<vr_srv::GetRobotInfo::Response> response)
  {
    std::string message;
    if (!require_robot(message)) {
      response->success = false;
      response->message = message;
      return;
    }
    std::error_code ec;
    const auto info = robot_->robotInfo(ec);
    if (ec) {
      response->success = false;
      response->message = error_message("robotInfo", ec);
      return;
    }
    response->success = true;
    response->message = "ok";
    response->version = info.version;
    response->type = info.type;
    response->sdk_version = robot_->sdkVersion();
    response->joint_num = info.joint_num;
  }

void XcoreRobotInterface::on_set_power(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    response->success = set_power(request->data, response->message);
  }

void XcoreRobotInterface::on_set_operate_mode(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    response->success = set_automatic(request->data, response->message);
  }

void XcoreRobotInterface::on_servo_enable(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    response->success = enable_servo(request->data, response->message);
  }
  // 关节运动
void XcoreRobotInterface::on_joint_move(
    const std::shared_ptr<vr_srv::JointMove::Request> request,
    std::shared_ptr<vr_srv::JointMove::Response> response)
  {
    // MOVEJ 是非实时离散命令；move_mode=0 为绝对角度，非 0 为相对当前角度。
    std::string message;
    if (!require_robot(message)) {
      response->success = false;
      response->message = message;
      return;
    }
    if (request->joint_positions.size() != kDof || !finite_values(request->joint_positions)) {
      response->success = false;
      response->message = "joint_move requires 7 finite joint positions";
      return;
    }
    if (!prepare_automatic_powered(message)) {
      response->success = false;
      response->message = message;
      return;
    }

    try {
      // 非实时 MOVEJ 会独占控制模式，进入前先停掉可能存在的 SERVOJ 会话。
      stop_servo_session();
      std::vector<double> target = request->joint_positions;
      if (request->move_mode != 0) {
        const auto current = current_joint_positions_or_sdk();
        for (std::size_t i = 0; i < kDof; ++i) {
          target[i] += current[i];
        }
      }

      std::error_code ec;
      robot_->setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
      if (ec) {
        response->success = false;
        response->message = error_message("setMotionControlMode(NrtCommand)", ec);
        return;
      }
      robot_->moveReset(ec);
      ec.clear();
      std::string cmd_id;
      const double speed = request->speed > 0.0 ? request->speed : rokae::USE_DEFAULT;
      robot_->moveAppend(rokae::MoveAbsJCommand(rokae::JointPosition(target), speed), cmd_id, ec);
      if (ec) {
        response->success = false;
        response->message = error_message("moveAppend(MoveAbsJ)", ec);
        return;
      }
      robot_->moveStart(ec);
      if (ec) {
        response->success = false;
        response->message = error_message("moveStart", ec);
        return;
      }
      if (request->is_blocking) {
        wait_until_idle(30.0);
      }
      response->success = true;
      response->message = "joint_move accepted: " + cmd_id;
      set_status(response->message);
    } catch (const std::exception & e) {
      response->success = false;
      response->message = std::string("joint_move exception: ") + e.what();
      set_status(response->message);
    }
  }

void XcoreRobotInterface::on_linear_move(
    const std::shared_ptr<vr_srv::LinearMove::Request> request,
    std::shared_ptr<vr_srv::LinearMove::Response> response)
  {
    // MOVEL 输入为 [x,y,z,roll,pitch,yaw,(elbow)]；平移单位由参数转换到 SDK 的米。
    std::string message;
    if (!require_robot(message)) {
      response->success = false;
      response->message = message;
      return;
    }
    if ((request->end_position.size() != 6 && request->end_position.size() != 7) ||
      !finite_values(request->end_position))
    {
      response->success = false;
      response->message = "linear_move requires [x,y,z,rx,ry,rz] plus optional elbow";
      return;
    }
    if (!prepare_automatic_powered(message)) {
      response->success = false;
      response->message = message;
      return;
    }

    try {
      const auto converted = convert_input_pose(request->end_position, cartesian_input_unit_);
      stop_servo_session();
      auto target = to_cartesian(converted);
      if (request->move_mode != 0) {
        std::error_code ec;
        auto current = robot_->cartPosture(rokae::CoordinateType::endInRef, ec);
        if (ec) {
          response->success = false;
          response->message = error_message("cartPosture", ec);
          return;
        }
        for (int i = 0; i < 3; ++i) {
          target.trans[i] += current.trans[i];
          target.rpy[i] += current.rpy[i];
        }
      }

      std::error_code ec;
      robot_->setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
      if (ec) {
        response->success = false;
        response->message = error_message("setMotionControlMode(NrtCommand)", ec);
        return;
      }
      robot_->moveReset(ec);
      ec.clear();
      std::string cmd_id;
      const double speed = request->speed > 0.0 ? request->speed : rokae::USE_DEFAULT;
      robot_->moveAppend(rokae::MoveLCommand(target, speed), cmd_id, ec);
      if (ec) {
        response->success = false;
        response->message = error_message("moveAppend(MoveL)", ec);
        return;
      }
      robot_->moveStart(ec);
      if (ec) {
        response->success = false;
        response->message = error_message("moveStart", ec);
        return;
      }
      if (request->is_blocking) {
        wait_until_idle(30.0);
      }
      response->success = true;
      response->message = "linear_move accepted: " + cmd_id;
      set_status(response->message);
    } catch (const std::exception & e) {
      response->success = false;
      response->message = std::string("linear_move exception: ") + e.what();
      set_status(response->message);
    }
  }
  // 逆解
void XcoreRobotInterface::on_kine_inverse(
    const std::shared_ptr<vr_srv::KineInverse7::Request> request,
    std::shared_ptr<vr_srv::KineInverse7::Response> response)
  {
    // 使用控制器/SDK 自带模型，与仿真的 MoveIt /compute_ik 不是同一个求解器。
    std::string message;
    if (!require_robot(message)) {
      response->success = false;
      response->message = message;
      return;
    }
    if ((request->cartesian_pose.size() != 6 && request->cartesian_pose.size() != 7) ||
      !finite_values(request->cartesian_pose))
    {
      response->success = false;
      response->message = "cartesian_pose must be [x,y,z,rx,ry,rz] plus optional elbow";
      return;
    }
    const auto converted = convert_input_pose(request->cartesian_pose, cartesian_input_unit_);
    std::error_code ec;
    const auto joints = robot_->model().calcIk(to_cartesian(converted), ec);
    if (ec) {
      response->success = false;
      response->message = error_message("calcIk", ec);
      return;
    }
    response->success = true;
    response->message = "ok";
    response->joint_positions = array_to_vector(joints);
  }
  // 正解
void XcoreRobotInterface::on_calculate_fk(
    const std::shared_ptr<vr_srv::CalculateFK::Request> request,
    std::shared_ptr<vr_srv::CalculateFK::Response> response)
  {
    // 输出位置单位由 cartesian_output_unit 控制，RPY 保持弧度。
    std::string message;
    if (!require_robot(message)) {
      response->success = false;
      response->message = message;
      return;
    }
    if (request->joint_positions.size() != kDof || !finite_values(request->joint_positions)) {
      response->success = false;
      response->message = "calculate_fk requires 7 finite joint positions";
      return;
    }
    std::array<double, kDof> joints{};
    std::copy(request->joint_positions.begin(), request->joint_positions.end(), joints.begin());
    std::error_code ec;
    const auto cart = robot_->model().calcFk(joints, ec);
    if (ec) {
      response->success = false;
      response->message = error_message("calcFk", ec);
      return;
    }
    response->success = true;
    response->message = "ok";
    response->cartesian_pose = from_cartesian(cart);
  }
  // 关节伺服

}  // namespace xcore_gateway
