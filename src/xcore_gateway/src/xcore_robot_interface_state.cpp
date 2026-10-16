// 机器人状态发布、状态文本发布和 SDK/ROS 位姿转换工具。

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
void XcoreRobotInterface::publish_state()
  {
    // 状态读取是尽力而为：位置失败则整帧停止；速度/力矩失败时用零值降级。
    publish_status();
    if (!connected()) {
      return;
    }

    std::error_code ec;
    const auto now = get_clock()->now();

    auto positions = robot_->jointPos(ec);
    if (ec) {
      set_status(error_message("jointPos", ec));
      return;
    }
    current_positions_ = array_to_vector(positions);

    auto velocities = robot_->jointVel(ec);
    std::array<double, kDof> zero{};
    if (ec) {
      velocities = zero;
      ec.clear();
    }
    auto torques = robot_->jointTorque(ec);
    if (ec) {
      torques = zero;
      ec.clear();
    }

    sensor_msgs::msg::JointState joint_msg;
    joint_msg.header.stamp = now;
    joint_msg.name = joint_names_;
    joint_msg.position = array_to_vector(positions);
    joint_msg.velocity = array_to_vector(velocities);
    joint_msg.effort = array_to_vector(torques);
    joint_state_pub_->publish(joint_msg);

    sensor_msgs::msg::JointState vel_msg;
    vel_msg.header.stamp = now;
    vel_msg.name = joint_names_;
    vel_msg.velocity = array_to_vector(velocities);
    joint_state_vel_pub_->publish(vel_msg);

    sensor_msgs::msg::JointState torque_msg;
    torque_msg.header.stamp = now;
    torque_msg.name = joint_names_;
    torque_msg.position = array_to_vector(torques);
    torque_pub_->publish(torque_msg);

    const auto cart = robot_->cartPosture(rokae::CoordinateType::endInRef, ec);
    if (!ec) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = now;
      pose.header.frame_id = "base";
      double scale = (cartesian_output_unit_ == "mm") ? 1000.0 : 1.0;
      pose.pose.position.x = cart.trans[0] * scale;
      pose.pose.position.y = cart.trans[1] * scale;
      pose.pose.position.z = cart.trans[2] * scale;
      tf2::Quaternion q;
      q.setRPY(cart.rpy[0], cart.rpy[1], cart.rpy[2]);
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();
      tcp_pose_pub_->publish(pose);
    }
  }

void XcoreRobotInterface::publish_status()
  {
    std_msgs::msg::String msg;
    bool connected_value = connected();
    bool servo_enabled_value = false;
    {
      std::lock_guard<std::mutex> lock(servo_mutex_);
      servo_enabled_value = servo_enabled_;
    }
    std::ostringstream oss;
    oss << "connected=" << (connected_value ? "true" : "false")
        << ", servo_enabled=" << (servo_enabled_value ? "true" : "false")
        << ", last_status=" << last_status_;
    msg.data = oss.str();
    status_pub_->publish(msg);
  }
  // 当前关节位置
std::vector<double> XcoreRobotInterface::current_joint_positions_or_sdk()
  {
    if (finite_values(current_positions_)) {
      return current_positions_;
    }
    std::string message;
    if (!require_robot(message)) {
      return current_positions_;
    }
    std::error_code ec;
    const auto positions = robot_->jointPos(ec);
    if (ec) {
      set_status(error_message("jointPos", ec));
      return current_positions_;
    }
    current_positions_ = array_to_vector(positions);
    return current_positions_;
  }
  // 阻塞等待机器人进入 idle 状态
bool XcoreRobotInterface::wait_until_idle(double timeout_s)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    while (std::chrono::steady_clock::now() < deadline) {
      std::error_code ec;
      const auto state = robot_->operationState(ec);
      if (!ec && (state == rokae::OperationState::idle || state == rokae::OperationState::unknown)) {
        return true;
      }
      rclcpp::sleep_for(100ms);
    }
    return false;
  }
  // 转sdk所需的输入结构
rokae::CartesianPosition XcoreRobotInterface::to_cartesian(const std::vector<double> & values) const
  {
    rokae::CartesianPosition cart;
    cart.trans[0] = values[0];
    cart.trans[1] = values[1];
    cart.trans[2] = values[2];
    cart.rpy[0] = values[3];
    cart.rpy[1] = values[4];
    cart.rpy[2] = values[5];
    if (values.size() >= 7) {
      cart.hasElbow = true;
      cart.elbow = values[6];
    }
    return cart;
  }
  // 转输入xyz的mm为m
std::vector<double> XcoreRobotInterface::convert_input_pose(const std::vector<double> & values, const std::string & unit) const
  {
    // VR 链路的位置量按 mm 表达，xCoreSDK 的笛卡尔位置按 m 表达。
    std::vector<double> converted = values;
    if (unit == "mm") {
      for (int i = 0; i < 3; ++i) {
        converted[i] *= 0.001;
      }
    }
    return converted;
  }
  // 把 SDK 位姿转成 ROS 服务返回数组
std::vector<double> XcoreRobotInterface::from_cartesian(const rokae::CartesianPosition & cart) const
  {
    const double scale = cartesian_output_unit_ == "mm" ? 1000.0 : 1.0;
    return {
      cart.trans[0] * scale, cart.trans[1] * scale, cart.trans[2] * scale,
      cart.rpy[0], cart.rpy[1], cart.rpy[2], cart.elbow};
  }

void XcoreRobotInterface::set_status(const std::string & status)
  {
    last_status_ = status;
  }


}  // namespace xcore_gateway
