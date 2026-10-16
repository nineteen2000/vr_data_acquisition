// 伺服目标缓存、关节安全限幅、调试目标发布和 ServoJ 定时发送。

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
void XcoreRobotInterface::accept_servo_target(const std::vector<double> & target, const std::string & source)
  {
    // 该函数只更新共享目标，不直接调用 SDK。校验顺序是会话使能、单步限幅、绝对限位。
    {
      std::lock_guard<std::mutex> lock(servo_mutex_);
      if (!servo_enabled_) {
        set_status("servo target from " + source + " ignored: servo_move_enable is false");
        return;
      }
    }

    const auto current = current_joint_positions_or_sdk();
    if (current.size() != kDof || !finite_values(current)) {
      set_status("servo target from " + source + " rejected: cannot read current joints for step limit");
      return;
    }

    sensor_msgs::msg::JointState error_msg;
    error_msg.header.stamp = get_clock()->now();
    error_msg.name = joint_names_;
    error_msg.position.resize(kDof);
    error_msg.effort.resize(2, 0.0);
    double max_abs_error = 0.0;
    std::size_t max_error_joint = 0;
    for (std::size_t i = 0; i < kDof; ++i) {
      const double error = target[i] - current[i];
      error_msg.position[i] = error;
      const double abs_error = std::fabs(error);
      if (abs_error > max_abs_error) {
        max_abs_error = abs_error;
        max_error_joint = i;
      }
    }
    error_msg.effort[0] = max_abs_error;
    error_msg.effort[1] = static_cast<double>(max_error_joint + 1);
    debug_servo_error_pub_->publish(error_msg);

    std::vector<double> limited_target;
    bool step_limited = false;
    limit_servo_step(target, current, limited_target, step_limited);
    bool joint_limited = false;
    limit_joint_positions(limited_target, joint_limited);
    {
      std::lock_guard<std::mutex> lock(servo_mutex_);
      if (!servo_enabled_) {
        set_status("servo target from " + source + " ignored: servo_move_enable is false");
          return;
      }
      latest_servo_target_ = limited_target;
      have_servo_target_ = true;
      received_servo_command_ = true;
      last_servo_command_time_ = get_clock()->now();
    }
    publish_debug_servo_target(limited_target);
    set_status(
      std::string("accepted servo target from ") + source +
      (step_limited ? " [step-limited]" : "") +
      (joint_limited ? " [joint-limited]" : ""));
  }
  // 调试servo话题
void XcoreRobotInterface::publish_debug_servo_target(const std::vector<double> & target)
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = get_clock()->now();
    msg.name = joint_names_;
    msg.position = target;
    debug_servo_target_pub_->publish(msg);
  }
void XcoreRobotInterface::publish_debug_servo_sent_target(
    const std::vector<double> & target,
    const rclcpp::Time & stamp)
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = stamp;
    msg.name = joint_names_;
    msg.position = target;
    debug_servo_sent_target_pub_->publish(msg);
  }
  // 限制每次 Servo 目标变化不超过 servo_max_step_rad_
bool XcoreRobotInterface::limit_servo_step(
    const std::vector<double> & target,
    const std::vector<double> & current,
    std::vector<double> & limited_target,
    bool & step_limited)
  {
    limited_target = target;
    step_limited = false;
    for (std::size_t i = 0; i < kDof; ++i) {
      const double delta = target[i] - current[i];
      if (std::fabs(delta) > servo_max_step_rad_) {
        limited_target[i] = current[i] + std::copysign(servo_max_step_rad_, delta);
        step_limited = true;
      }
    }
    return true;
  }
  // 软限位裁目标角
void XcoreRobotInterface::limit_joint_positions(std::vector<double> & target, bool & joint_limited) const
  {
    joint_limited = false;
    if (joint_limits_min_.size() != kDof || joint_limits_max_.size() != kDof) {
      return;
    }
    for (std::size_t i = 0; i < kDof; ++i) {
      const double clamped = std::clamp(target[i], joint_limits_min_[i], joint_limits_max_[i]);
      if (clamped != target[i]) {
        target[i] = clamped;
        joint_limited = true;
      }
    }
  }
  // 向 SDK 发送最新关节目标
void XcoreRobotInterface::servo_tick()
  {
    // 真正的实时发送点。watchdog 从第一条外部命令开始计时，超时后关闭
    std::string status_after_unlock;
    bool publish_status_after_unlock = false;
    std::vector<double> sent_target;
    rclcpp::Time sent_stamp{0, 0, RCL_ROS_TIME};
    bool publish_sent_target_after_unlock = false;
    {
      std::lock_guard<std::mutex> lock(servo_mutex_);
      if (!servo_enabled_ || !rt_controller_) {
        return;
      }

      bool timed_out = false;
      if (received_servo_command_) {
        double elapsed = (get_clock()->now() - last_servo_command_time_).seconds();
        // 超过0.1s没有新目标就停止
        if (elapsed > servo_command_timeout_s_) {
          RCLCPP_WARN(get_logger(),
            "Servo command timeout (%.3fs > %.3fs); emergency stop servo",
            elapsed, servo_command_timeout_s_);
          stop_servo_session_locked();
          status_after_unlock = "servo watchdog timeout; servo disabled";
          publish_status_after_unlock = true;
          timed_out = true;
        }
      }

      if (timed_out) {
        // Leave the critical section, then publish the timeout status below.
      } else if (!have_servo_target_) {
        return;
      } else {
        // xCoreSDK 示例在 sendCommand 前更新实时状态；这里同步暴露实时侧错误，避免控制器已停而 ROS 仍显示 accepted。
        try {
          robot_->updateRobotState(std::chrono::milliseconds(1));
          rt_controller_->sendCommand(rokae::JointPosition(latest_servo_target_));
          sent_target = latest_servo_target_;
          sent_stamp = get_clock()->now();
          publish_sent_target_after_unlock = true;
        } catch (const std::exception & e) {
          status_after_unlock = std::string("servo sendCommand exception: ") + e.what();
          publish_status_after_unlock = true;
        }
      }
    }
    if (publish_status_after_unlock && !status_after_unlock.empty()) {
      set_status(status_after_unlock);
    }
    if (publish_sent_target_after_unlock) {
      publish_debug_servo_sent_target(sent_target, sent_stamp);
    }
  }
  // 周期读取关节角、速度、力矩、TCP 位姿并发布

}  // namespace xcore_gateway
