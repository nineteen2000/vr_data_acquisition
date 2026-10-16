// 连接、上下电、自动模式和 ServoJ 会话生命周期。

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
  // robot_ 非空表示已链接控制器
bool XcoreRobotInterface::connected() const
  {
    return static_cast<bool>(robot_);
  }

  // 连接
bool XcoreRobotInterface::connect_robot(std::string & message)
  {
    // 连接成功直接返回。
    if (connected()) {
      message = "already connected";
      return true;
    }

    try {
      robot_ = std::make_shared<rokae::xMateErProRobot>(robot_ip_, local_ip_);
      std::error_code ec;
      robot_->connectToRobot(ec);
      // 链接失败立刻清空
      if (ec) {
        // 释放sdk对象
        robot_.reset();
        message = error_message("connectToRobot", ec);
        set_status(message);
        return false;
      }

      const auto info = robot_->robotInfo(ec);
      if (ec) {
        const auto info_ec = ec;
        std::error_code disconnect_ec;
        robot_->disconnectFromRobot(disconnect_ec);
        robot_.reset();
        message = error_message("robotInfo", info_ec);
        set_status(message);
        return false;
      }
      message = "connected to " + info.type + ", controller=" + info.version;
      set_status(message);
      return true;
    } catch (const std::exception & e) {
      robot_.reset();
      message = std::string("connect exception: ") + e.what();
      set_status(message);
      return false;
    }
  }

  // 机械臂执行时的前置检验
bool XcoreRobotInterface::require_robot(std::string & message)
  {
    if (connected()) {
      return true;
    }
    message = "robot is not connected; call connect or set connect_on_start:=true";
    set_status(message);
    return false;
  }

bool XcoreRobotInterface::set_power(bool on, std::string & message)
  {
    if (!require_robot(message)) {
      return false;
    }
    std::error_code ec;
    robot_->setPowerState(on, ec);
    if (ec) {
      message = error_message("setPowerState", ec);
      set_status(message);
      return false;
    }
    message = on ? "power on" : "power off";
    set_status(message);
    return true;
  }
  // 设置自动模式
bool XcoreRobotInterface::set_automatic(bool automatic, std::string & message)
  {
    if (!require_robot(message)) {
      return false;
    }
    std::error_code ec;
    robot_->setOperateMode(automatic ? rokae::OperateMode::automatic : rokae::OperateMode::manual, ec);
    if (ec) {
      message = error_message("setOperateMode", ec);
      set_status(message);
      return false;
    }
    message = automatic ? "operate mode automatic" : "operate mode manual";
    set_status(message);
    return true;
  }
  // 自动模式并上电
bool XcoreRobotInterface::prepare_automatic_powered(std::string & message)
  {
    // 所有会运动的接口都复用此门槛，顺序固定为自动模式 -> 上电。
    if (!set_automatic(true, message)) {
      return false;
    }
    return set_power(true, message);
  }
  // 关节伺服
bool XcoreRobotInterface::enable_servo(bool enable, std::string & message)
  {
    if (!require_robot(message)) {
      return false;
    }
    if (!enable) {
      stop_servo_session();
      clear_pose_reference();
      message = "servo move disabled";
      set_status(message);
      return true;
    }
    if (servo_enabled_) {
      message = "servo move already enabled";
      return true;
    }
    if (!prepare_automatic_powered(message)) {
      return false;
    }
    // servoj
    try {
      std::error_code ec;
      bool entered_rt_mode = false;
      std::shared_ptr<rokae::RtMotionControlCobot<kDof>> rt_controller;
      auto rollback_failed_rt_enable = [&]() {
        if (rt_controller) {
          try {
            rt_controller->stopServoJoint();
            rt_controller->stopMove();
          } catch (const std::exception & e) {
            RCLCPP_WARN(get_logger(), "rollback servo enable stop realtime motion error: %s", e.what());
          }
        }
        {
          std::lock_guard<std::mutex> lock(servo_mutex_);
          rt_controller_.reset();
          servo_enabled_ = false;
          have_servo_target_ = false;
          received_servo_command_ = false;
        }
        clear_pose_reference();
        if (entered_rt_mode && robot_) {
          std::error_code cleanup_ec;
          robot_->setMotionControlMode(rokae::MotionControlMode::Idle, cleanup_ec);
          if (cleanup_ec) {
            RCLCPP_WARN(get_logger(), "rollback servo enable setMotionControlMode(Idle) error: %s",
              cleanup_ec.message().c_str());
          }
          robot_->stopReceiveRobotState();
        }
      };

      robot_->setRtNetworkTolerance(static_cast<unsigned>(std::clamp(rt_network_tolerance_, 0, 100)), ec);
      if (ec) {
        message = error_message("setRtNetworkTolerance", ec);
        set_status(message);
        return false;
      }
      // 使能实时运动控制
      robot_->setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
      if (ec) {
        message = error_message("setMotionControlMode(RtCommand)", ec);
        set_status(message);
        return false;
      }
      entered_rt_mode = true;
      // 获取实时运动控制器对象
      rt_controller = robot_->getRtMotionController().lock();
      if (!rt_controller) {
        message = "getRtMotionController returned null";
        rollback_failed_rt_enable();
        set_status(message);
        return false;
      }

      robot_->startReceiveRobotState(
        std::chrono::milliseconds(1),
        {rokae::RtSupportedFields::jointPos_m, rokae::RtSupportedFields::jointVel_m,
         rokae::RtSupportedFields::tau_m, rokae::RtSupportedFields::tcpPoseAbc_m});
      
      rt_controller->setServoJoint(servo_period_s_, servo_lookahead_s_, servo_kp_, ec);
      if (ec) {
        message = error_message("setServoJoint", ec);
        rollback_failed_rt_enable();
        set_status(message);
        return false;
      }
      // 关节伺服控制
      rt_controller->startMove(rokae::RtControllerMode::jointPosition);
      // 使能后先 hold 当前关节位姿；watchdog 等第一条外部命令到达后才开始计时。
      {
        std::lock_guard<std::mutex> lock(servo_mutex_);
        rt_controller_ = rt_controller;
        latest_servo_target_ = current_joint_positions_or_sdk();
        have_servo_target_ = finite_values(latest_servo_target_);
        received_servo_command_ = false;
        servo_enabled_ = true;
      }
      message = "servo move enabled";
      set_status(message);
      return true;
    } catch (const std::exception & e) {
      message = std::string("enable servo exception: ") + e.what();
      try {
        std::error_code cleanup_ec;
        robot_->setMotionControlMode(rokae::MotionControlMode::Idle, cleanup_ec);
        robot_->stopReceiveRobotState();
      } catch (const std::exception & cleanup_error) {
        RCLCPP_WARN(get_logger(), "rollback servo enable after exception failed: %s", cleanup_error.what());
      }
      set_status(message);
      return false;
    }
  }
  // 加锁并停止servoj
void XcoreRobotInterface::stop_servo_session()
  {
    std::lock_guard<std::mutex> lock(servo_mutex_);
    stop_servo_session_locked();
  }

void XcoreRobotInterface::stop_servo_session_locked()
  {
    // 结束帧标记当前目标完成，再停止 ServoJ/Move 并切回 Idle。
    if (rt_controller_) {
      if (have_servo_target_ && finite_values(latest_servo_target_)) {
        rokae::JointPosition finished(latest_servo_target_);
        finished.setFinished();
        rt_controller_->sendCommand(finished);
      }
      rt_controller_->stopServoJoint();
      rt_controller_->stopMove();
    }
    rt_controller_.reset();
    servo_enabled_ = false;
    have_servo_target_ = false;
    ik_failing_ = false;
    received_servo_command_ = false;
    clear_pose_reference();

    if (robot_) {
      std::error_code ec;
      robot_->setMotionControlMode(rokae::MotionControlMode::Idle, ec);
      if (ec) {
        RCLCPP_WARN(get_logger(), "setMotionControlMode(Idle) error: %s", ec.message().c_str());
      }
      robot_->stopReceiveRobotState();
    }
  }


}  // namespace xcore_gateway
