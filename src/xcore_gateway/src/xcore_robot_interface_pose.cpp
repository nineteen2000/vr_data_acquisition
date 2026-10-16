// VR MODE_POSE 到机器人 TCP/IK 目标的映射、限速和 IK 失败处理。

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
bool XcoreRobotInterface::lock_pose_reference(const geometry_msgs::msg::Pose & vr_pose, bool reset_lock_time)
  {
    // 真机参考 TCP 必须来自 SDK 实时状态，避免上游 ROS 话题延迟影响相对跟随基准。
    std::string message;
    if (!require_robot(message)) {
      return false;
    }
    if (!finite_pose(vr_pose)) {
      set_status("cannot lock pose reference: non-finite VR pose");
      return false;
    }

    tf2::Quaternion vr_quat = pose_quaternion(vr_pose);
    if (vr_quat.length2() <= 0.0) {
      set_status("cannot lock pose reference: invalid VR quaternion");
      return false;
    }
    vr_quat.normalize();

    std::error_code ec;
    const auto cart = robot_->cartPosture(rokae::CoordinateType::endInRef, ec);
    if (ec) {
      set_status(error_message("cartPosture for ArmServoCommand MODE_POSE", ec));
      return false;
    }

    tf2::Quaternion tcp_quat;
    tcp_quat.setRPY(cart.rpy[0], cart.rpy[1], cart.rpy[2]);
    if (tcp_quat.length2() <= 0.0) {
      set_status("cannot lock pose reference: invalid TCP quaternion");
      return false;
    }
    tcp_quat.normalize();

    reference_vr_position_ = pose_position(vr_pose);
    reference_vr_quat_ = vr_quat;
    reference_tcp_position_ = tf2::Vector3(cart.trans[0], cart.trans[1], cart.trans[2]);
    reference_tcp_quat_ = tcp_quat;
    reference_tcp_elbow_ = cart.elbow;
    filtered_target_position_ = reference_tcp_position_;
    filtered_target_quat_ = reference_tcp_quat_;
    filtered_cartesian_target_valid_ = true;
    last_cartesian_filter_time_ = get_clock()->now();
    if (reset_lock_time) {
      pose_reference_lock_time_ = get_clock()->now();
    }
    pose_reference_valid_ = true;
    set_status("locked ArmServoCommand MODE_POSE VR and TCP reference");
    return true;
  }
  // 将 VR 位姿变化映射为机器人 TCP 位姿，再通过 IK 转成关节目标
void XcoreRobotInterface::handle_pose_command(const geometry_msgs::msg::Pose & pose)
  {
    // MODE_POSE 语义统一为 VR 原始位姿：这里按米处理，不使用 arm_servo_input_unit_ 的 mm 转换。
    if (!finite_pose(pose)) {
      set_status("rejected ArmServoCommand MODE_POSE: non-finite pose");
      return;
    }
    if (!pose_reference_valid_ && !lock_pose_reference(pose)) {
      return;
    }
    if (pose_reference_settle_s_ > 0.0 &&
      (get_clock()->now() - pose_reference_lock_time_).seconds() < pose_reference_settle_s_)
    {
      // Trigger press can perturb PICO tracking by centimeters. During the
      // settle window, keep re-locking the reference and hold the current joints
      // so the arm does not react to that transient.
      if (!lock_pose_reference(pose, false)) {
        return;
      }
      accept_servo_target(current_joint_positions_or_sdk(), "ArmServoCommand MODE_POSE reference settle");
      return;
    }

    tf2::Quaternion current_vr_quat = pose_quaternion(pose);
    if (current_vr_quat.length2() <= 0.0) {
      set_status("rejected ArmServoCommand MODE_POSE: zero quaternion");
      return;
    }
    current_vr_quat.normalize();

    const MappedPoseDelta mapped_delta = mapped_pose_delta(
      pose_position(pose),
      reference_vr_position_,
      current_vr_quat,
      reference_vr_quat_,
      vr_rot_,
      position_scale_,
      rotation_scale_);
    const tf2::Vector3 raw_target_position = reference_tcp_position_ + mapped_delta.position;
    bool workspace_limited = false;
    // work_space限制
    const tf2::Vector3 target_position =
      limit_workspace_position(raw_target_position, workspace_limited);

    tf2::Quaternion target_quat = mapped_delta.rotation * reference_tcp_quat_;
    target_quat.normalize();
    tf2::Vector3 limited_target_position;
    tf2::Quaternion limited_target_quat;
    limit_cartesian_target(target_position, target_quat, limited_target_position, limited_target_quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(limited_target_quat).getRPY(roll, pitch, yaw);

    std::string message;
    if (!require_robot(message)) {
      return;
    }
    std::error_code ec;
    // 先用 SDK calcIk 保持原有失败语义，再用 all IK 从上一目标附近选解，减少冗余轴跳解。
    auto model = robot_->model();
    const auto cartesian = to_cartesian(
      {limited_target_position.x(), limited_target_position.y(), limited_target_position.z(), roll, pitch, yaw,
       reference_tcp_elbow_});
    const auto joints = model.calcIk(cartesian, ec);
    if (ec) {
      handle_ik_failure(ec, "calcIk for ArmServoCommand MODE_POSE");
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
    accept_servo_target(
      target,
      std::string("ArmServoCommand MODE_POSE") +
      (workspace_limited ? " [workspace-limited]" : ""));
  }
  // 是否在设定工作空间内
bool XcoreRobotInterface::within_workspace(const tf2::Vector3 & position) const
  {
    // 空 workspace 表示只启用关节限位；实机校准后可打开 XYZ 保守限幅。
    if (workspace_min_.size() != 3 || workspace_max_.size() != 3) {
      return true;
    }
    return position.x() >= workspace_min_[0] && position.x() <= workspace_max_[0] &&
      position.y() >= workspace_min_[1] && position.y() <= workspace_max_[1] &&
      position.z() >= workspace_min_[2] && position.z() <= workspace_max_[2];
  }
  // 限定在设定xyz工作空间内
tf2::Vector3 XcoreRobotInterface::limit_workspace_position(const tf2::Vector3 & position, bool & limited) const
  {
    limited = false;
    if (workspace_min_.size() != 3 || workspace_max_.size() != 3) {
      return position;
    }
    tf2::Vector3 result = position;
    const double values[3] = {position.x(), position.y(), position.z()};
    double limited_values[3] = {values[0], values[1], values[2]};
    for (int i = 0; i < 3; ++i) {
      const double clamped = std::clamp(values[i], workspace_min_[i], workspace_max_[i]);
      if (clamped != values[i]) {
        limited = true;
        limited_values[i] = clamped;
      }
    }
    result.setValue(limited_values[0], limited_values[1], limited_values[2]);
    return result;
  }
  // 清空VR-TCP参考位姿
void XcoreRobotInterface::clear_pose_reference()
  {
    pose_reference_valid_ = false;
    reference_vr_position_ = tf2::Vector3();
    reference_vr_quat_ = tf2::Quaternion();
    reference_tcp_position_ = tf2::Vector3();
    reference_tcp_quat_ = tf2::Quaternion();
    reference_tcp_elbow_ = 0.0;
    reset_cartesian_target_filter();
  }
  // 清除ik失败状态
void XcoreRobotInterface::reset_ik_fail_state()
  {
    ik_failing_ = false;
  }
void XcoreRobotInterface::reset_cartesian_target_filter()
  {
    filtered_cartesian_target_valid_ = false;
    filtered_target_position_ = tf2::Vector3();
    filtered_target_quat_ = tf2::Quaternion::getIdentity();
    last_cartesian_filter_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }
void XcoreRobotInterface::limit_cartesian_target(
    const tf2::Vector3 & desired_position,
    const tf2::Quaternion & desired_quat,
    tf2::Vector3 & limited_position,
    tf2::Quaternion & limited_quat)
  {
    const auto now = get_clock()->now();
    tf2::Quaternion normalized_desired_quat = desired_quat;
    normalized_desired_quat.normalize();
    if (!filtered_cartesian_target_valid_) {
      filtered_target_position_ = desired_position;
      filtered_target_quat_ = normalized_desired_quat;
      last_cartesian_filter_time_ = now;
      filtered_cartesian_target_valid_ = true;
      limited_position = filtered_target_position_;
      limited_quat = filtered_target_quat_;
      return;
    }

    double dt = (now - last_cartesian_filter_time_).seconds();
    if (!std::isfinite(dt) || dt <= 0.0) {
      dt = servo_period_s_;
    }

    const tf2::Vector3 position_delta = desired_position - filtered_target_position_;
    const double distance = position_delta.length();
    if (pose_max_linear_speed_mps_ > 0.0) {
      const double max_linear_step = pose_max_linear_speed_mps_ * dt;
      if (distance > max_linear_step && distance > 0.0) {
        filtered_target_position_ += position_delta * (max_linear_step / distance);
      } else {
        filtered_target_position_ = desired_position;
      }
    } else {
      filtered_target_position_ = desired_position;
    }

    tf2::Quaternion current_quat = filtered_target_quat_;
    current_quat.normalize();
    if (current_quat.dot(normalized_desired_quat) < 0.0) {
      normalized_desired_quat = tf2::Quaternion(
        -normalized_desired_quat.x(), -normalized_desired_quat.y(),
        -normalized_desired_quat.z(), -normalized_desired_quat.w());
    }
    if (pose_max_angular_speed_radps_ > 0.0) {
      const double angle = current_quat.angleShortestPath(normalized_desired_quat);
      const double max_angular_step = pose_max_angular_speed_radps_ * dt;
      if (angle > max_angular_step && angle > 0.0) {
        filtered_target_quat_ = current_quat.slerp(normalized_desired_quat, max_angular_step / angle);
      } else {
        filtered_target_quat_ = normalized_desired_quat;
      }
    } else {
      filtered_target_quat_ = normalized_desired_quat;
    }
    filtered_target_quat_.normalize();
    last_cartesian_filter_time_ = now;
    limited_position = filtered_target_position_;
    limited_quat = filtered_target_quat_;
  }
  // 根据 IK 错误类型处理：保持上一解、报警或超时停机
void XcoreRobotInterface::handle_ik_failure(const std::error_code & ec, const std::string & source)
  {
    const auto now = get_clock()->now();
    if (!ik_failing_) {
      first_ik_fail_time_ = now;
      ik_failing_ = true;
    }
    const double elapsed = (now - first_ik_fail_time_).seconds();

    std::string semantic;
    switch (ec.value()) {
      case -50102:
        semantic = "奇异点(singularity)";
        break;
      case -50114:
        semantic = "超限位(joint limit)";
        break;
      case -50519:
        semantic = "超范围(out of range)";
        break;
      case -50002:
        semantic = "其它逆解错误";
        break;
      default:
        semantic = "未知错误";
        break;
    }

    std::ostringstream status;
    status << source << " failed: code=" << ec.value() << ", " << semantic;
    if (ik_failing_ && ik_fail_stop_timeout_s_ > 0.0 && elapsed >= ik_fail_stop_timeout_s_) {
      std::ostringstream stop_status;
      stop_status << source << " IK 持续失败 " << std::fixed << std::setprecision(2) << elapsed
                  << "s（阈值 " << ik_fail_stop_timeout_s_ << "s，code=" << ec.value() << " "
                  << semantic << "），已主动停机，请重新使能。";
      const std::string stop_message = stop_status.str();
      set_status(stop_message);
      RCLCPP_ERROR(get_logger(), "%s", stop_message.c_str());
      stop_servo_session();
      return;
    }

    bool hold_previous = false;
    if (!ik_hold_on_fail_) {
      status << "; IK失败兜底=关闭，丢帧";
    } else if (!have_servo_target_ || !finite_values(latest_servo_target_)) {
      status << "; IK失败兜底=开启，但无上次有效关节解可保持，丢帧";
    } else {
      status << "; IK失败兜底=开启，保持上次有效关节解";
      hold_previous = true;
    }

    const std::string status_message = status.str();
    set_status(status_message);
    if (ec.value() == -50114 || ec.value() == -50519) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "%s", status_message.c_str());
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "%s", status_message.c_str());
    }

    if (!hold_previous) {
      return;
    }

    std::vector<double> last_target = latest_servo_target_;
    accept_servo_target(last_target, source + " [IK失败兜底:保持上次解]");
  }
  // 接收外部伺服目标，做步长限幅、关节限位，然后缓存

}  // namespace xcore_gateway
