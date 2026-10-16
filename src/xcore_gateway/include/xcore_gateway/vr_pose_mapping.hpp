#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"

namespace xcore_gateway
{

// AR5L 是 7 自由度机型，接口长度统一按 7 轴校验。
constexpr std::size_t kDof = 7;

inline std::vector<std::string> default_joint_names()
{
  return {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
}

inline bool finite_values(const std::vector<double> & values)
{
  return std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v); });
}

inline std::vector<double> array_to_vector(const std::array<double, kDof> & values)
{
  return std::vector<double>(values.begin(), values.end());
}

inline bool finite_pose(const geometry_msgs::msg::Pose & pose)
{
  return finite_values({
    pose.position.x, pose.position.y, pose.position.z,
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w});
}

inline tf2::Vector3 pose_position(const geometry_msgs::msg::Pose & pose)
{
  return tf2::Vector3(pose.position.x, pose.position.y, pose.position.z);
}

inline tf2::Quaternion pose_quaternion(const geometry_msgs::msg::Pose & pose)
{
  return tf2::Quaternion(
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
}

inline tf2::Matrix3x3 vr_rotation_matrix(const std::vector<double> & values)
{
  if (!finite_values(values)) {
    return tf2::Matrix3x3(
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0);
  }
  if (values.size() == 16) {
    return tf2::Matrix3x3(
      values[0], values[1], values[2],
      values[4], values[5], values[6],
      values[8], values[9], values[10]);
  }
  if (values.size() != 9) {
    return tf2::Matrix3x3(
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0);
  }
  return tf2::Matrix3x3(
    values[0], values[1], values[2],
    values[3], values[4], values[5],
    values[6], values[7], values[8]);
}

inline tf2::Matrix3x3 vr_position_matrix(const std::vector<double> & values)
{
  if (values.size() == 16 && finite_values(values)) {
    return tf2::Matrix3x3(
      values[3] * std::abs(values[0]), values[3] * std::abs(values[1]),
      values[3] * std::abs(values[2]),
      values[7] * std::abs(values[4]), values[7] * std::abs(values[5]),
      values[7] * std::abs(values[6]),
      values[11] * std::abs(values[8]), values[11] * std::abs(values[9]),
      values[11] * std::abs(values[10]));
  }
  return vr_rotation_matrix(values);
}

struct MappedPoseDelta
{
  tf2::Vector3 position;
  tf2::Quaternion rotation;
};

inline MappedPoseDelta mapped_pose_delta(
  const tf2::Vector3 & current_vr_position,
  const tf2::Vector3 & reference_vr_position,
  const tf2::Quaternion & current_vr_quat,
  const tf2::Quaternion & reference_vr_quat,
  const std::vector<double> & vr_rot,
  double position_scale,
  double rotation_scale)
{
  // 平移先按 position_scale 缩放，再由 vr_rot 从 VR 轴映射到机器人 base 轴。
  const tf2::Vector3 delta = (current_vr_position - reference_vr_position) * position_scale;

  // 旋转增量 q_current * inverse(q_ref)，再按 rotation_scale 做 SLERP 缩放。
  tf2::Quaternion q_delta = current_vr_quat * reference_vr_quat.inverse();
  q_delta.normalize();
  tf2::Quaternion scaled = tf2::Quaternion::getIdentity().slerp(q_delta, rotation_scale);
  scaled.normalize();

  // 旋转轴也使用和位置相同的 VR->base 轴映射：R_base = M * R_vr * M^-1。
  const tf2::Matrix3x3 position_map = vr_position_matrix(vr_rot);
  const tf2::Matrix3x3 rotation_map = vr_rotation_matrix(vr_rot);
  const tf2::Matrix3x3 mapped_matrix =
    rotation_map * tf2::Matrix3x3(scaled) * rotation_map.transpose();

  tf2::Quaternion mapped;
  mapped_matrix.getRotation(mapped);
  mapped.normalize();
  return {position_map * delta, mapped};
}

}  // namespace xcore_gateway
