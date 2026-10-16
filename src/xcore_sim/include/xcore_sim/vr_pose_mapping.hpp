#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "tf2/LinearMath/Matrix3x3.h"

namespace xcore_sim
{

// 默认的关节名称
inline std::vector<std::string> default_joint_names()
{
  return {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
}

// 检查是否只包含有限值
inline bool is_finite_vector(const std::vector<double> & values)
{
  return std::all_of(values.begin(), values.end(), [](double value) {
    return std::isfinite(value);
  });
}

inline tf2::Matrix3x3 vr_rotation_matrix(const std::vector<double> & values)
{
  return tf2::Matrix3x3(
    values[0], values[1], values[2],
    values[3], values[4], values[5],
    values[6], values[7], values[8]);
}

}  // namespace xcore_sim
