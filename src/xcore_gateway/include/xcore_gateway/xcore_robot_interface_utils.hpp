#pragma once

#include <algorithm>
#include <cstddef>
#include <limits>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

#include "xcore_gateway/vr_pose_mapping.hpp"

namespace xcore_gateway
{

inline std::string error_message(const std::string & action, const std::error_code & ec)
{
  std::ostringstream oss;
  oss << action << " failed: " << ec.message() << " (code=" << ec.value() << ")";
  return oss.str();
}

inline double squared_joint_distance(const std::vector<double> & lhs, const std::vector<double> & rhs)
{
  double distance = 0.0;
  const std::size_t count = std::min(lhs.size(), rhs.size());
  for (std::size_t i = 0; i < count; ++i) {
    const double delta = lhs[i] - rhs[i];
    distance += delta * delta;
  }
  return distance;
}

inline bool solution_within_limits(
  const std::vector<double> & solution,
  const std::vector<double> & lower,
  const std::vector<double> & upper)
{
  if (lower.size() != kDof || upper.size() != kDof) {
    return true;
  }
  for (std::size_t i = 0; i < kDof; ++i) {
    constexpr double tolerance = 1e-6;
    if (solution[i] < lower[i] - tolerance || solution[i] > upper[i] + tolerance) {
      return false;
    }
  }
  return true;
}

inline std::vector<double> select_closest_ik_solution(
  const std::vector<std::vector<double>> & solutions,
  const std::vector<double> & reference,
  const std::vector<double> & lower,
  const std::vector<double> & upper)
{
  std::vector<double> best;
  double best_distance = std::numeric_limits<double>::infinity();
  for (const auto & solution : solutions) {
    if (solution.size() != kDof || !finite_values(solution)) {
      continue;
    }
    if (!solution_within_limits(solution, lower, upper)) {
      continue;
    }
    const double distance = squared_joint_distance(solution, reference);
    if (distance < best_distance) {
      best = solution;
      best_distance = distance;
    }
  }
  return best;
}

}  // namespace xcore_gateway
