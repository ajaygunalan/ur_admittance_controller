#pragma once

#include <ur_admittance_controller/utilities/types.hpp>
#include <algorithm>

namespace ur_admittance_controller {
namespace algorithms {

/**
 * @brief Apply workspace boundary limits to velocity
 * 
 * Prevents motion beyond workspace boundaries by zeroing velocity
 * components that would move the robot outside limits
 * 
 * @param velocity Current velocity command
 * @param current_position Current Cartesian position
 * @param workspace_limits Limits as [x_min, x_max, y_min, y_max, z_min, z_max]
 * @return Limited velocity
 */
inline Vector6d applyWorkspaceLimits(
    const Vector6d& velocity,
    const Eigen::Vector3d& current_position,
    const Vector6d& workspace_limits) {
  
  Vector6d limited_velocity = velocity;
  
  // Check each axis against boundaries
  for (size_t i = 0; i < 3; ++i) {
    const double min = workspace_limits[i * 2];
    const double max = workspace_limits[i * 2 + 1];
    
    // Only allow motion away from boundary
    if (current_position[i] <= min) {
      limited_velocity[i] = std::max(0.0, limited_velocity[i]);
    }
    if (current_position[i] >= max) {
      limited_velocity[i] = std::min(0.0, limited_velocity[i]);
    }
  }
  
  return limited_velocity;
}

/**
 * @brief Apply velocity magnitude limits
 * 
 * @param velocity Input velocity vector
 * @param max_linear_vel Maximum linear velocity (m/s)
 * @param max_angular_vel Maximum angular velocity (rad/s)
 * @return Limited velocity vector
 */
inline Vector6d limitVelocityMagnitude(
    const Vector6d& velocity,
    double max_linear_vel,
    double max_angular_vel) {
  
  Vector6d limited = velocity;
  
  // Limit linear velocity
  double linear_norm = velocity.head<3>().norm();
  if (linear_norm > max_linear_vel) {
    limited.head<3>() *= (max_linear_vel / linear_norm);
  }
  
  // Limit angular velocity
  double angular_norm = velocity.tail<3>().norm();
  if (angular_norm > max_angular_vel) {
    limited.tail<3>() *= (max_angular_vel / angular_norm);
  }
  
  return limited;
}

/**
 * @brief Check if position is within workspace
 * 
 * @param position Current position
 * @param workspace_limits Limits as [x_min, x_max, y_min, y_max, z_min, z_max]
 * @return True if within limits
 */
inline bool isWithinWorkspace(
    const Eigen::Vector3d& position,
    const Vector6d& workspace_limits) {
  
  for (size_t i = 0; i < 3; ++i) {
    if (position[i] < workspace_limits[i * 2] || 
        position[i] > workspace_limits[i * 2 + 1]) {
      return false;
    }
  }
  return true;
}

} // namespace algorithms
} // namespace ur_admittance_controller