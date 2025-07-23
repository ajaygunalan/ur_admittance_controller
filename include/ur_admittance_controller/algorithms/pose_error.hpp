#pragma once

#include <ur_admittance_controller/utilities/types.hpp>
#include <Eigen/Geometry>

namespace ur_admittance_controller {
namespace algorithms {

/**
 * @brief Compute pose error between current and desired poses
 * 
 * Uses the convention: error = current - desired
 * This gives the error from desired to current frame
 * 
 * @param X_current Current pose transform
 * @param X_desired Desired pose transform  
 * @return 6D pose error vector [position_error; orientation_error]
 */
inline Vector6d computePoseError(
    const Eigen::Isometry3d& X_current,
    const Eigen::Isometry3d& X_desired) {
  
  Vector6d error;
  
  // Position error: current - desired
  error.head<3>() = X_current.translation() - X_desired.translation();
  
  // Orientation error using quaternions
  Eigen::Quaterniond q_current(X_current.rotation());
  Eigen::Quaterniond q_desired(X_desired.rotation());
  
  // Ensure shortest path by checking dot product
  if (q_current.dot(q_desired) < 0.0) {
    q_desired.coeffs() *= -1.0;
  }
  
  // Compute error quaternion: q_error = q_current * q_desired^(-1)
  Eigen::Quaterniond q_error = q_current * q_desired.inverse();
  q_error.normalize();
  
  // Convert to axis-angle representation
  Eigen::AngleAxisd aa_error(q_error);
  error.tail<3>() = aa_error.axis() * aa_error.angle();
  
  return error;
}

/**
 * @brief Extract position and orientation errors separately
 * 
 * @param pose_error 6D pose error vector
 * @return Pair of (position_error_norm, orientation_error_norm)
 */
inline std::pair<double, double> getPoseErrorNorms(const Vector6d& pose_error) {
  return {pose_error.head<3>().norm(), pose_error.tail<3>().norm()};
}

} // namespace algorithms
} // namespace ur_admittance_controller