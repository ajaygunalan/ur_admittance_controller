#pragma once

#include <ur_admittance_controller/utilities/types.hpp>
#include <ur_admittance_controller/utilities/error.hpp>

namespace ur_admittance_controller {
namespace algorithms {

/**
 * @brief Core admittance control algorithm
 * 
 * Implements the admittance equation: M*a + D*v + K*x = F_ext
 * Solving for acceleration: a = M^(-1) * (F_ext - D*v - K*x)
 * 
 * @param external_wrench Scaled external wrench (F_ext)
 * @param velocity_commanded Current commanded velocity
 * @param pose_error Current pose error (x)
 * @param mass_inverse Diagonal elements of M^(-1)
 * @param damping Diagonal elements of D
 * @param stiffness Diagonal elements of K
 * @return Computed acceleration vector
 */
inline Vector6d computeAdmittanceAcceleration(
    const Vector6d& external_wrench,
    const Vector6d& velocity_commanded,
    const Vector6d& pose_error,
    const Vector6d& mass_inverse,
    const Vector6d& damping,
    const Vector6d& stiffness) {
  
  // Core admittance equation: a = M^(-1) * (F_ext - D*v - K*x)
  return mass_inverse.array() *
      (external_wrench.array() - damping.array() * velocity_commanded.array() -
       stiffness.array() * pose_error.array());
}

/**
 * @brief Apply safety limits to acceleration
 * 
 * @param acceleration Input acceleration vector
 * @param max_linear_acc Maximum linear acceleration (m/s²)
 * @param max_angular_acc Maximum angular acceleration (rad/s²)
 * @return Limited acceleration vector
 */
inline Vector6d limitAcceleration(
    const Vector6d& acceleration,
    double max_linear_acc,
    double max_angular_acc) {
  
  Vector6d limited = acceleration;
  
  // Limit linear acceleration
  double linear_norm = acceleration.head<3>().norm();
  if (linear_norm > max_linear_acc) {
    limited.head<3>() *= (max_linear_acc / linear_norm);
  }
  
  // Limit angular acceleration  
  double angular_norm = acceleration.tail<3>().norm();
  if (angular_norm > max_angular_acc) {
    limited.tail<3>() *= (max_angular_acc / angular_norm);
  }
  
  return limited;
}

/**
 * @brief Integrate acceleration to velocity using Euler method
 * 
 * @param current_velocity Current velocity
 * @param acceleration Acceleration to integrate
 * @param dt Time step
 * @return Updated velocity
 */
inline Vector6d integrateVelocity(
    const Vector6d& current_velocity,
    const Vector6d& acceleration,
    double dt) {
  return current_velocity + acceleration * dt;
}

} // namespace algorithms
} // namespace ur_admittance_controller