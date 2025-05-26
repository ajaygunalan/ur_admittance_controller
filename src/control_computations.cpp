/**
 * @file control_computations.cpp
 * @brief Control algorithms for UR Admittance Controller
 * 
 * This file contains the mathematical and algorithmic implementations
 * of the admittance control law and related computations.
 */

#include "admittance_controller.hpp"
#include "admittance_constants.hpp"
#include <memory>
#include <algorithm>
#include <cmath>

namespace ur_admittance_controller {

// Use centralized constants
using namespace constants;

bool AdmittanceController::computeAdmittanceControl(const rclcpp::Duration& period, Vector6d& cmd_vel_out)
{
  // Compute pose error for impedance control
  pose_error_ = computePoseError();
  if (pose_error_.hasNaN()) {
    return false;
  }
  
  // Safety check and stiffness management
  if (!updateStiffnessEngagement(period)) {
    return false;
  }
  
  // RK4 integration for numerical stability
  double dt = period.seconds();
  if (dt <= 0.0 || dt > 0.1) {  // Sanity check on time step
    return false;
  }
  
  // Current state
  Vector6d v0 = desired_vel_;
  
  // k1 = f(t, v0)
  Vector6d stiffness_force1 = stiffness_engagement_factor_ * (stiffness_ * pose_error_);
  Vector6d k1 = mass_inverse_ * (wrench_filtered_ - damping_ * v0 - stiffness_force1);
  
  if (k1.hasNaN()) {
    return false;
  }
  
  // k2 = f(t + dt/2, v0 + k1*dt/2)
  Vector6d v1 = v0 + k1 * (dt / 2.0);
  Vector6d k2 = mass_inverse_ * (wrench_filtered_ - damping_ * v1 - stiffness_force1);
  
  if (k2.hasNaN()) {
    return false;
  }
  
  // k3 = f(t + dt/2, v0 + k2*dt/2)
  Vector6d v2 = v0 + k2 * (dt / 2.0);
  Vector6d k3 = mass_inverse_ * (wrench_filtered_ - damping_ * v2 - stiffness_force1);
  
  if (k3.hasNaN()) {
    return false;
  }
  
  // k4 = f(t + dt, v0 + k3*dt)
  Vector6d v3 = v0 + k3 * dt;
  Vector6d k4 = mass_inverse_ * (wrench_filtered_ - damping_ * v3 - stiffness_force1);
  
  if (k4.hasNaN()) {
    return false;
  }
  
  // RK4 integration: v_new = v0 + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
  desired_vel_ = v0 + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
  
  // Final NaN check after integration
  if (desired_vel_.hasNaN()) {
    return false;
  }
  
  // Apply axis enables
  for (size_t i = 0; i < 6; ++i) {
    if (!params_.admittance.enabled_axes[i]) {
      desired_vel_(i) = 0.0;
    }
  }
  
  // Apply velocity limits
  if (!applyCartesianVelocityLimits()) {
    return false;
  }
  
  // Check for drift and reset if needed
  if (desired_vel_.norm() < params_.admittance.drift_reset_threshold) {
    if (!handleDriftReset()) {
      return false;
    }
    cmd_vel_out.setZero();
    return true;
  }
  
  // Set output and return success
  cart_twist_ = desired_vel_;
  cmd_vel_out = cart_twist_;
  return true;
}

Vector6d AdmittanceController::computePoseError()
{
  Vector6d error = Vector6d::Zero();
  
  // Position error
  error.head<3>() = desired_pose_.translation() - current_pose_.translation();
  
  // Orientation error calculation with improved singularity handling
  Eigen::Matrix3d R_current = current_pose_.rotation();
  Eigen::Matrix3d R_desired = desired_pose_.rotation();
  
  // Convert to quaternions for more stable interpolation
  Eigen::Quaterniond q_current(R_current);
  q_current.normalize();  // Ensure unit quaternion
  
  Eigen::Quaterniond q_desired(R_desired);
  q_desired.normalize();  // Ensure unit quaternion
  
  // Ensure shortest path (handle quaternion double cover)
  if (q_current.dot(q_desired) < 0) {
    q_current.coeffs() = -q_current.coeffs();
  }
  
  // Calculate error quaternion
  Eigen::Quaterniond q_error = q_desired * q_current.inverse();
  q_error.normalize();  // Ensure unit quaternion for error
  
  // Convert to scaled axis-angle representation
  Eigen::AngleAxisd aa_error(q_error);
  
  // Handle small angles to avoid division by zero
  if (aa_error.angle() < QUATERNION_EPSILON) {
    error.tail<3>().setZero();
  } else {
    // Scale axis by angle for smooth control
    error.tail<3>() = aa_error.axis() * aa_error.angle();
    
    // Limit maximum orientation error to avoid singularities
    double error_norm = error.tail<3>().norm();
    if (error_norm > MAX_ORIENTATION_ERROR) {
      error.tail<3>() *= MAX_ORIENTATION_ERROR / error_norm;
    }
  }
  
  return error;
}

bool AdmittanceController::updateStiffnessEngagement(const rclcpp::Duration& period)
{
  // Check for invalid inputs
  if (period.seconds() <= 0.0) {
    return false;
  }
  
  double position_error_norm = pose_error_.head<3>().norm();
  double orientation_error_norm = pose_error_.tail<3>().norm();
  
  // Check if error exceeds safety limits
  bool error_within_limits = true;
  if (stiffness_engagement_factor_ > STIFFNESS_ENGAGEMENT_THRESHOLD && 
      (position_error_norm > safe_startup_params_.max_position_error || 
       orientation_error_norm > safe_startup_params_.max_orientation_error)) {
    error_within_limits = false;
    
    // Use RT-safe logging with predefined message type
    rtLogWarn(RTLogType::WARN_POSE_ERROR_LIMIT);
    
    // Reduce stiffness immediately
    stiffness_recently_changed_ = true;
    stiffness_engagement_factor_ = STIFFNESS_REDUCTION_FACTOR;
  }
  
  // Gradually engage stiffness if recently changed
  if (stiffness_recently_changed_ && error_within_limits) {
    stiffness_engagement_factor_ += period.seconds() / safe_startup_params_.stiffness_ramp_time;
    if (stiffness_engagement_factor_ >= 1.0) {
      stiffness_engagement_factor_ = 1.0;
      stiffness_recently_changed_ = false;
      // Use RT-safe logging with predefined message type
      rtLogInfo(RTLogType::INFO_STIFFNESS_ENGAGED);
    }
  }
  
  // Publish pose error for monitoring - deferred to non-RT context
  return true;
}

bool AdmittanceController::publishPoseError()
{
  // This is a non-critical operation, so always return true
  // Even if publishing fails, the controller can continue
  if (rt_pose_error_pub_->trylock()) {
    auto& msg = rt_pose_error_pub_->msg_;
    msg.linear.x = pose_error_(0);
    msg.linear.y = pose_error_(1);
    msg.linear.z = pose_error_(2);
    msg.angular.x = pose_error_(3);
    msg.angular.y = pose_error_(4);
    msg.angular.z = pose_error_(5);
    rt_pose_error_pub_->unlockAndPublish();
  }
  return true;
}

bool AdmittanceController::applyCartesianVelocityLimits()
{
  // Check for NaN values
  if (desired_vel_.hasNaN()) {
    return false;
  }
  
  // Separate linear and angular components
  double linear_velocity_magnitude = desired_vel_.head<3>().norm();
  double angular_velocity_magnitude = desired_vel_.tail<3>().norm();
  
  // Clamp linear velocity
  if (linear_velocity_magnitude > params_.max_linear_velocity) {
    desired_vel_.head<3>() *= (params_.max_linear_velocity / linear_velocity_magnitude);
  }
  
  // Clamp angular velocity
  if (angular_velocity_magnitude > params_.max_angular_velocity) {
    desired_vel_.tail<3>() *= (params_.max_angular_velocity / angular_velocity_magnitude);
  }
  
  // Verify the result doesn't have NaN values after operations
  return !desired_vel_.hasNaN();
}

bool AdmittanceController::handleDriftReset()
{
  // Reset integration states
  desired_vel_.setZero();
  cart_twist_.setZero();
  
  // Reset to actual positions (with error checking)
  try {
    size_t i = 0;
    for (const auto& index : pos_state_indices_) {
      if (index >= state_interfaces_.size()) {
        return false;
      }
      joint_positions_[i++] = state_interfaces_[index].get_value();
    }
  } catch (...) {
    // Catch and report any errors without propagating exceptions
    reportRTError(RTErrorType::CONTROL_ERROR);
    return false;
  }
  
  // Update desired pose to current pose
  desired_pose_ = current_pose_;
  
  // Success - logging deferred to non-RT context
  return true;
}

// Parameter update method moved to realtime_control_core.cpp
// to maintain proper RT/non-RT separation. This method is no longer needed
// as the RT-safe checkParameterUpdates() in realtime_control_core.cpp handles
// parameter updates correctly with proper atomic operations.

// Matrix update methods removed - using centralized implementations from realtime_control_core.cpp

bool AdmittanceController::convertToJointSpace(
    const Vector6d& cart_vel, 
    const rclcpp::Duration& period)
{
  // Check for invalid inputs
  if (period.seconds() <= 0.0 || cart_vel.hasNaN()) {
    return false;
  }
  
  // Boundary checks
  if (params_.joints.size() == 0 || 
      pos_state_indices_.size() < params_.joints.size() ||
      current_pos_.size() < params_.joints.size()) {
    return false;
  }
  
  // Read current joint positions - without using exceptions
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    size_t idx = pos_state_indices_[i];
    if (idx >= state_interfaces_.size()) {
      return false;
    }
    current_pos_[i] = state_interfaces_[idx].get_value();
    if (std::isnan(current_pos_[i])) {
      return false;
    }
  }
  
  // Convert velocity to displacement
  for (size_t i = 0; i < 6 && i < cart_displacement_deltas_.size(); ++i) {
    cart_displacement_deltas_[i] = cart_vel(i) * period.seconds();
  }
  
  // Use kinematics to convert Cartesian to joint space
  if (!kinematics_ || !(*kinematics_)->convert_cartesian_deltas_to_joint_deltas(
        current_pos_, cart_displacement_deltas_, params_.tip_link, joint_deltas_)) {
    reportRTError(RTErrorType::KINEMATICS_ERROR);
    return false;
  }
  
  // Update joint positions
  for (size_t i = 0; i < params_.joints.size() && i < joint_deltas_.size(); ++i) {
    joint_positions_[i] = current_pos_[i] + joint_deltas_[i];
    if (std::isnan(joint_positions_[i])) {
      return false;
    }
  }
  
  return true;
}

bool AdmittanceController::applyJointLimits(const rclcpp::Duration& period)
{
  // Check for invalid inputs
  if (period.seconds() <= 0.0) {
    return false;
  }
  
  // Boundary checks
  if (params_.joints.size() == 0 || joint_deltas_.size() == 0 ||
      params_.joints.size() > joint_positions_.size() || 
      joint_deltas_.size() > joint_positions_.size() ||
      joint_limits_.size() < params_.joints.size()) {
    return false;
  }
  
  // Apply limits without exceptions
  for (size_t i = 0; i < params_.joints.size() && i < joint_deltas_.size(); ++i) {
    // Apply velocity limits first
    double velocity = joint_deltas_[i] / period.seconds();
    if (std::abs(velocity) > joint_limits_[i].max_velocity) {
      double scale = joint_limits_[i].max_velocity / std::abs(velocity);
      joint_positions_[i] = current_pos_[i] + joint_deltas_[i] * scale;
    }
    
    // Apply position limits (hard constraint)
    joint_positions_[i] = std::max(joint_limits_[i].min_position, 
                          std::min(joint_positions_[i], joint_limits_[i].max_position));
    
    // Check for NaN values after operations
    if (std::isnan(joint_positions_[i])) {
      return false;
    }
  }
  
  return true;
}

} // namespace ur_admittance_controller
