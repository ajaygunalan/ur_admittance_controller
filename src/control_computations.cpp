/**
 * @file control_computations.cpp
 * @brief Control algorithms for UR Admittance Controller
 * 
 * This file contains the mathematical and algorithmic implementations
 * of the admittance control law and related computations.
 */

#include "admittance_controller.hpp"
#include <memory>
#include <algorithm>
#include <cmath>

namespace ur_admittance_controller {

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
  
  // Compute admittance control law: M*a + D*v + K*e = F_ext
  Vector6d stiffness_force = stiffness_engagement_factor_ * (stiffness_ * pose_error_);
  desired_accel_ = mass_inverse_ * 
    (wrench_filtered_ - damping_ * desired_vel_ - stiffness_force);
  
  // Check for numerical issues
  if (desired_accel_.hasNaN()) {
    return false;
  }
  
  // Integrate acceleration to velocity
  desired_vel_ += desired_accel_ * period.seconds();
  
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
    
    // Queue logging for non-RT context instead of direct logging
    reportRTError(RTErrorType::CONTROL_ERROR);
    
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
      // Defer logging to non-RT context
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
      if (index >= state_interfaces_.size() || !state_interfaces_[index].get_optional()) {
        return false;
      }
      joint_positions_[i++] = state_interfaces_[index].get_optional().value();
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

void AdmittanceController::checkParameterUpdates()
{
  if (!params_.dynamic_parameters) return;
  
  auto new_params = param_listener_->get_params();
  
  // Use bit flags for efficient change detection
  uint8_t changes = 0;
  constexpr uint8_t MASS_CHANGED = 1;
  constexpr uint8_t STIFFNESS_CHANGED = 2;
  constexpr uint8_t DAMPING_CHANGED = 4;
  
  // Check for changes using memcmp for arrays
  if (std::memcmp(params_.admittance.mass.data(), new_params.admittance.mass.data(), 
                  sizeof(double) * 6) != 0) {
    changes |= MASS_CHANGED;
  }
  if (std::memcmp(params_.admittance.stiffness.data(), new_params.admittance.stiffness.data(), 
                  sizeof(double) * 6) != 0) {
    changes |= STIFFNESS_CHANGED;
  }
  if (std::memcmp(params_.admittance.damping_ratio.data(), new_params.admittance.damping_ratio.data(), 
                  sizeof(double) * 6) != 0) {
    changes |= DAMPING_CHANGED;
  }
  
  if (changes == 0) {
    params_ = new_params;  // Update non-control parameters
    return;
  }
  
  // Update parameters
  params_ = new_params;
  
  // Update matrices based on changes
  if (changes & MASS_CHANGED) {
    updateMassMatrix();
  }
  
  if (changes & STIFFNESS_CHANGED) {
    updateStiffnessMatrix();
    // Trigger gradual engagement
    stiffness_recently_changed_ = true;
    stiffness_engagement_factor_ = 0.0;
    RCLCPP_INFO(get_node()->get_logger(), "Starting gradual stiffness engagement");
  }
  
  if (changes & (STIFFNESS_CHANGED | DAMPING_CHANGED)) {
    updateDampingMatrix();
  }
}

void AdmittanceController::updateMassMatrix()
{
  for (size_t i = 0; i < 6; ++i) {
    mass_(i, i) = params_.admittance.mass[i];
  }
  mass_inverse_ = mass_.inverse();
  RCLCPP_INFO(get_node()->get_logger(), "Mass parameters updated");
}

void AdmittanceController::updateStiffnessMatrix()
{
  for (size_t i = 0; i < 6; ++i) {
    stiffness_(i, i) = params_.admittance.stiffness[i];
  }
  RCLCPP_INFO(get_node()->get_logger(), "Stiffness parameters updated");
}

void AdmittanceController::updateDampingMatrix()
{
  // Smooth damping calculation to avoid discontinuity when stiffness changes
  for (size_t i = 0; i < 6; ++i) {
    double stiffness_value = params_.admittance.stiffness[i];
    
    if (stiffness_value <= 0.0) {
      // Pure admittance mode - direct damping value
      // Scale by sqrt(mass) for consistent units [Ns/m or Nms/rad]
      damping_(i, i) = params_.admittance.damping_ratio[i] * 
        std::sqrt(params_.admittance.mass[i]);
    } 
    else if (stiffness_value >= STIFFNESS_BLEND_THRESHOLD) {
      // Full impedance mode - critical damping formula
      damping_(i, i) = 2.0 * params_.admittance.damping_ratio[i] * 
        std::sqrt(params_.admittance.mass[i] * stiffness_value);
    }
    else {
      // Smooth transition zone - blend between the two formulas
      double blend_factor = stiffness_value / STIFFNESS_BLEND_THRESHOLD;
      
      double admittance_damping = params_.admittance.damping_ratio[i] * 
        std::sqrt(params_.admittance.mass[i]);
        
      double impedance_damping = 2.0 * params_.admittance.damping_ratio[i] * 
        std::sqrt(params_.admittance.mass[i] * stiffness_value);
      
      // Smoothly blend between the two values
      damping_(i, i) = (1.0 - blend_factor) * admittance_damping + 
                      blend_factor * impedance_damping;
    }
  }
  RCLCPP_INFO(get_node()->get_logger(), "Damping parameters updated with smooth transitions");
}

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
    if (idx >= state_interfaces_.size() || !state_interfaces_[idx].get_optional()) {
      return false;
    }
    current_pos_[i] = state_interfaces_[idx].get_optional().value();
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
    joint_positions_[i] = std::clamp(
      joint_positions_[i], 
      joint_limits_[i].min_position, 
      joint_limits_[i].max_position);
    
    // Check for NaN values after operations
    if (std::isnan(joint_positions_[i])) {
      return false;
    }
  }
  
  return true;
}

} // namespace ur_admittance_controller
