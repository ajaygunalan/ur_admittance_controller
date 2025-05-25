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

Vector6d AdmittanceController::computeAdmittanceControl(const rclcpp::Duration& period)
{
  try {
    // Compute pose error for impedance control
    pose_error_ = computePoseError();
    
    // Safety check and stiffness management
    updateStiffnessEngagement(period);
    
    // Compute admittance control law: M*a + D*v + K*e = F_ext
    Vector6d stiffness_force = stiffness_engagement_factor_ * (stiffness_ * pose_error_);
    desired_accel_ = mass_inverse_ * 
      (wrench_filtered_ - damping_ * desired_vel_ - stiffness_force);
    
    // Integrate acceleration to velocity
    desired_vel_ += desired_accel_ * period.seconds();
    
    // Apply axis enables
    for (size_t i = 0; i < 6; ++i) {
      if (!params_.admittance.enabled_axes[i]) {
        desired_vel_(i) = 0.0;
      }
    }
    
    // Apply velocity limits
    applyCartesianVelocityLimits();
    
    // Check for drift and reset if needed
    if (desired_vel_.norm() < params_.admittance.drift_reset_threshold) {
      handleDriftReset();
      return Vector6d::Zero();
    }
    
    cart_twist_ = desired_vel_;
    return cart_twist_;
    
  } catch (const std::exception & e) {
    if (rclcpp::ok()) {
      RCLCPP_ERROR_SKIPFIRST_THROTTLE(rt_logger_, *get_node()->get_clock(), 1000,
        "Exception in computeAdmittanceControl: %s", e.what());
    }
    return Vector6d::Zero();
  }
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
  Eigen::Quaterniond q_desired(R_desired);
  
  // Ensure shortest path (handle quaternion double cover)
  if (q_current.dot(q_desired) < 0) {
    q_current.coeffs() = -q_current.coeffs();
  }
  
  // Calculate error quaternion
  Eigen::Quaterniond q_error = q_desired * q_current.inverse();
  
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

void AdmittanceController::updateStiffnessEngagement(const rclcpp::Duration& period)
{
  double position_error_norm = pose_error_.head<3>().norm();
  double orientation_error_norm = pose_error_.tail<3>().norm();
  
  // Check if error exceeds safety limits
  bool error_within_limits = true;
  if (stiffness_engagement_factor_ > STIFFNESS_ENGAGEMENT_THRESHOLD && 
      (position_error_norm > safe_startup_params_.max_position_error || 
       orientation_error_norm > safe_startup_params_.max_orientation_error)) {
    error_within_limits = false;
    
    RCLCPP_WARN(get_node()->get_logger(), 
      "SAFETY: Pose error exceeds limits (pos: %.3f > %.3f, orient: %.3f > %.3f). Reducing stiffness.",
      position_error_norm, safe_startup_params_.max_position_error,
      orientation_error_norm, safe_startup_params_.max_orientation_error);
    
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
      RCLCPP_INFO(get_node()->get_logger(), "Stiffness fully engaged");
    }
  }
  
  // Publish pose error for monitoring
  publishPoseError();
}

void AdmittanceController::publishPoseError()
{
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
}

void AdmittanceController::applyCartesianVelocityLimits()
{
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
}

void AdmittanceController::handleDriftReset()
{
  // Reset integration states
  desired_vel_.setZero();
  cart_twist_.setZero();
  
  // Reset to actual positions
  size_t i = 0;
  for (const auto& index : pos_state_indices_) {
    joint_positions_[i++] = state_interfaces_[index].get_optional().value();
  }
  
  // Update desired pose to current pose
  desired_pose_ = current_pose_;
  
  RCLCPP_DEBUG(get_node()->get_logger(), "Drift correction applied");
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
  try {
    // Read current joint positions
    for (size_t i = 0; i < params_.joints.size(); ++i) {
      current_pos_[i] = state_interfaces_[pos_state_indices_[i]].get_optional().value();
    }
    
    // Convert velocity to displacement
    for (size_t i = 0; i < 6; ++i) {
      cart_displacement_deltas_[i] = cart_vel(i) * period.seconds();
    }
    
    // Use kinematics to convert Cartesian to joint space
    if (!kinematics_ || !(*kinematics_)->convert_cartesian_deltas_to_joint_deltas(
          current_pos_, cart_displacement_deltas_, params_.tip_link, joint_deltas_)) {
      
      if (rclcpp::ok()) {
        RCLCPP_WARN_SKIPFIRST_THROTTLE(rt_logger_, *get_node()->get_clock(), 1000,
          "Kinematics conversion failed - zeroing motion");
      }
      return false;
    }
    
    // Update joint positions
    for (size_t i = 0; i < params_.joints.size() && i < joint_deltas_.size(); ++i) {
      joint_positions_[i] = current_pos_[i] + joint_deltas_[i];
    }
    
    return true;
    
  } catch (const std::exception & e) {
    if (rclcpp::ok()) {
      RCLCPP_ERROR_SKIPFIRST_THROTTLE(rt_logger_, *get_node()->get_clock(), 1000,
        "Exception in convertToJointSpace: %s", e.what());
    }
    return false;
  }
}

void AdmittanceController::applyJointLimits(const rclcpp::Duration& period)
{
  try {
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
    }
  } catch (const std::exception & e) {
    if (rclcpp::ok()) {
      RCLCPP_ERROR_SKIPFIRST_THROTTLE(rt_logger_, *get_node()->get_clock(), 1000,
        "Exception in applyJointLimits: %s", e.what());
    }
  }
}

} // namespace ur_admittance_controller
