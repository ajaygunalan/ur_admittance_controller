/**
 * @file realtime_computations.cpp
 * @brief Real-time control algorithms and computations for UR Admittance Controller
 * 
 * This file contains all real-time safe control computations including:
 * - Real-time control loop implementation
 * - Admittance control law computations
 * - Matrix update operations
 * - Parameter update handling
 * - Performance critical code
 */

#include "admittance_controller.hpp"
#include "admittance_constants.hpp"
#include "matrix_utilities.hpp"
#include <cstring>
#include <memory>
#include <algorithm>
#include <cmath>

namespace ur_admittance_controller {

// Use centralized constants
using namespace constants;

//=============================================================================
// Real-time Control Loop
//=============================================================================

controller_interface::return_type AdmittanceController::update_reference_from_subscribers(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Update parameters if dynamic
  if (params_.dynamic_parameters) {
    params_ = param_listener_->get_params();
  }
  
  return controller_interface::return_type::OK;
}

controller_interface::return_type AdmittanceController::update_and_write_commands(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Step 1: Check for parameter updates - can't fail
  checkParameterUpdates();
  
  // Step 2: Update sensor data and check for early exit conditions
  if (!updateSensorData()) {
    // Sensor data update failed - already logged in updateSensorData
    return controller_interface::return_type::OK; // Continue to next cycle
  }
  
  // Step 3: Update transforms
  if (!updateTransforms()) {
    // Transform update failed - already logged in updateTransforms
    return controller_interface::return_type::OK; // Continue to next cycle
  }
  
  // Step 4: Deadband check
  if (!checkDeadband()) {
    // Below deadband - no error, just skip calculation
    return controller_interface::return_type::OK;
  }
  
  // Step 5: Compute admittance control outputs
  Vector6d cmd_vel;
  if (!computeAdmittanceControl(period, cmd_vel)) {
    reportRTError(RTErrorType::CONTROL_ERROR);
    return safeStop();
  }
  
  // Step 6: Convert to joint space velocities
  if (!convertToJointSpace(cmd_vel, period)) {
    reportRTError(RTErrorType::KINEMATICS_ERROR);
    return safeStop();
  }
  
  // Step 7: Apply joint limits - failure is critical
  if (!applyJointLimits(period)) {
    reportRTError(RTErrorType::JOINT_LIMITS_ERROR);
    return safeStop(); // Changed to stop if limits can't be applied properly
  }
  
  // Step 8: Update controller references
  updateReferenceInterfaces();
  
  // Step 9: Publish monitoring data - non-critical
  publishMonitoringData();
  
  // Process any errors that occurred in a non-RT context
  processNonRTErrors();
  
  return controller_interface::return_type::OK; 
}

//=============================================================================
// Admittance Control Computation
//=============================================================================

bool AdmittanceController::computeAdmittanceControl(const rclcpp::Duration& period, Vector6d& cmd_vel_out)
{
  // Compute pose error for impedance control
  error_tip_base_ = computePoseError_tip_base();
  if (error_tip_base_.hasNaN()) {
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
  Vector6d stiffness_force1 = stiffness_engagement_factor_ * (stiffness_ * error_tip_base_);
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
  V_base_tip_base_ = desired_vel_;
  cmd_vel_out = V_base_tip_base_;
  return true;
}

Vector6d AdmittanceController::computePoseError_tip_base()
{
  Vector6d error = Vector6d::Zero();
  
  // Position error (tip position error in base frame)
  error.head<3>() = X_base_tip_desired_.translation() - X_base_tip_current_.translation();
  
  // Orientation error calculation with improved singularity handling
  Eigen::Matrix3d R_current = X_base_tip_current_.rotation();
  Eigen::Matrix3d R_desired = X_base_tip_desired_.rotation();
  
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
  
  double position_error_norm = error_tip_base_.head<3>().norm();
  double orientation_error_norm = error_tip_base_.tail<3>().norm();
  
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

//=============================================================================
// Parameter Update Handling
//=============================================================================

// REAL-TIME SAFE: This method runs in the RT thread and only reads from the buffer
void AdmittanceController::checkParameterUpdates()
{
  if (!params_.dynamic_parameters) return;

  // Signal that we need a parameter update in the non-RT thread
  parameter_update_needed_.store(true);
  
  // Read the latest parameters from the RT buffer
  const auto* latest_params = param_buffer_.readFromRT();
  
  // If no parameters are available yet, return
  if (!latest_params) return;
  
  // Safely copy the parameters (fast operation, no allocations)
  params_ = *latest_params;
}

// Non-real-time parameter update function
void AdmittanceController::prepareParameterUpdate()
{
  if (!parameter_update_needed_.load()) return;
  
  // Get latest parameters from the parameter server
  auto new_params = param_listener_->get_params();
  
  // No parameter validation - assume all parameters are valid
  
  // Proper floating-point comparison with epsilon tolerance
  // Check for changes using proper floating-point comparison
  bool mass_changed = false;
  bool stiffness_changed = false;
  bool damping_changed = false;
  
  for (size_t i = 0; i < 6; ++i) {
    if (!utils::areEqual(params_.admittance.mass[i], new_params.admittance.mass[i])) {
      mass_changed = true;
      break;
    }
  }
  
  for (size_t i = 0; i < 6; ++i) {
    if (!utils::areEqual(params_.admittance.stiffness[i], new_params.admittance.stiffness[i])) {
      stiffness_changed = true;
      break;
    }
  }
  
  for (size_t i = 0; i < 6; ++i) {
    if (!utils::areEqual(params_.admittance.damping_ratio[i], new_params.admittance.damping_ratio[i])) {
      damping_changed = true;
      break;
    }
  }
  
  // If no changes, just write the params to buffer and return
  if (!mass_changed && !stiffness_changed && !damping_changed) {
    param_buffer_.writeFromNonRT(new_params);
    parameter_update_needed_.store(false);
    return;
  }
  
  // Update matrices based on changes - this can safely log since we're non-RT
  if (mass_changed) {
    updateMassMatrix(new_params, true);
  }
  
  if (stiffness_changed) {
    updateStiffnessMatrix(new_params, true);
    // Trigger gradual engagement
    stiffness_recently_changed_ = true;
    stiffness_engagement_factor_ = 0.0;
    RCLCPP_INFO(get_node()->get_logger(), "Starting gradual stiffness engagement");
  }
  
  if (stiffness_changed || damping_changed) {
    updateDampingMatrix(new_params, true);
  }
  
  // Write updated parameters to the RT buffer
  param_buffer_.writeFromNonRT(new_params);
  parameter_update_needed_.store(false);
}

//=============================================================================
// Matrix Update Methods
//=============================================================================

// Update mass matrix - can be called from non-RT context only
void AdmittanceController::updateMassMatrix(const ur_admittance_controller::Params& params, bool log_changes)
{
  // Convert parameter vector to array for utility function
  std::array<double, 6> mass_array;
  for (size_t i = 0; i < 6; ++i) {
    mass_array[i] = params.admittance.mass[i];
    mass_(i, i) = mass_array[i];
  }
  
  // Use centralized mass inverse computation with stability checks
  mass_inverse_ = utils::computeMassInverse(mass_array);
  
  if (log_changes) {
    // Check condition number for logging
    double max_mass = mass_.diagonal().maxCoeff();
    double min_mass = mass_.diagonal().minCoeff();
    double condition_number = max_mass / min_mass;
    
    if (condition_number > MAX_CONDITION_NUMBER || min_mass <= 0.0) {
      RCLCPP_WARN(get_node()->get_logger(), 
        "Mass matrix regularized (condition number: %.2e, min mass: %.6f)", 
        condition_number, min_mass);
    } else {
      RCLCPP_INFO(get_node()->get_logger(), 
        "Mass parameters updated (condition number: %.2e)", condition_number);
    }
  }
}

// Legacy method - forwards to the new implementation
void AdmittanceController::updateMassMatrix()
{
  // Forward to the new implementation with current params
  updateMassMatrix(params_, false);
}

// Update stiffness matrix - can be called from non-RT context only
void AdmittanceController::updateStiffnessMatrix(const ur_admittance_controller::Params& params, bool log_changes)
{
  for (size_t i = 0; i < 6; ++i) {
    stiffness_(i, i) = params.admittance.stiffness[i];
  }
  
  if (log_changes) {
    RCLCPP_INFO(get_node()->get_logger(), "Stiffness parameters updated");
  }
}

// Legacy method - forwards to the new implementation
void AdmittanceController::updateStiffnessMatrix()
{
  // Forward to the new implementation with current params
  updateStiffnessMatrix(params_, false);
}

// Update damping matrix - can be called from non-RT context only
void AdmittanceController::updateDampingMatrix(const ur_admittance_controller::Params& params, bool log_changes)
{
  // Simplified damping calculation - direct critical damping without transition zones
  for (size_t i = 0; i < 6; ++i) {
    // Critical damping: D = 2 * sqrt(K * M) * damping_ratio
    double critical_damping = 2.0 * std::sqrt(params.admittance.stiffness[i] * params.admittance.mass[i]);
    damping_(i, i) = params.admittance.damping_ratio[i] * critical_damping;
  }
  
  if (log_changes) {
    RCLCPP_INFO(get_node()->get_logger(), 
      "Damping parameters updated with simplified critical damping formula");
  }
}

// Legacy method - forwards to the new implementation
void AdmittanceController::updateDampingMatrix()
{
  // Forward to the new implementation with current params
  updateDampingMatrix(params_, false);
}

//=============================================================================
// Velocity and Joint Space Operations
//=============================================================================

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

//=============================================================================
// Helper Methods
//=============================================================================

bool AdmittanceController::handleDriftReset()
{
  // Reset integration states
  desired_vel_.setZero();
  V_base_tip_base_.setZero();
  
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
  X_base_tip_desired_ = X_base_tip_current_;
  
  // Success - logging deferred to non-RT context
  return true;
}

bool AdmittanceController::publishPoseError()
{
  // This is a non-critical operation, so always return true
  // Even if publishing fails, the controller can continue
  if (rt_pose_error_pub_->trylock()) {
    auto& msg = rt_pose_error_pub_->msg_;
    msg.linear.x = error_tip_base_(0);
    msg.linear.y = error_tip_base_(1);
    msg.linear.z = error_tip_base_(2);
    msg.angular.x = error_tip_base_(3);
    msg.angular.y = error_tip_base_(4);
    msg.angular.z = error_tip_base_(5);
    rt_pose_error_pub_->unlockAndPublish();
  }
  return true;
}

void AdmittanceController::processNonRTErrors()
{
  // 1. Process RT logs first
  processRTLogs();
  
  // 2. Update transform caches in non-RT context if needed
  // This is safe to do here since this is explicitly outside the RT control path
  updateTransformCaches();
  
  // 3. Update parameters in non-RT context if needed
  // This handles all parameter updates and logging in non-RT context
  prepareParameterUpdate();
  
  // 4. Process any real-time errors in a non-real-time context
  RTErrorType error = last_rt_error_.exchange(RTErrorType::NONE);
  
  if (error != RTErrorType::NONE && rclcpp::ok()) {
    // Handle error without real-time violations
    switch (error) {
      case RTErrorType::UPDATE_ERROR:
        RCLCPP_ERROR(get_node()->get_logger(), "Real-time error occurred in update loop");
        break;
      case RTErrorType::SENSOR_ERROR:
        RCLCPP_ERROR(get_node()->get_logger(), "Real-time error in sensor data processing");
        break;
      case RTErrorType::TRANSFORM_ERROR:
        RCLCPP_ERROR(get_node()->get_logger(), "Transform lookup failed in real-time context");
        // Explicitly request transform update on error
        transform_update_needed_.store(true);
        break;
      case RTErrorType::CONTROL_ERROR:
        RCLCPP_ERROR(get_node()->get_logger(), "Control computation error in real-time context");
        break;
      case RTErrorType::KINEMATICS_ERROR:
        RCLCPP_ERROR(get_node()->get_logger(), "Kinematics error in real-time context");
        break;
      case RTErrorType::JOINT_LIMITS_ERROR:
        RCLCPP_ERROR(get_node()->get_logger(), "Joint limits error in real-time context");
        break;
      default:
        RCLCPP_ERROR(get_node()->get_logger(), "Unknown real-time error occurred");
        break;
    }
  }
}

void AdmittanceController::updateReferenceInterfaces()
{
  // First update our reference interfaces (exported to downstream controllers)
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    joint_position_references_[i] = joint_positions_[i];
  }
  
  // Now write to the claimed downstream controller reference interfaces
  // This is critical for proper controller chaining
  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    // Use the mapping to get the correct joint index for each command interface
    const size_t joint_idx = cmd_interface_to_joint_index_[i];
    
    // Write the joint position reference to the downstream controller interface
    if (!command_interfaces_[i].set_value(joint_position_references_[joint_idx])) {
      reportRTError(RTErrorType::CONTROL_ERROR);
    }
  }
}

void AdmittanceController::publishMonitoringData()
{
  publishCartesianVelocity();
}

controller_interface::return_type AdmittanceController::safeStop()
{
  try {
    // Zero all velocities
    V_base_tip_base_.setZero();
    desired_vel_.setZero();
    
    // Maintain current positions
    for (size_t i = 0; i < params_.joints.size(); ++i) {
      joint_position_references_[i] = state_interfaces_[pos_state_indices_[i]].get_optional().value();
    }
    
    return controller_interface::return_type::OK;
    
  } catch (const std::exception &) {
    // Real-time safe error reporting without formatting or memory allocation
    reportRTError(RTErrorType::CONTROL_ERROR);
    return controller_interface::return_type::ERROR;
  }
}

} // namespace ur_admittance_controller