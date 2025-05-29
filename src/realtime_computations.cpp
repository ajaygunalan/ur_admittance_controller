/**
 * @file realtime_computations.cpp
 * @brief Real-time control loop implementation for admittance control
 *
 * This file contains the real-time safe implementations of the admittance
 * control algorithm, including force/torque processing, admittance computation,
 * and joint space conversion.
 */

#include "admittance_controller.hpp"
#include "admittance_constants.hpp"
#include "matrix_utilities.hpp"
#include <cstring>
#include <memory>
#include <algorithm>
#include <cmath>

namespace ur_admittance_controller {

using namespace constants;

/**
 * @brief Main real-time update loop
 * 
 * This method is called at the control rate and performs:
 * 1. Parameter updates
 * 2. Sensor data acquisition
 * 3. Transform updates
 * 4. Deadband checking
 * 5. Admittance control computation
 * 6. Joint space conversion
 * 7. Joint limit enforcement
 * 8. Reference interface updates
 * 9. Monitoring data publication
 */
controller_interface::return_type AdmittanceController::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Check for parameter updates from ROS
  checkParameterUpdates();
  
  // Update force/torque sensor data
  if (!updateSensorData()) {
    return controller_interface::return_type::OK;
  }
  
  // Update transform caches if needed
  if (!updateTransforms()) {
    return controller_interface::return_type::OK;
  }
  
  // Check if forces are within deadband
  if (!checkDeadband()) {
    return controller_interface::return_type::OK;
  }
  
  // Compute admittance control law
  Vector6d cmd_vel;
  if (!computeAdmittanceControl(period, cmd_vel)) {
    reportRTError(RTErrorType::CONTROL_ERROR);
    return safeStop();
  }
  
  // Convert Cartesian velocity to joint space
  if (!convertToJointSpace(cmd_vel, period)) {
    reportRTError(RTErrorType::KINEMATICS_ERROR);
    return safeStop();
  }
  
  // Apply joint position and velocity limits
  if (!applyJointLimits(period)) {
    reportRTError(RTErrorType::JOINT_LIMITS_ERROR);
    return safeStop();
  }
  
  // Update command interfaces
  updateReferenceInterfaces();
  
  // Publish monitoring data
  publishMonitoringData();
  
  // Process any errors in non-RT context
  processNonRTErrors();
  
  return controller_interface::return_type::OK; 
}

controller_interface::return_type AdmittanceController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  return update_reference_from_subscribers(time, period);
}

/**
 * @brief Compute admittance control law using Runge-Kutta 4th order integration
 *
 * The admittance control law is:
 * M*a + D*v + K*x = F_external
 *
 * Where:
 * - M: Virtual mass matrix
 * - D: Damping matrix
 * - K: Stiffness matrix (scaled by engagement factor)
 * - F_external: Measured external force/torque
 *
 * @param period Control loop period
 * @param cmd_vel_out Output Cartesian velocity command
 * @return true if computation successful
 */
bool AdmittanceController::computeAdmittanceControl(const rclcpp::Duration& period, Vector6d& cmd_vel_out)
{
  // Compute pose error between desired and current poses
  error_tip_base_ = computePoseError_tip_base();
  if (error_tip_base_.hasNaN()) {
    return false;
  }
  
  // Update stiffness engagement factor for safe ramping
  if (!updateStiffnessEngagement(period)) {
    return false;
  }
  
  // Validate time step
  double dt = period.seconds();
  if (dt <= 0.0 || dt > 0.1) {
    return false;
  }
  
  // Runge-Kutta 4th order integration
  Vector6d v0 = desired_vel_;
  
  // k1 = f(t_n, v_n)
  Vector6d stiffness_force1 = stiffness_engagement_factor_ * (stiffness_ * error_tip_base_);
  Vector6d k1 = mass_inverse_ * (wrench_filtered_ - damping_ * v0 - stiffness_force1);
  
  if (k1.hasNaN()) {
    return false;
  }
  
  // k2 = f(t_n + dt/2, v_n + k1*dt/2)
  Vector6d v1 = v0 + k1 * (dt / 2.0);
  Vector6d k2 = mass_inverse_ * (wrench_filtered_ - damping_ * v1 - stiffness_force1);
  
  if (k2.hasNaN()) {
    return false;
  }
  
  // k3 = f(t_n + dt/2, v_n + k2*dt/2)
  Vector6d v2 = v0 + k2 * (dt / 2.0);
  Vector6d k3 = mass_inverse_ * (wrench_filtered_ - damping_ * v2 - stiffness_force1);
  
  if (k3.hasNaN()) {
    return false;
  }
  
  // k4 = f(t_n + dt, v_n + k3*dt)
  Vector6d v3 = v0 + k3 * dt;
  Vector6d k4 = mass_inverse_ * (wrench_filtered_ - damping_ * v3 - stiffness_force1);
  
  if (k4.hasNaN()) {
    return false;
  }
  
  // v_{n+1} = v_n + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
  desired_vel_ = v0 + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
  
  if (desired_vel_.hasNaN()) {
    return false;
  }
  
  // Apply axis enable/disable mask
  for (size_t i = 0; i < 6; ++i) {
    if (!params_.admittance.enabled_axes[i]) {
      desired_vel_(i) = 0.0;
    }
  }
  
  // Apply velocity limits
  if (!applyCartesianVelocityLimits()) {
    return false;
  }
  
  // Check for drift and reset if velocity is near zero
  if (desired_vel_.norm() < params_.admittance.drift_reset_threshold) {
    if (!handleDriftReset()) {
      return false;
    }
    cmd_vel_out.setZero();
    return true;
  }
  
  // Output commanded velocity
  V_base_tip_base_ = desired_vel_;
  cmd_vel_out = V_base_tip_base_;
  return true;
}

/**
 * @brief Compute pose error between desired and current end-effector poses
 *
 * This method computes the error in the tip frame:
 * - Position error: simple difference
 * - Orientation error: axis-angle representation of rotation error
 *
 * @return 6D error vector [position_error; orientation_error]
 */
Vector6d AdmittanceController::computePoseError_tip_base()
{
  Vector6d error = Vector6d::Zero();
  
  // Position error (simple difference)
  error.head<3>() = X_base_tip_desired_.translation() - X_base_tip_current_.translation();
  
  // Extract rotation matrices
  Eigen::Matrix3d R_current = X_base_tip_current_.rotation();
  Eigen::Matrix3d R_desired = X_base_tip_desired_.rotation();
  
  // Convert to quaternions and normalize
  Eigen::Quaterniond q_current(R_current);
  q_current.normalize();
  
  Eigen::Quaterniond q_desired(R_desired);
  q_desired.normalize();
  
  // Ensure quaternions are in the same hemisphere
  if (q_current.dot(q_desired) < 0) {
    q_current.coeffs() = -q_current.coeffs();
  }
  
  // Compute error quaternion: q_error = q_desired * q_current^{-1}
  Eigen::Quaterniond q_error = q_desired * q_current.inverse();
  q_error.normalize();
  
  // Convert to axis-angle representation
  Eigen::AngleAxisd aa_error(q_error);
  
  // Handle near-zero rotation
  if (aa_error.angle() < QUATERNION_EPSILON) {
    error.tail<3>().setZero();
  } else {
    // Orientation error as axis * angle
    error.tail<3>() = aa_error.axis() * aa_error.angle();
    
    // Clamp orientation error to maximum allowed
    double error_norm = error.tail<3>().norm();
    if (error_norm > MAX_ORIENTATION_ERROR) {
      error.tail<3>() *= MAX_ORIENTATION_ERROR / error_norm;
    }
  }
  
  return error;
}

bool AdmittanceController::updateStiffnessEngagement(const rclcpp::Duration& period)
{
  if (period.seconds() <= 0.0) {
    return false;
  }
  
  double position_error_norm = error_tip_base_.head<3>().norm();
  double orientation_error_norm = error_tip_base_.tail<3>().norm();
  
  bool error_within_limits = true;
  if (stiffness_engagement_factor_ > STIFFNESS_ENGAGEMENT_THRESHOLD && 
      (position_error_norm > safe_startup_params_.max_position_error || 
       orientation_error_norm > safe_startup_params_.max_orientation_error)) {
    error_within_limits = false;
    
    rtLogWarn(RTLogType::WARN_POSE_ERROR_LIMIT);
    
    stiffness_recently_changed_ = true;
    stiffness_engagement_factor_ = STIFFNESS_REDUCTION_FACTOR;
  }
  
  if (stiffness_recently_changed_ && error_within_limits) {
    stiffness_engagement_factor_ += period.seconds() / safe_startup_params_.stiffness_ramp_time;
    if (stiffness_engagement_factor_ >= 1.0) {
      stiffness_engagement_factor_ = 1.0;
      stiffness_recently_changed_ = false;
      rtLogInfo(RTLogType::INFO_STIFFNESS_ENGAGED);
    }
  }
  
  return true;
}


void AdmittanceController::checkParameterUpdates()
{
  if (!params_.dynamic_parameters) return;

  parameter_update_needed_.store(true);
  
  const auto* latest_params = param_buffer_.readFromRT();
  
  if (!latest_params) return;
  
  params_ = *latest_params;
}

void AdmittanceController::prepareParameterUpdate()
{
  if (!parameter_update_needed_.load()) return;
  
  auto new_params = param_listener_->get_params();
  
  
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
  
  if (!mass_changed && !stiffness_changed && !damping_changed) {
    param_buffer_.writeFromNonRT(new_params);
    parameter_update_needed_.store(false);
    return;
  }
  
  if (mass_changed) {
    updateMassMatrix(new_params, true);
  }
  
  if (stiffness_changed) {
    updateStiffnessMatrix(new_params, true);
    stiffness_recently_changed_ = true;
    stiffness_engagement_factor_ = 0.0;
    RCLCPP_INFO(get_node()->get_logger(), "Starting gradual stiffness engagement");
  }
  
  if (stiffness_changed || damping_changed) {
    updateDampingMatrix(new_params, true);
  }
  
  param_buffer_.writeFromNonRT(new_params);
  parameter_update_needed_.store(false);
}


void AdmittanceController::updateMassMatrix(const ur_admittance_controller::Params& params, bool log_changes)
{
  std::array<double, 6> mass_array;
  for (size_t i = 0; i < 6; ++i) {
    mass_array[i] = params.admittance.mass[i];
    mass_(i, i) = mass_array[i];
  }
  
  mass_inverse_ = utils::computeMassInverse(mass_array);
  
  if (log_changes) {
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

void AdmittanceController::updateMassMatrix()
{
  updateMassMatrix(params_, false);
}

void AdmittanceController::updateStiffnessMatrix(const ur_admittance_controller::Params& params, bool log_changes)
{
  for (size_t i = 0; i < 6; ++i) {
    stiffness_(i, i) = params.admittance.stiffness[i];
  }
  
  if (log_changes) {
    RCLCPP_INFO(get_node()->get_logger(), "Stiffness parameters updated");
  }
}

void AdmittanceController::updateStiffnessMatrix()
{
  updateStiffnessMatrix(params_, false);
}

void AdmittanceController::updateDampingMatrix(const ur_admittance_controller::Params& params, bool log_changes)
{
  for (size_t i = 0; i < 6; ++i) {
    double critical_damping = 2.0 * std::sqrt(params.admittance.stiffness[i] * params.admittance.mass[i]);
    damping_(i, i) = params.admittance.damping_ratio[i] * critical_damping;
  }
  
  if (log_changes) {
    RCLCPP_INFO(get_node()->get_logger(), 
      "Damping parameters updated with simplified critical damping formula");
  }
}

void AdmittanceController::updateDampingMatrix()
{
  updateDampingMatrix(params_, false);
}


bool AdmittanceController::applyCartesianVelocityLimits()
{
  if (desired_vel_.hasNaN()) {
    return false;
  }
  
  double linear_velocity_magnitude = desired_vel_.head<3>().norm();
  double angular_velocity_magnitude = desired_vel_.tail<3>().norm();
  
  if (linear_velocity_magnitude > params_.max_linear_velocity) {
    desired_vel_.head<3>() *= (params_.max_linear_velocity / linear_velocity_magnitude);
  }
  
  if (angular_velocity_magnitude > params_.max_angular_velocity) {
    desired_vel_.tail<3>() *= (params_.max_angular_velocity / angular_velocity_magnitude);
  }
  
  return !desired_vel_.hasNaN();
}

/**
 * @brief Convert Cartesian velocity to joint space using inverse kinematics
 *
 * @param cart_vel Cartesian velocity command (6D)
 * @param period Control loop period
 * @return true if conversion successful
 */
bool AdmittanceController::convertToJointSpace(
    const Vector6d& cart_vel, 
    const rclcpp::Duration& period)
{
  // Validate inputs
  if (period.seconds() <= 0.0 || cart_vel.hasNaN()) {
    return false;
  }
  
  // Check array sizes
  if (params_.joints.size() == 0 || 
      pos_state_indices_.size() < params_.joints.size() ||
      current_pos_.size() < params_.joints.size()) {
    return false;
  }
  
  // Get current joint positions
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    size_t idx = pos_state_indices_[i];
    if (idx >= state_interfaces_.size()) {
      return false;
    }
    auto value = state_interfaces_[idx].get_optional();
    if (!value.has_value()) {
      return false;
    }
    current_pos_[i] = value.value();
    if (std::isnan(current_pos_[i])) {
      return false;
    }
  }
  
  // Convert velocity to displacement
  for (size_t i = 0; i < 6 && i < cart_displacement_deltas_.size(); ++i) {
    cart_displacement_deltas_[i] = cart_vel(i) * period.seconds();
  }
  
  // Call inverse kinematics
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

/**
 * @brief Apply joint position and velocity limits
 *
 * @param period Control loop period
 * @return true if limits applied successfully
 */
bool AdmittanceController::applyJointLimits(const rclcpp::Duration& period)
{
  // Validate period
  if (period.seconds() <= 0.0) {
    return false;
  }
  
  // Validate array sizes
  if (params_.joints.size() == 0 || joint_deltas_.size() == 0 ||
      params_.joints.size() > joint_positions_.size() || 
      joint_deltas_.size() > joint_positions_.size() ||
      joint_limits_.size() < params_.joints.size()) {
    return false;
  }
  
  // Apply limits to each joint
  for (size_t i = 0; i < params_.joints.size() && i < joint_deltas_.size(); ++i) {
    // Check velocity limit
    double velocity = joint_deltas_[i] / period.seconds();
    if (std::abs(velocity) > joint_limits_[i].max_velocity) {
      // Scale down movement to respect velocity limit
      double scale = joint_limits_[i].max_velocity / std::abs(velocity);
      joint_positions_[i] = current_pos_[i] + joint_deltas_[i] * scale;
    }
    
    // Apply position limits
    joint_positions_[i] = std::max(joint_limits_[i].min_position, 
                          std::min(joint_positions_[i], joint_limits_[i].max_position));
    
    // Check for NaN
    if (std::isnan(joint_positions_[i])) {
      return false;
    }
  }
  
  return true;
}


bool AdmittanceController::handleDriftReset()
{
  desired_vel_.setZero();
  V_base_tip_base_.setZero();
  
  try {
    size_t i = 0;
    for (const auto& index : pos_state_indices_) {
      if (index >= state_interfaces_.size()) {
        return false;
      }
      auto value = state_interfaces_[index].get_optional();
      if (value.has_value()) {
        joint_positions_[i++] = value.value();
      } else {
        return false;
      }
    }
  } catch (...) {
    reportRTError(RTErrorType::CONTROL_ERROR);
    return false;
  }
  
  X_base_tip_desired_ = X_base_tip_current_;
  
  return true;
}

bool AdmittanceController::publishPoseError()
{
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
  processRTLogs();
  
  updateTransformCaches();
  
  prepareParameterUpdate();
  
  RTErrorType error = last_rt_error_.exchange(RTErrorType::NONE);
  
  if (error != RTErrorType::NONE && rclcpp::ok()) {
    switch (error) {
      case RTErrorType::UPDATE_ERROR:
        RCLCPP_ERROR(get_node()->get_logger(), "Real-time error occurred in update loop");
        break;
      case RTErrorType::SENSOR_ERROR:
        RCLCPP_ERROR(get_node()->get_logger(), "Real-time error in sensor data processing");
        break;
      case RTErrorType::TRANSFORM_ERROR:
        RCLCPP_ERROR(get_node()->get_logger(), "Transform lookup failed in real-time context");
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

/**
 * @brief Update command interfaces with computed joint positions
 */
void AdmittanceController::updateReferenceInterfaces()
{
  // Update reference positions
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    joint_position_references_[i] = joint_positions_[i];
  }
  
  // Write to command interfaces
  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    const size_t joint_idx = cmd_interface_to_joint_index_[i];
    
    if (!command_interfaces_[i].set_value(joint_position_references_[joint_idx])) {
      reportRTError(RTErrorType::CONTROL_ERROR);
    }
  }
}

void AdmittanceController::publishMonitoringData()
{
  publishCartesianVelocity();
}

/**
 * @brief Emergency stop with position hold
 *
 * @return Controller return status
 */
controller_interface::return_type AdmittanceController::safeStop()
{
  try {
    // Zero all velocities
    V_base_tip_base_.setZero();
    desired_vel_.setZero();
    
    // Hold current position
    for (size_t i = 0; i < params_.joints.size(); ++i) {
      joint_position_references_[i] = state_interfaces_[pos_state_indices_[i]].get_optional().value();
    }
    
    return controller_interface::return_type::OK;
    
  } catch (const std::exception &) {
    reportRTError(RTErrorType::CONTROL_ERROR);
    return controller_interface::return_type::ERROR;
  }
}

}  // namespace ur_admittance_controller
