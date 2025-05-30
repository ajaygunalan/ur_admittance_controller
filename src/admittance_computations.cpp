/**
 * @file admittance_computations.cpp
 * @brief Core admittance control algorithm implementation
 *
 * This file contains the pure algorithmic implementations of the admittance
 * control law, independent of any control framework.
 */

#include "admittance_node.hpp"
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
// Main control loop has been moved to controlLoop() in admittance_node.cpp
// The following functions are the core algorithm implementations

bool AdmittanceNode::computeAdmittanceStep(const rclcpp::Duration & period)
{
  // Check for parameter updates from ROS
  checkParameterUpdates();
  
  // Update transform caches if needed
  if (!updateTransforms()) {
    return false;
  }
  
  // Check if forces are within deadband
  if (!checkDeadband()) {
    // Zero velocity when in deadband
    V_base_tip_base_.setZero();
    desired_vel_.setZero();
    return true;
  }
  
  // Compute admittance control law
  Vector6d cmd_vel;
  if (!computeAdmittanceControl(period, cmd_vel)) {
    RCLCPP_ERROR(get_logger(), "Failed to compute admittance control");
    return false;
  }
  
  // Store computed velocity
  V_base_tip_base_ = cmd_vel;
  
  // Convert Cartesian velocity to joint space
  if (!convertToJointSpace(cmd_vel, period)) {
    RCLCPP_ERROR(get_logger(), "Failed to convert to joint space");
    return false;
  }
  
  // Apply joint position and velocity limits
  if (!applyJointLimits(period)) {
    RCLCPP_ERROR(get_logger(), "Joint limits violated");
    return false;
  }
  
  return true;
}

// writeJointCommands removed - trajectory publishing handled in controlLoop()

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
bool AdmittanceNode::computeAdmittanceControl(const rclcpp::Duration& period, Vector6d& cmd_vel_out)
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
Vector6d AdmittanceNode::computePoseError_tip_base()
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

bool AdmittanceNode::updateStiffnessEngagement(const rclcpp::Duration& period)
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
    
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Pose error exceeds safe startup limits");
    
    stiffness_recently_changed_ = true;
    stiffness_engagement_factor_ = STIFFNESS_REDUCTION_FACTOR;
  }
  
  if (stiffness_recently_changed_ && error_within_limits) {
    stiffness_engagement_factor_ += period.seconds() / safe_startup_params_.stiffness_ramp_time;
    if (stiffness_engagement_factor_ >= 1.0) {
      stiffness_engagement_factor_ = 1.0;
      stiffness_recently_changed_ = false;
      RCLCPP_INFO(get_logger(), "Stiffness fully engaged");
    }
  }
  
  return true;
}


void AdmittanceNode::checkParameterUpdates()
{
  if (!params_.dynamic_parameters) return;

  // In the node version, check if parameters were updated
  if (params_updated_.load()) {
    params_ = param_listener_->get_params();
    params_updated_.store(false);
  }
}



void AdmittanceNode::updateMassMatrix(const ur_admittance_controller::Params& params, bool log_changes)
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
      RCLCPP_WARN(get_logger(), 
        "Mass matrix regularized (condition number: %.2e, min mass: %.6f)", 
        condition_number, min_mass);
    } else {
      RCLCPP_INFO(get_logger(), 
        "Mass parameters updated (condition number: %.2e)", condition_number);
    }
  }
}

void AdmittanceNode::updateMassMatrix()
{
  updateMassMatrix(params_, false);
}

void AdmittanceNode::updateStiffnessMatrix(const ur_admittance_controller::Params& params, bool log_changes)
{
  for (size_t i = 0; i < 6; ++i) {
    stiffness_(i, i) = params.admittance.stiffness[i];
  }
  
  if (log_changes) {
    RCLCPP_INFO(get_logger(), "Stiffness parameters updated");
  }
}

void AdmittanceNode::updateStiffnessMatrix()
{
  updateStiffnessMatrix(params_, false);
}

void AdmittanceNode::updateDampingMatrix(const ur_admittance_controller::Params& params, bool log_changes)
{
  for (size_t i = 0; i < 6; ++i) {
    double critical_damping = 2.0 * std::sqrt(params.admittance.stiffness[i] * params.admittance.mass[i]);
    damping_(i, i) = params.admittance.damping_ratio[i] * critical_damping;
  }
  
  if (log_changes) {
    RCLCPP_INFO(get_logger(), 
      "Damping parameters updated with simplified critical damping formula");
  }
}

void AdmittanceNode::updateDampingMatrix()
{
  updateDampingMatrix(params_, false);
}


bool AdmittanceNode::applyCartesianVelocityLimits()
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
bool AdmittanceNode::convertToJointSpace(
    const Vector6d& cart_vel, 
    const rclcpp::Duration& period)
{
  // Validate inputs
  if (period.seconds() <= 0.0 || cart_vel.hasNaN()) {
    return false;
  }
  
  // Check array sizes
  if (params_.joints.size() == 0 || 
      current_pos_.size() < params_.joints.size()) {
    return false;
  }
  
  // Get current joint positions from stored values
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    current_pos_ = joint_positions_;
  }
  
  // Convert velocity to displacement
  for (size_t i = 0; i < 6 && i < cart_displacement_deltas_.size(); ++i) {
    cart_displacement_deltas_[i] = cart_vel(i) * period.seconds();
  }
  
  // Call inverse kinematics
  if (!kinematics_ || !(*kinematics_)->convert_cartesian_deltas_to_joint_deltas(
        current_pos_, cart_displacement_deltas_, params_.tip_link, joint_deltas_)) {
    RCLCPP_ERROR(get_logger(), "Inverse kinematics computation failed for tip_link '%s'", params_.tip_link.c_str());
    return false;
  }
  
  // Update joint positions (velocities will be calculated in applyJointLimits)
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
bool AdmittanceNode::applyJointLimits(const rclcpp::Duration& period)
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
    double current_velocity = joint_deltas_[i] / period.seconds();
    
    // Check acceleration limit (requires previous velocity)
    double acceleration = (current_velocity - previous_joint_velocities_[i]) / period.seconds();
    
    if (std::abs(acceleration) > joint_limits_[i].max_acceleration) {
      // Limit velocity change to respect acceleration constraint
      double max_velocity_change = joint_limits_[i].max_acceleration * period.seconds();
      double limited_velocity;
      
      if (current_velocity > previous_joint_velocities_[i]) {
        limited_velocity = previous_joint_velocities_[i] + max_velocity_change;
      } else {
        limited_velocity = previous_joint_velocities_[i] - max_velocity_change;
      }
      
      // Update joint delta based on limited velocity
      joint_deltas_[i] = limited_velocity * period.seconds();
      current_velocity = limited_velocity;
      
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
        "Joint %zu acceleration limited: %.3f -> %.3f rad/s²", 
        i, acceleration, joint_limits_[i].max_acceleration);
    }
    
    // Check velocity limit
    if (std::abs(current_velocity) > joint_limits_[i].max_velocity) {
      // Scale down movement to respect velocity limit
      double scale = joint_limits_[i].max_velocity / std::abs(current_velocity);
      joint_deltas_[i] *= scale;
      current_velocity *= scale;
      
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
        "Joint %zu velocity limited: %.3f -> %.3f rad/s", 
        i, current_velocity / scale, current_velocity);
    }
    
    // Update joint position
    joint_positions_[i] = current_pos_[i] + joint_deltas_[i];
    
    // Apply position limits
    joint_positions_[i] = std::max(joint_limits_[i].min_position, 
                          std::min(joint_positions_[i], joint_limits_[i].max_position));
    
    // Store velocity for next iteration and output
    joint_velocities_[i] = current_velocity;
    previous_joint_velocities_[i] = current_velocity;
    
    // Check for NaN
    if (std::isnan(joint_positions_[i]) || std::isnan(joint_velocities_[i])) {
      return false;
    }
  }
  
  return true;
}


bool AdmittanceNode::handleDriftReset()
{
  desired_vel_.setZero();
  V_base_tip_base_.setZero();
  
  // In the node version, joint positions are already up to date
  // No need to read from hardware interfaces
  
  X_base_tip_desired_ = X_base_tip_current_;
  
  return true;
}

bool AdmittanceNode::publishPoseError()
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = error_tip_base_(0);
  msg.linear.y = error_tip_base_(1);
  msg.linear.z = error_tip_base_(2);
  msg.angular.x = error_tip_base_(3);
  msg.angular.y = error_tip_base_(4);
  msg.angular.z = error_tip_base_(5);
  pose_error_pub_->publish(msg);
  return true;
}


/**
 * @brief Update command interfaces with computed joint positions
 */
void AdmittanceNode::updateJointReferences()
{
  // In the node version, joint references are updated directly
  // and published in the control loop as trajectory messages
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    joint_position_references_[i] = joint_positions_[i];
  }
}

void AdmittanceNode::publishMonitoringData()
{
  publishCartesianVelocity();
}

/**
 * @brief Emergency stop with position hold
 *
 * @return Controller return status
 */
bool AdmittanceNode::safeStop()
{
  try {
    // Zero all velocities
    V_base_tip_base_.setZero();
    desired_vel_.setZero();
    
    // Set joint velocities to zero
    for (size_t i = 0; i < joint_velocities_.size(); ++i) {
      joint_velocities_[i] = 0.0;
    }
    
    return true;
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Exception in safeStop: %s", e.what());
    return false;
  }
}

}  // namespace ur_admittance_controller
