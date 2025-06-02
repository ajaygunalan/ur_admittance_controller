// Core admittance control algorithm implementation
// Implements the classic admittance equation: M*accel + D*vel + K*pos = F_external

#include "admittance_node.hpp"
#include "admittance_constants.hpp"
#include <algorithm>
#include <cmath>

namespace ur_admittance_controller {

using namespace constants;

// Solve admittance equation and integrate to compute Cartesian velocity commands
// Mathematical model: M*acceleration + D*velocity + K*position_error = F_external
// Solves for acceleration, then integrates: v_new = v_old + acceleration * dt
bool AdmittanceNode::ComputeAdmittanceControl(const rclcpp::Duration& period, Vector6d& cmd_vel_out) {
  // Calculate 6-DOF pose error (position + orientation)
  error_tip_base_ = ComputePoseError_tip_base();
  if (error_tip_base_.hasNaN()) {
    return false;
  }
  
  // Enforce safety limits on pose error magnitude
  if (!ValidatePoseErrorSafety(error_tip_base_)) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Safety stop: pose error exceeds safe limits");
    return false;
  }
  
  // Validate integration time step for numerical stability
  const double dt = period.seconds();
  if (dt <= 0.0 || dt > 0.1) {
    return false;
  }
  // Solve admittance equation for acceleration using optimized diagonal operations
  // acceleration = M^-1 * (F_external - D*velocity - K*position_error)
  const Vector6d acceleration = M_inverse_diag_.array() *
      (F_sensor_filtered_.array() - D_diag_.array() * V_base_tip_desired_.array() -
       K_diag_.array() * error_tip_base_.array());
  if (acceleration.hasNaN()) {
    return false;
  }
  // Forward Euler integration to update velocity
  V_base_tip_desired_ = V_base_tip_desired_ + acceleration * dt;
  if (V_base_tip_desired_.hasNaN()) {
    return false;
  }
  // Apply selective compliance - disable motion on specified axes
  for (size_t i = 0; i < 6; ++i) {
    if (!params_.admittance.enabled_axes[i]) {
      V_base_tip_desired_(i) = 0.0;
    }
  }
  // Handle drift compensation when motion is minimal
  if (V_base_tip_desired_.norm() < params_.admittance.drift_reset_threshold) {
    if (!HandleDriftReset()) {
      return false;
    }
    cmd_vel_out.setZero();
    return true;
  }
  // Output final Cartesian velocity command
  V_base_tip_base_ = V_base_tip_desired_;
  cmd_vel_out = V_base_tip_base_;
  return true;
}

// Compute 6-DOF pose error: [position_error; orientation_error]
// Uses quaternion-based orientation error to avoid singularities
Vector6d AdmittanceNode::ComputePoseError_tip_base() {
  Vector6d error = Vector6d::Zero();

  // Safely access reference pose
  Eigen::Isometry3d desired_pose;
  {
    std::lock_guard<std::mutex> lock(desired_pose_mutex_);
    desired_pose = X_base_tip_desired_;
  }
  // Compute position error as simple vector difference
  error.head<3>() = desired_pose.translation() - X_base_tip_current_.translation();
  // Extract rotation matrices for orientation error computation
  const Eigen::Matrix3d R_current = X_base_tip_current_.rotation();
  const Eigen::Matrix3d R_desired = desired_pose.rotation();
  // Convert to unit quaternions for robust orientation representation
  Eigen::Quaterniond q_current(R_current);
  q_current.normalize();

  Eigen::Quaterniond q_desired(R_desired);
  q_desired.normalize();
  // Ensure shortest rotation path by choosing same quaternion hemisphere
  if (q_current.dot(q_desired) < 0) {
    q_current.coeffs() = -q_current.coeffs();
  }
  // Compute orientation error quaternion: q_error = q_desired * q_current^-1
  Eigen::Quaterniond q_error = q_desired * q_current.inverse();
  q_error.normalize();
  // Convert error quaternion to axis-angle for linear control
  const Eigen::AngleAxisd aa_error(q_error);
  // Convert to 3D orientation error vector (axis * angle)
  if (aa_error.angle() < QUATERNION_EPSILON) {
    error.tail<3>().setZero();  // Handle numerical precision near identity
  } else {
    error.tail<3>() = aa_error.axis() * aa_error.angle();

    // Clamp orientation error to prevent excessive rotation commands
    const double error_norm = error.tail<3>().norm();
    if (error_norm > MAX_ORIENTATION_ERROR) {
      error.tail<3>() *= MAX_ORIENTATION_ERROR / error_norm;
    }
  }
  return error;
}



// Removed redundant manual parameter callback - using auto-generated parameter handling


// Update 6x6 virtual mass matrix from parameters
void AdmittanceNode::UpdateMassMatrix() {
  // Build diagonal mass matrix from parameter array
  M_.diagonal() = Eigen::Map<const Eigen::VectorXd>(params_.admittance.mass.data(), 6);
  M_inverse_.diagonal() = M_.diagonal().cwiseInverse();

  // Cache diagonal for optimized element-wise operations
  M_inverse_diag_ = Eigen::Map<const Eigen::VectorXd>(params_.admittance.mass.data(), 6).cwiseInverse();
}

// Compute damping matrix using critical damping relationship: D = 2*ζ*√(M*K)
void AdmittanceNode::UpdateDampingMatrix() {
  using namespace constants;

  // Calculate critical damping for each DOF independently
  for (size_t i = 0; i < 6; ++i) {
    // Use virtual stiffness when stiffness is zero (pure admittance mode)
    const double effective_stiffness = (params_.admittance.stiffness[i] <= 0.0)
                                           ? VIRTUAL_STIFFNESS
                                           : params_.admittance.stiffness[i];

    // Critical damping formula for stable second-order response
    const double damping_value = 2.0 * params_.admittance.damping_ratio[i] *
                                 std::sqrt(params_.admittance.mass[i] * effective_stiffness);

    D_(i, i) = damping_value;
    D_diag_(i) = damping_value;  // Cache for performance
  }

  // Update stiffness diagonal cache
  for (size_t i = 0; i < 6; ++i) {
    K_diag_(i) = params_.admittance.stiffness[i];
  }
}


// Transform Cartesian velocity to joint velocities using KDL inverse kinematics
bool AdmittanceNode::ConvertToJointSpace(const Vector6d& cart_vel,
                                        const rclcpp::Duration& period) {
  // Validate input parameters and vector sizes
  if (period.seconds() <= 0.0 || cart_vel.hasNaN() || params_.joints.empty() ||
      v_.size() < params_.joints.size() ||
      q_.size() < params_.joints.size()) {
    return false;
  }
  
  // Get current joint positions from sensor data (read-only)
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    q_current_ = q_;  // Use sensor positions for IK
  }
  
  // Check KDL readiness
  if (!kinematics_ready_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "KDL kinematics not ready, waiting for initialization");
    return false;
  }

  // Convert to KDL types
  KDL::JntArray q_kdl(kdl_chain_.getNrOfJoints());
  for (size_t i = 0; i < std::min(q_current_.size(), (size_t)kdl_chain_.getNrOfJoints()); ++i) {
    q_kdl(i) = q_current_[i];
  }

  // Send Cartesian velocity directly to KDL (CORRECT: velocity → velocity)
  KDL::Twist cart_twist;
  cart_twist.vel.x(cart_vel(0));    // Linear velocity [m/s]
  cart_twist.vel.y(cart_vel(1));
  cart_twist.vel.z(cart_vel(2));
  cart_twist.rot.x(cart_vel(3));    // Angular velocity [rad/s]
  cart_twist.rot.y(cart_vel(4));
  cart_twist.rot.z(cart_vel(5));

  // Solve for joint velocities using KDL
  KDL::JntArray v_kdl(kdl_chain_.getNrOfJoints());
  if (ik_vel_solver_->CartToJnt(q_kdl, cart_twist, v_kdl) < 0) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "KDL IK velocity solver failed");
    return false;
  }

  // Store joint velocities directly (CORRECT: velocity → velocity)
  for (size_t i = 0; i < std::min(v_.size(), (size_t)kdl_chain_.getNrOfJoints()); ++i) {
    v_[i] = v_kdl(i);  // [rad/s]
    if (std::isnan(v_[i])) return false;
  }
  
  return true;
}

// Reset velocity states and update reference pose to eliminate drift
bool AdmittanceNode::HandleDriftReset() {
  // Clear velocity states
  V_base_tip_desired_.setZero();
  V_base_tip_base_.setZero();

  // Reset reference pose to current pose (eliminates accumulated error)
  {
    std::lock_guard<std::mutex> lock(desired_pose_mutex_);
    X_base_tip_desired_ = X_base_tip_current_;
  }

  RCLCPP_DEBUG(get_logger(), "Drift compensation: reference pose updated");
  return true;
}

bool AdmittanceNode::ValidatePoseErrorSafety(const Vector6d& pose_error) {
  using namespace constants;
  
  // Check position error magnitude
  double position_error_norm = pose_error.head<3>().norm();
  double orientation_error_norm = pose_error.tail<3>().norm();
  
  if (position_error_norm > MAX_SAFE_POSITION_ERROR) {
    RCLCPP_ERROR(get_logger(), "SAFETY: Position error %.3f m > %.3f m limit", 
      position_error_norm, MAX_SAFE_POSITION_ERROR);
    return false;
  }
  
  if (orientation_error_norm > MAX_SAFE_ORIENTATION_ERROR) {
    RCLCPP_ERROR(get_logger(), "SAFETY: Orientation error %.3f rad (%.1f°) > %.3f rad limit", 
      orientation_error_norm, orientation_error_norm * 180.0 / M_PI, MAX_SAFE_ORIENTATION_ERROR);
    return false;
  }
  
  return true;
}

// Unified control step - everything in one blazing-fast function
bool AdmittanceNode::UnifiedControlStep(double dt) {
  // 1. Check kinematics readiness (initialized in constructor)
  if (!kinematics_ready_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, 
                         "Kinematics not ready - LoadKinematics() failed in constructor");
    return false;
  }
  
  // 2. Initialize desired pose to current robot pose (only once)
  if (!desired_pose_initialized_.load()) {
    if (!InitializeDesiredPose()) {
      return false;
    }
  }
  
  // 2.5. Check for parameter updates (auto-generated parameter library)
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
    UpdateMassMatrix();
    UpdateDampingMatrix();
    // Update stiffness matrix
    for (size_t i = 0; i < 6; ++i) {
      K_(i, i) = params_.admittance.stiffness[i];
    }
  }
  
  // 3. Update current pose
  if (!GetCurrentEndEffectorPose(X_base_tip_current_)) {
    return false;
  }
  
  // 4. Check deadband
  if (!CheckDeadband()) {
    V_base_tip_base_.setZero();
    V_base_tip_desired_.setZero();
    // Zero velocities - robot stops
    for (size_t i = 0; i < v_.size(); ++i) {
      v_[i] = 0.0;
    }
  } else {
    // 5. Compute admittance control (OPTIMIZED version)
    Vector6d cmd_vel;
    rclcpp::Duration period = rclcpp::Duration::from_seconds(dt);
    if (ComputeAdmittanceControl(period, cmd_vel)) {
      // 6. Convert to joint space
      if (ConvertToJointSpace(cmd_vel, period)) {
        V_base_tip_base_ = cmd_vel;
      } else {
        return false;
      }
    } else {
      return false;
    }
  }
  
  // 7. Integrate command positions (RACE CONDITION FIX: separate from sensor data)
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    for (size_t i = 0; i < params_.joints.size() && 
                       i < v_.size() && 
                       i < q_cmd_.size(); ++i) {
      q_cmd_[i] += v_[i] * dt;  // Command integration only
    }
  }
  
  // 8. Publish trajectory command using command positions (RACE CONDITION FIX)
  trajectory_msg_.header.stamp = now();
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    trajectory_msg_.points[0].positions = q_cmd_;  // Use command positions
  }
  trajectory_msg_.points[0].velocities = v_;
  trajectory_pub_->publish(trajectory_msg_);
  
  return true;
}

}  // namespace ur_admittance_controller
