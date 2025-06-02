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
  error_tcp_base_ = X_tcp_base_error();
  if (error_tcp_base_.hasNaN()) {
    return false;
  }
  
  // Enforce safety limits on pose error magnitude
  if (!ValidatePoseErrorSafety(error_tcp_base_)) {
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
      (Wrench_tcp_base_.array() - D_diag_.array() * V_tcp_base_desired_.array() -
       K_diag_.array() * error_tcp_base_.array());
  if (acceleration.hasNaN()) {
    return false;
  }
  // Forward Euler integration to update velocity
  V_tcp_base_desired_ = V_tcp_base_desired_ + acceleration * dt;
  if (V_tcp_base_desired_.hasNaN()) {
    return false;
  }
  // Apply selective compliance - disable motion on specified axes
  for (size_t i = 0; i < 6; ++i) {
    if (!params_.admittance.enabled_axes[i]) {
      V_tcp_base_desired_(i) = 0.0;
    }
  }
  // No drift reset - user controls desired pose via topic
  // Output final Cartesian velocity command
  cmd_vel_out = V_tcp_base_desired_;
  return true;
}

// Compute 6-DOF pose error: [position_error; orientation_error]
// Uses quaternion-based orientation error to avoid singularities
Vector6d AdmittanceNode::X_tcp_base_error() {
  Vector6d error = Vector6d::Zero();

  // Access reference pose (no mutex needed - single threaded)
  Eigen::Isometry3d desired_pose = X_tcp_base_desired_;
  // Compute position error as simple vector difference
  error.head<3>() = desired_pose.translation() - X_tcp_base_current_.translation();
  // Extract rotation matrices for orientation error computation
  const Eigen::Matrix3d R_current = X_tcp_base_current_.rotation();
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

// Update stiffness matrix and diagonal cache from parameters
void AdmittanceNode::UpdateStiffnessMatrix() {
  // Update both matrix and diagonal cache consistently
  for (size_t i = 0; i < 6; ++i) {
    K_(i, i) = params_.admittance.stiffness[i];
    K_diag_(i) = params_.admittance.stiffness[i];
  }
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
}

// Update all admittance matrices (M, D, K) consistently
void AdmittanceNode::UpdateAdmittanceMatrices() {
  // Update in correct order: M and K first, then D (which depends on both)
  UpdateMassMatrix();      // M depends on nothing
  UpdateStiffnessMatrix(); // K depends on nothing  
  UpdateDampingMatrix();   // D depends on M and K
}

// Transform Cartesian velocity to joint velocities using KDL inverse kinematics
bool AdmittanceNode::CartesianVelocityToJointVelocity(const Vector6d& cart_vel) {
  // Only check for NaN - other checks are redundant (arrays sized in constructor)
  if (cart_vel.hasNaN()) {
    RCLCPP_ERROR(get_logger(), "NaN detected in Cartesian velocity command");
    return false;
  }
  
  // Get current joint positions from sensor data (already in q_current_)
  // No need to copy - q_current_ is already updated by JointStateCallback
  
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
  for (size_t i = 0; i < std::min(q_dot_cmd_.size(), (size_t)kdl_chain_.getNrOfJoints()); ++i) {
    q_dot_cmd_[i] = v_kdl(i);  // [rad/s]
    if (std::isnan(q_dot_cmd_[i])) return false;
  }
  
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
  // 1. Update current pose
  if (!GetCurrentEndEffectorPose(X_tcp_base_current_)) {
    return false;
  }
  
  // 4. Check deadband
  if (!CheckDeadband()) {
    V_tcp_base_commanded_.setZero();
    // Note: Keep V_tcp_base_desired_ for dynamics continuity
    // Zero velocities - robot stops
    for (size_t i = 0; i < q_dot_cmd_.size(); ++i) {
      q_dot_cmd_[i] = 0.0;
    }
  } else {
    // 5. Compute admittance control (OPTIMIZED version)
    rclcpp::Duration period = rclcpp::Duration::from_seconds(dt);
    if (ComputeAdmittanceControl(period, V_tcp_base_commanded_)) {
      // 6. Convert to joint space
      if (!CartesianVelocityToJointVelocity(V_tcp_base_commanded_)) {
        return false;
      }
    } else {
      return false;
    }
  }
  
  // 7. Integrate command positions
  for (size_t i = 0; i < params_.joints.size() && 
                     i < q_dot_cmd_.size() && 
                     i < q_cmd_.size(); ++i) {
    q_cmd_[i] += q_dot_cmd_[i] * dt;  // Command integration
  }
  
  // 8. Publish trajectory command using command positions
  trajectory_msg_.header.stamp = now();
  trajectory_msg_.points[0].positions = q_cmd_;  // Use command positions
  trajectory_msg_.points[0].velocities = q_dot_cmd_;
  trajectory_pub_->publish(trajectory_msg_);
  
  return true;
}

}  // namespace ur_admittance_controller
