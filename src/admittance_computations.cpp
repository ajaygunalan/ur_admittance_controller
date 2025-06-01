// Core admittance control algorithm implementation

#include "admittance_node.hpp"
#include "admittance_constants.hpp"
#include <algorithm>
#include <cmath>

namespace ur_admittance_controller {

using namespace constants;

// Inlined matrix utility functions
namespace {

// Helper function to convert parameter vectors to arrays
inline std::array<double, 6> paramVectorToArray(const std::vector<double>& param_vec) {
  std::array<double, 6> result;
  for (size_t i = 0; i < 6; ++i) {
    result[i] = param_vec[i];
  }
  return result;
}

// Template function for diagonal matrix assignment
template<typename MatrixType, typename ArrayType>
inline void assignDiagonalMatrix(MatrixType& matrix, const ArrayType& array) {
  for (size_t i = 0; i < 6; ++i) {
    matrix(i, i) = array[i];
  }
}

// Validate Vector6d for NaN values
inline bool validateVector6d(const Vector6d& vec) {
  return !vec.hasNaN();
}

inline Matrix6d computeDampingMatrix(
    const std::array<double, 6>& mass,
    const std::array<double, 6>& stiffness, 
    const std::array<double, 6>& damping_ratio)
{
    using namespace constants;
    
    Matrix6d damping = Matrix6d::Zero();
    
    for (size_t i = 0; i < 6; ++i) {
        const double mass_value = mass[i];
        const double stiffness_value = stiffness[i];
        const double damping_ratio_value = damping_ratio[i];
        
        if (stiffness_value <= 0.0) {
            damping(i, i) = 2.0 * damping_ratio_value * 
                           std::sqrt(mass_value * VIRTUAL_STIFFNESS);
        } 
        else if (stiffness_value >= STIFFNESS_BLEND_THRESHOLD) {
            damping(i, i) = 2.0 * damping_ratio_value * 
                           std::sqrt(mass_value * stiffness_value);
        }
        else {
            const double blend_factor = stiffness_value / STIFFNESS_BLEND_THRESHOLD;
            const double admittance_damping = 2.0 * damping_ratio_value * 
                                             std::sqrt(mass_value * VIRTUAL_STIFFNESS);
            const double impedance_damping = 2.0 * damping_ratio_value * 
                                            std::sqrt(mass_value * stiffness_value);
            damping(i, i) = (1.0 - blend_factor) * admittance_damping + 
                           blend_factor * impedance_damping;
        }
    }
    
    return damping;
}

inline Matrix6d computeMassInverse(const std::array<double, 6>& mass)
{
    using namespace constants;
    
    Matrix6d mass_matrix = Matrix6d::Zero();
    Matrix6d mass_inverse = Matrix6d::Zero();
    
    for (size_t i = 0; i < 6; ++i) {
        mass_matrix(i, i) = mass[i];
    }
    
    const double max_mass = mass_matrix.diagonal().maxCoeff();
    const double min_mass = mass_matrix.diagonal().minCoeff();
    const double condition_number = max_mass / min_mass;
    
    if (condition_number > MAX_CONDITION_NUMBER || min_mass <= 0.0) {
        for (size_t i = 0; i < 6; ++i) {
            mass_matrix(i, i) += REGULARIZATION_FACTOR;
        }
    }
    
    for (size_t i = 0; i < 6; ++i) {
        mass_inverse(i, i) = 1.0 / mass_matrix(i, i);
    }
    
    return mass_inverse;
}


}  // namespace

// Main admittance control step

bool AdmittanceNode::computeAdmittanceStep(const rclcpp::Duration & period)
{
  // Lazy initialization: try to load kinematics if not ready yet
  if (!kinematics_ready_) {
    if (robot_description_received_.load()) {
      loadKinematics();  // Attempt initialization, ignore return value
    }
    if (!kinematics_ready_) {
      return false;  // Still not ready, skip this cycle
    }
  }
  
  // Initialize desired pose to current robot pose (only once)
  if (!desired_pose_initialized_.load()) {
    if (!initializeDesiredPose()) {
      return false;  // Desired pose not ready yet
    }
  }
  
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
  
  return true;
}

// Compute admittance control using forward Euler integration
//
// MATHEMATICAL FRAMEWORK:
// 1. Admittance equation: M·a + D·v + K·x = F_external  
// 2. Solve for acceleration: a = M⁻¹ × (F_external - D·v - K·x)
// 3. Integrate acceleration: v_new = v_old + a × dt
//
bool AdmittanceNode::computeAdmittanceControl(const rclcpp::Duration& period, Vector6d& cmd_vel_out)
{
  // Compute pose error between desired and current poses
  error_tip_base_ = computePoseError_tip_base();
  if (!validateVector6d(error_tip_base_)) {
    return false;
  }
  
  // Safety check: validate pose error is within safe limits
  if (!validatePoseErrorSafety(error_tip_base_)) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, 
      "Admittance control disabled due to unsafe pose error magnitude");
    return false;
  }
  
  
  // Validate time step
  double dt = period.seconds();
  if (dt <= 0.0 || dt > 0.1) {
    return false;
  }
  
  // Compute acceleration from admittance equation: a = M⁻¹ × (F_external - D·v - K·x)
  Vector6d acceleration = mass_inverse_ * (wrench_filtered_ - damping_ * desired_vel_ - stiffness_ * error_tip_base_);
  
  if (!validateVector6d(acceleration)) {
    return false;
  }
  
  // Forward Euler integration: v_new = v_old + a × dt
  desired_vel_ = desired_vel_ + acceleration * dt;
  
  if (!validateVector6d(desired_vel_)) {
    return false;
  }
  
  // Apply axis enable/disable mask
  for (size_t i = 0; i < 6; ++i) {
    if (!params_.admittance.enabled_axes[i]) {
      desired_vel_(i) = 0.0;
    }
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

// Compute pose error between desired and current end-effector poses
Vector6d AdmittanceNode::computePoseError_tip_base()
{
  Vector6d error = Vector6d::Zero();
  
  // Get desired pose with thread safety
  Eigen::Isometry3d desired_pose;
  {
    std::lock_guard<std::mutex> lock(desired_pose_mutex_);
    desired_pose = X_base_tip_desired_;
  }
  
  // Position error (simple difference)
  error.head<3>() = desired_pose.translation() - X_base_tip_current_.translation();
  
  // Extract rotation matrices
  Eigen::Matrix3d R_current = X_base_tip_current_.rotation();
  Eigen::Matrix3d R_desired = desired_pose.rotation();
  
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



void AdmittanceNode::checkParameterUpdates()
{
  if (!params_.dynamic_parameters) return;
  
  // Reduce parameter check frequency for better real-time performance
  auto now = this->now();
  if ((now - last_param_check_).seconds() < PARAM_CHECK_INTERVAL) {
    return;
  }
  last_param_check_ = now;

  // Use non-blocking parameter access for real-time safety
  ur_admittance_controller::Params temp_params = params_;
  if (param_listener_->try_get_params(temp_params)) {
    // Parameters were updated and lock was acquired without blocking
    
    // Check what changed to avoid unnecessary matrix recomputation
    bool mass_changed = (temp_params.admittance.mass != params_.admittance.mass);
    bool stiffness_changed = (temp_params.admittance.stiffness != params_.admittance.stiffness);
    bool damping_changed = (temp_params.admittance.damping_ratio != params_.admittance.damping_ratio);
    
    params_ = temp_params;
    
    // Only update matrices that actually changed
    if (mass_changed) {
      updateMassMatrix(true);
      updateDampingMatrix(true); // Damping depends on mass
      RCLCPP_INFO(get_logger(), "Mass parameters updated");
    }
    if (stiffness_changed) {
      updateStiffnessMatrix(true);
      if (!mass_changed) updateDampingMatrix(true); // Damping depends on stiffness
      RCLCPP_INFO(get_logger(), "Stiffness parameters updated");
    }
    if (damping_changed && !mass_changed && !stiffness_changed) {
      updateDampingMatrix(true);
      RCLCPP_INFO(get_logger(), "Damping parameters updated");
    }
    
    // Enhanced parameter change tracking and logging
    if (mass_changed || stiffness_changed || damping_changed) {
      RCLCPP_INFO(get_logger(), 
        "Parameter update completed at %.3f seconds - Mass:%s Stiffness:%s Damping:%s",
        now.seconds(),
        mass_changed ? "✓" : "○",
        stiffness_changed ? "✓" : "○", 
        damping_changed ? "✓" : "○");
        
      RCLCPP_DEBUG(get_logger(), "Admittance parameters updated");
    }
  }
  // If try_get_params returns false, continue with existing parameters (non-blocking)
}



void AdmittanceNode::updateMassMatrix(bool log_changes)
{
  std::array<double, 6> mass_array = paramVectorToArray(params_.admittance.mass);
  assignDiagonalMatrix(mass_, mass_array);
  
  mass_inverse_ = computeMassInverse(mass_array);
  
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

void AdmittanceNode::updateStiffnessMatrix(bool log_changes)
{
  std::array<double, 6> stiffness_array = paramVectorToArray(params_.admittance.stiffness);
  assignDiagonalMatrix(stiffness_, stiffness_array);
  
  if (log_changes) {
    RCLCPP_INFO(get_logger(), "Stiffness parameters updated");
  }
}

void AdmittanceNode::updateDampingMatrix(bool log_changes)
{
  std::array<double, 6> mass_array = paramVectorToArray(params_.admittance.mass);
  std::array<double, 6> stiffness_array = paramVectorToArray(params_.admittance.stiffness);
  std::array<double, 6> damping_ratio_array = paramVectorToArray(params_.admittance.damping_ratio);
  
  damping_ = computeDampingMatrix(mass_array, stiffness_array, damping_ratio_array);
  
  if (log_changes) {
    RCLCPP_INFO(get_logger(), 
      "Damping parameters updated using robust matrix utilities (with virtual stiffness blending)");
  }
}


// Convert Cartesian velocity to joint space using inverse kinematics
bool AdmittanceNode::convertToJointSpace(
    const Vector6d& cart_vel, 
    const rclcpp::Duration& period)
{
  // Validate inputs and sizes
  if (period.seconds() <= 0.0 || cart_vel.hasNaN() || params_.joints.empty() || 
      joint_velocities_.size() < params_.joints.size() ||
      joint_positions_.size() < params_.joints.size()) return false;
  
  // Get current joint positions from stored values
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    current_pos_ = joint_positions_;
  }
  
  // Check KDL readiness
  if (!kinematics_ready_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "KDL kinematics not ready, waiting for initialization");
    return false;
  }

  // Convert to KDL types
  KDL::JntArray q_current(kdl_chain_.getNrOfJoints());
  for (size_t i = 0; i < std::min(current_pos_.size(), (size_t)kdl_chain_.getNrOfJoints()); ++i) {
    q_current(i) = current_pos_[i];
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
  KDL::JntArray q_dot(kdl_chain_.getNrOfJoints());
  if (ik_vel_solver_->CartToJnt(q_current, cart_twist, q_dot) < 0) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "KDL IK velocity solver failed");
    return false;
  }

  // Store joint velocities directly (CORRECT: velocity → velocity)
  for (size_t i = 0; i < std::min(joint_velocities_.size(), (size_t)kdl_chain_.getNrOfJoints()); ++i) {
    joint_velocities_[i] = q_dot(i);  // [rad/s]
  }
  
  // Integrate velocities to get positions (CORRECT: q = q₀ + v×Δt)
  for (size_t i = 0; i < params_.joints.size() && 
                     i < joint_velocities_.size() && 
                     i < joint_positions_.size() && 
                     i < current_pos_.size(); ++i) {
    joint_positions_[i] = current_pos_[i] + joint_velocities_[i] * period.seconds();
    if (std::isnan(joint_positions_[i]) || std::isnan(joint_velocities_[i])) return false;
  }
  
  return true;
}

bool AdmittanceNode::handleDriftReset()
{
  desired_vel_.setZero();
  V_base_tip_base_.setZero();
  
  // Update desired pose to current pose (reset reference)
  {
    std::lock_guard<std::mutex> lock(desired_pose_mutex_);
    X_base_tip_desired_ = X_base_tip_current_;
  }
  
  RCLCPP_DEBUG(get_logger(), "Drift reset: desired pose updated to current pose");
  
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




// Emergency stop with position hold
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

bool AdmittanceNode::validatePoseErrorSafety(const Vector6d& pose_error)
{
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

}  // namespace ur_admittance_controller
