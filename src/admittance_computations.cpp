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



inline Matrix6d computeDampingMatrix(
    const std::array<double, 6>& mass,
    const std::array<double, 6>& stiffness, 
    const std::array<double, 6>& damping_ratio)
{
    using namespace constants;
    
    Matrix6d damping = Matrix6d::Zero();
    
    for (size_t i = 0; i < 6; ++i) {
        // Use virtual stiffness for low/zero stiffness values
        double effective_stiffness = (stiffness[i] <= 0.0) ? VIRTUAL_STIFFNESS : stiffness[i];
        
        // Standard critical damping formula: D = 2*ζ*√(M*K)
        damping(i, i) = 2.0 * damping_ratio[i] * std::sqrt(mass[i] * effective_stiffness);
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

// Main admittance control step - REMOVED: Logic moved to computeAdmittanceControlInNode()

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
  if (error_tip_base_.hasNaN()) {
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
  
  if (acceleration.hasNaN()) {
    return false;
  }
  
  // Forward Euler integration: v_new = v_old + a × dt
  desired_vel_ = desired_vel_ + acceleration * dt;
  
  if (desired_vel_.hasNaN()) {
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



// Parameter callback implementation
rcl_interfaces::msg::SetParametersResult AdmittanceNode::onParameterChange(
  const std::vector<rclcpp::Parameter> & /*parameters*/) 
{
  // Reload parameters from generate_parameter_library
  params_ = param_listener_->get_params();
  
  // Update matrices with new parameters
  updateMassMatrix();
  updateDampingMatrix();
  
  // Update stiffness matrix (diagonal assignment)
  for (size_t i = 0; i < 6; ++i) {
    stiffness_(i, i) = params_.admittance.stiffness[i];
  }
  
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}



void AdmittanceNode::updateMassMatrix()
{
  std::array<double, 6> mass_array = paramVectorToArray(params_.admittance.mass);
  mass_.diagonal() = Eigen::Map<const Eigen::VectorXd>(mass_array.data(), 6);
  mass_inverse_ = computeMassInverse(mass_array);
}


void AdmittanceNode::updateDampingMatrix()
{
  std::array<double, 6> mass_array = paramVectorToArray(params_.admittance.mass);
  std::array<double, 6> stiffness_array = paramVectorToArray(params_.admittance.stiffness);
  std::array<double, 6> damping_ratio_array = paramVectorToArray(params_.admittance.damping_ratio);
  
  damping_ = computeDampingMatrix(mass_array, stiffness_array, damping_ratio_array);
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
    if (std::isnan(joint_velocities_[i])) return false;
  }
  
  // INTEGRATION REMOVED: Now done in thread's integrateAndPublish() method
  // joint_positions_[i] = current_pos_[i] + joint_velocities_[i] * period.seconds();
  
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





// Removed unused safeStop() function

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

// Unified control step - everything in one blazing-fast function
bool AdmittanceNode::unifiedControlStep(double dt)
{
  // 1. Lazy kinematics initialization
  if (!kinematics_ready_) {
    if (robot_description_received_.load()) {
      loadKinematics();
    }
    if (!kinematics_ready_) {
      return false;
    }
  }
  
  // 2. Initialize desired pose to current robot pose (only once)
  if (!desired_pose_initialized_.load()) {
    if (!initializeDesiredPose()) {
      return false;
    }
  }
  
  // 3. Update current pose
  if (!getCurrentEndEffectorPose(X_base_tip_current_)) {
    return false;
  }
  
  // 4. Check deadband
  if (!checkDeadband()) {
    V_base_tip_base_.setZero();
    desired_vel_.setZero();
    // Zero velocities - robot stops
    for (size_t i = 0; i < joint_velocities_.size(); ++i) {
      joint_velocities_[i] = 0.0;
    }
  } else {
    // 5. Compute admittance control
    Vector6d cmd_vel;
    rclcpp::Duration period = rclcpp::Duration::from_seconds(dt);
    if (computeAdmittanceControl(period, cmd_vel)) {
      // 6. Convert to joint space
      if (convertToJointSpace(cmd_vel, period)) {
        V_base_tip_base_ = cmd_vel;
      } else {
        return false;
      }
    } else {
      return false;
    }
  }
  
  // 7. Integrate joint positions (direct integration, no mutex overhead)
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    for (size_t i = 0; i < params_.joints.size() && 
                       i < joint_velocities_.size() && 
                       i < joint_positions_.size(); ++i) {
      joint_positions_[i] += joint_velocities_[i] * dt;
    }
  }
  
  // 8. Publish trajectory command
  trajectory_msg_.header.stamp = now();
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    trajectory_msg_.points[0].positions = joint_positions_;
  }
  trajectory_msg_.points[0].velocities = joint_velocities_;
  trajectory_pub_->publish(trajectory_msg_);
  
  return true;
}

}  // namespace ur_admittance_controller
