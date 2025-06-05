#include "admittance_node.hpp"
#include <algorithm>
#include <cmath>

namespace ur_admittance_controller {

void AdmittanceNode::compute_admittance() {
  Vector6d error = compute_pose_error();
  Vector6d scaled_wrench = admittance_ratio_ * Wrench_tcp_base_;
  
  Vector6d acceleration = M_inverse_diag_.array() *
      (scaled_wrench.array() - D_diag_.array() * V_tcp_base_commanded_.array() -
       K_diag_.array() * error.array());
  
  double acc_norm = acceleration.head<3>().norm();
  if (acc_norm > arm_max_acc_) {
    RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000,
                                "Admittance generates high arm acceleration! norm: " << acc_norm);
    acceleration.head<3>() *= (arm_max_acc_ / acc_norm);
  }
  
  const double dt = control_period_.seconds();
  V_tcp_base_commanded_ += acceleration * dt;
}

Vector6d AdmittanceNode::compute_pose_error() {
  Vector6d error;
  error.head<3>() = X_tcp_base_current_.translation() - X_tcp_base_desired_.translation();
  
  Eigen::Quaterniond q_current(X_tcp_base_current_.rotation());
  Eigen::Quaterniond q_desired(X_tcp_base_desired_.rotation());
  
  if (q_current.dot(q_desired) < 0) {
    q_current.coeffs() = -q_current.coeffs();
  }
  
  Eigen::Quaterniond q_error = q_current * q_desired.inverse();
  Eigen::AngleAxisd aa_error(q_error);
  error.tail<3>() = aa_error.axis() * aa_error.angle();
  
  return error;
}

void AdmittanceNode::update_admittance_parameters() {
  auto& p = params_.admittance;
  M_inverse_diag_ = Eigen::Map<const Eigen::VectorXd>(p.mass.data(), 6).cwiseInverse();
  K_diag_ = Eigen::Map<const Eigen::VectorXd>(p.stiffness.data(), 6);
  D_diag_ = Eigen::Map<const Eigen::VectorXd>(p.damping.data(), 6);
}

bool AdmittanceNode::compute_joint_velocities(const Vector6d& cart_vel) {
  // Update KDL solver with current joint angles
  for (size_t i = 0; i < num_joints_; ++i) {
    q_kdl_(i) = q_current_[i];
  }
  
  // Pack Cartesian velocity
  KDL::Twist twist;
  twist.vel = KDL::Vector(cart_vel(0), cart_vel(1), cart_vel(2));
  twist.rot = KDL::Vector(cart_vel(3), cart_vel(4), cart_vel(5));
  
  // Solve inverse kinematics
  if (ik_vel_solver_->CartToJnt(q_kdl_, twist, v_kdl_) < 0) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "IK failed");
    return false;
  }
  
  // Extract solution with NaN safety
  for (size_t i = 0; i < num_joints_; ++i) {
    q_dot_cmd_[i] = v_kdl_(i);
  }
  return std::none_of(q_dot_cmd_.begin(), q_dot_cmd_.end(), 
                      [](double v) { return std::isnan(v); });
}

void AdmittanceNode::computeForwardKinematics() {
  if (!joint_states_updated_) {
    return;
  }
  
  // Copy joints to KDL solver
  for (size_t i = 0; i < num_joints_; ++i) {
    q_kdl_(i) = q_current_[i];
  }
  
  // Solve forward kinematics
  KDL::Frame frame;
  if (fk_pos_solver_->JntToCart(q_kdl_, frame) < 0) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "FK failed");
    return;
  }
  
  // Update TCP pose
  X_tcp_base_current_.translation() = Eigen::Vector3d(frame.p.x(), frame.p.y(), frame.p.z());
  double x, y, z, w;
  frame.M.GetQuaternion(x, y, z, w);
  X_tcp_base_current_.linear() = Eigen::Quaterniond(w, x, y, z).toRotationMatrix();
  
  joint_states_updated_ = false;
}

void AdmittanceNode::initializeStateVectors() {
  const auto joint_count = params_.joints.size();
  
  // Joint space vectors
  q_current_.resize(joint_count, 0.0);
  q_dot_cmd_.resize(joint_count, 0.0);
  
  // Cartesian space vectors
  Wrench_tcp_base_ = Vector6d::Zero();
  V_tcp_base_commanded_ = Vector6d::Zero();
  
  // Poses
  X_tcp_base_current_ = Eigen::Isometry3d::Identity();
  X_tcp_base_desired_ = Eigen::Isometry3d::Identity();
  
  // ROS message
  velocity_msg_.data.resize(joint_count);
  
  // Control limits
  workspace_limits_ << -0.5, 0.5, -0.5, 0.5, 0.0, 0.7;
  arm_max_vel_ = 1.5;
  arm_max_acc_ = 1.0;
  admittance_ratio_ = 1.0;
}

void AdmittanceNode::setDefaultEquilibrium() {
  declare_parameter("equilibrium.position", std::vector<double>{0.1, 0.4, 0.5});
  declare_parameter("equilibrium.orientation", std::vector<double>{0.0, 0.0, 0.0, 1.0});
  
  auto eq_pos = get_parameter("equilibrium.position").as_double_array();
  auto eq_ori = get_parameter("equilibrium.orientation").as_double_array();
  
  X_tcp_base_desired_.translation() << eq_pos[0], eq_pos[1], eq_pos[2];
  X_tcp_base_desired_.linear() = Eigen::Quaterniond(eq_ori[3], eq_ori[0], eq_ori[1], eq_ori[2]).toRotationMatrix();
  
  RCLCPP_DEBUG(get_logger(), "Equilibrium pose set: position=[%.3f, %.3f, %.3f]", 
               eq_pos[0], eq_pos[1], eq_pos[2]);
}

// Initialize KDL kinematics from URDF
bool AdmittanceNode::load_kinematics() {
  std::string urdf_string;
  if (!get_parameter("robot_description", urdf_string) || urdf_string.empty()) {
    RCLCPP_ERROR(get_logger(), "robot_description parameter not found or empty");
    return false;
  }

  if (!kdl_parser::treeFromString(urdf_string, kdl_tree_)) {
    RCLCPP_ERROR(get_logger(), "Failed to parse URDF into KDL tree");
    return false;
  }

  if (!kdl_tree_.getChain(params_.base_link, params_.tip_link, kdl_chain_)) {
    RCLCPP_ERROR(get_logger(), "Failed to extract kinematic chain: %s -> %s",
                 params_.base_link.c_str(), params_.tip_link.c_str());
    return false;
  }

  // Initialize solvers
  constexpr double PRECISION = 1e-5;
  constexpr int MAX_ITERATIONS = 150;
  constexpr double DAMPING = 0.01;
  
  ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_wdls>(kdl_chain_, PRECISION, MAX_ITERATIONS);
  ik_vel_solver_->setLambda(DAMPING);
  fk_pos_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);

  // Pre-allocate arrays
  num_joints_ = kdl_chain_.getNrOfJoints();
  q_kdl_.resize(num_joints_);
  v_kdl_.resize(num_joints_);

  RCLCPP_DEBUG(get_logger(), "KDL kinematics ready: %s -> %s (%zu joints)",
               params_.base_link.c_str(), params_.tip_link.c_str(), num_joints_);
  return true;
}

// Apply workspace limits to Cartesian motion
void AdmittanceNode::limit_to_workspace() {
  static constexpr double BUFFER = 0.01;  // Safety margin before warnings
  const auto& pos = X_tcp_base_current_.translation();
  
  // Enforce per-axis workspace boundaries
  for (size_t i = 0; i < 3; ++i) {
    const double min = workspace_limits_[i * 2];
    const double max = workspace_limits_[i * 2 + 1];
    const bool at_min = pos[i] <= min;
    const bool at_max = pos[i] >= max;
    
    // Warn if outside buffer zone
    if ((at_min && pos[i] < min - BUFFER) || (at_max && pos[i] > max + BUFFER)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Out of workspace: %c = %.3f not in [%.3f, %.3f]",
                           "xyz"[i], pos[i], min, max);
    }
    
    // Prevent motion beyond limits
    if (at_min) V_tcp_base_commanded_[i] = std::max(0.0, V_tcp_base_commanded_[i]);
    if (at_max) V_tcp_base_commanded_[i] = std::min(0.0, V_tcp_base_commanded_[i]);
  }
  
  // Cap velocity magnitude
  if (auto n = V_tcp_base_commanded_.head<3>().norm(); n > arm_max_vel_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "High velocity: %.3f m/s", n);
    V_tcp_base_commanded_.head<3>() *= (arm_max_vel_ / n);
  }
}

}  // namespace ur_admittance_controller
