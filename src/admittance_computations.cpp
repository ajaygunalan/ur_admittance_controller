#include "admittance_node.hpp"
#include <algorithm>
#include <cmath>
#include <rclcpp/parameter_client.hpp>
#include <sstream>
#include <iomanip>

namespace ur_admittance_controller {
  
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
  declare_parameter("equilibrium.position", std::vector<double>{0.49, 0.13, 0.49});
  declare_parameter("equilibrium.orientation", std::vector<double>{-0.00, -0.71, 0.71, 0.00});
  
  auto eq_pos = get_parameter("equilibrium.position").as_double_array();
  auto eq_ori = get_parameter("equilibrium.orientation").as_double_array();
  
  X_tcp_base_desired_.translation() << eq_pos[0], eq_pos[1], eq_pos[2];
  // Convert from WXYZ (parameter format) to Eigen's WXYZ constructor format
  X_tcp_base_desired_.linear() = Eigen::Quaterniond(eq_ori[0], eq_ori[1], eq_ori[2], eq_ori[3]).toRotationMatrix();
  
  RCLCPP_DEBUG(get_logger(), "Equilibrium pose set: position=[%.3f, %.3f, %.3f]", 
               eq_pos[0], eq_pos[1], eq_pos[2]);
}

void AdmittanceNode::update_admittance_parameters() {
  auto& p = params_.admittance;
  M_inverse_diag_ = Eigen::Map<const Eigen::VectorXd>(p.mass.data(), 6).cwiseInverse();
  K_diag_ = Eigen::Map<const Eigen::VectorXd>(p.stiffness.data(), 6);
  D_diag_ = Eigen::Map<const Eigen::VectorXd>(p.damping.data(), 6);
  
  // Log admittance parameters on startup/update
  RCLCPP_INFO(get_logger(), 
    "Admittance params - M: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f], "
    "K: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f], "
    "D: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
    p.mass[0], p.mass[1], p.mass[2], p.mass[3], p.mass[4], p.mass[5],
    p.stiffness[0], p.stiffness[1], p.stiffness[2], p.stiffness[3], p.stiffness[4], p.stiffness[5],
    p.damping[0], p.damping[1], p.damping[2], p.damping[3], p.damping[4], p.damping[5]);
}

Vector6d AdmittanceNode::compute_pose_error() {
  Vector6d error;
  // Use same convention as ur3_admittance_controller: current - desired
  error.head<3>() = X_tcp_base_current_.translation() - X_tcp_base_desired_.translation();
  
  Eigen::Quaterniond q_current(X_tcp_base_current_.rotation());
  Eigen::Quaterniond q_desired(X_tcp_base_desired_.rotation());
  
  // Ensure shortest path by checking dot product (flip desired if needed)
  if (q_current.dot(q_desired) < 0.0) {
    q_desired.coeffs() *= -1.0;
  }
  
  // Use same convention as ur3: q_error = q_current * q_desired^(-1)
  // This gives the rotation from desired to current orientation
  Eigen::Quaterniond q_error = q_current * q_desired.inverse();
  
  // Normalize for numerical stability using built-in method
  q_error.normalize();
  
  // Convert to axis-angle representation
  Eigen::AngleAxisd aa_error(q_error);
  error.tail<3>() = aa_error.axis() * aa_error.angle();
  
  // Log TCP error for debugging
  double position_error_norm = error.head<3>().norm();
  double orientation_error_norm = error.tail<3>().norm();
  
  RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
    "TCP Error - Position: %.4f m [%.3f, %.3f, %.3f], Orientation: %.4f rad [%.3f, %.3f, %.3f]",
    position_error_norm, error(0), error(1), error(2),
    orientation_error_norm, error(3), error(4), error(5));
  
  
  return error;
}

void AdmittanceNode::get_X_tcp_base_current_() {
  // Transfer joint positions to KDL format for FK computation
  for (size_t i = 0; i < num_joints_; ++i) {
    q_kdl_(i) = q_current_[i];
  }
  
  // Log initial joint configuration once for debugging
  RCLCPP_INFO_ONCE(get_logger(), "FK solver initialized - %zu joints: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
      num_joints_, q_current_[0], q_current_[1], q_current_[2], 
      q_current_[3], q_current_[4], q_current_[5]);
  
  // Step 1: Compute FK from base_link to wrist_3_link (last movable joint)
  KDL::Frame wrist3_frame;
  if (fk_pos_solver_->JntToCart(q_kdl_, wrist3_frame) < 0) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Forward kinematics solver failed");
    return;
  }
  
  // Step 2: Apply fixed tool offset to get actual TCP position
  KDL::Frame tool_frame = wrist3_frame * wrist3_to_tool_transform_;
  
  // Convert KDL frame to Eigen format for admittance calculations
  X_tcp_base_current_.translation() = Eigen::Vector3d(tool_frame.p.x(), tool_frame.p.y(), tool_frame.p.z());
  
  // Extract quaternion and convert to rotation matrix
  double x, y, z, w;
  tool_frame.M.GetQuaternion(x, y, z, w);
  X_tcp_base_current_.linear() = Eigen::Quaterniond(w, x, y, z).toRotationMatrix();
}

void AdmittanceNode::compute_admittance() {
  // Calculate position/orientation error between current and desired TCP pose
  Vector6d error = compute_pose_error();
  
  // Scale external wrench by admittance ratio (0-1) for safety/tuning
  Vector6d scaled_wrench = admittance_ratio_ * Wrench_tcp_base_;
  
  // Core admittance equation: M*a + D*v + K*x = F_ext
  // Solving for acceleration: a = M^(-1) * (F_ext - D*v - K*x)
  Vector6d acceleration = M_inverse_diag_.array() *
      (scaled_wrench.array() - D_diag_.array() * V_tcp_base_commanded_.array() -
       K_diag_.array() * error.array());
  
  RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
    "Admittance accel: [%.3f, %.3f, %.3f] m/s², norm: %.3f",
    acceleration(0), acceleration(1), acceleration(2), acceleration.head<3>().norm());
  
  // Safety limit: cap translational acceleration to prevent violent motions
  double acc_norm = acceleration.head<3>().norm();
  if (acc_norm > arm_max_acc_) {
    RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000,
                                "Capping high acceleration: " << acc_norm << " -> " << arm_max_acc_ << " m/s²");
    acceleration.head<3>() *= (arm_max_acc_ / acc_norm);
  }
  
  // Integrate acceleration to get velocity command (simple Euler integration)
  const double dt = control_period_.seconds();
  V_tcp_base_commanded_ += acceleration * dt;
  
  RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
    "TCP velocity cmd: [%.3f, %.3f, %.3f] m/s, norm: %.3f",
    V_tcp_base_commanded_(0), V_tcp_base_commanded_(1), V_tcp_base_commanded_(2),
    V_tcp_base_commanded_.head<3>().norm());
}

bool AdmittanceNode::compute_joint_velocities(const Vector6d& cart_vel) {
  // q_kdl_ already contains current joint angles from get_X_tcp_base_current_()
  
  // Pack Cartesian velocity (tool frame)
  KDL::Twist twist_tool;
  twist_tool.vel = KDL::Vector(cart_vel(0), cart_vel(1), cart_vel(2));
  twist_tool.rot = KDL::Vector(cart_vel(3), cart_vel(4), cart_vel(5));
  
  // Transform velocity from tool frame to wrist_3 frame using KDL's built-in adjoint
  KDL::Twist twist_wrist3 = wrist3_to_tool_transform_ * twist_tool;
  
  // Solve inverse kinematics for wrist_3 velocity
  if (ik_vel_solver_->CartToJnt(q_kdl_, twist_wrist3, v_kdl_) < 0) {
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

// Apply workspace limits to Cartesian motion
void AdmittanceNode::limit_to_workspace() {
  static constexpr double BUFFER = 0.01;  // 10mm warning zone beyond hard limits
  const auto& pos = X_tcp_base_current_.translation();
  
  // Check each axis (X,Y,Z) against configured workspace boundaries
  for (size_t i = 0; i < 3; ++i) {
    const double min = workspace_limits_[i * 2];
    const double max = workspace_limits_[i * 2 + 1];
    const bool at_min = pos[i] <= min;
    const bool at_max = pos[i] >= max;
    
    // Issue warning if we've exceeded the soft buffer zone
    if ((at_min && pos[i] < min - BUFFER) || (at_max && pos[i] > max + BUFFER)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "TCP outside workspace: %c-axis at %.3f m (limits: [%.3f, %.3f])",
                           "XYZ"[i], pos[i], min, max);
    }
    
    // Hard limit enforcement: only allow motion away from boundary
    if (at_min) V_tcp_base_commanded_[i] = std::max(0.0, V_tcp_base_commanded_[i]);  // Block negative velocity
    if (at_max) V_tcp_base_commanded_[i] = std::min(0.0, V_tcp_base_commanded_[i]);  // Block positive velocity
  }
  
  // Global velocity limit to prevent dangerous speeds
  double velocity_norm = V_tcp_base_commanded_.head<3>().norm();
  if (velocity_norm > arm_max_vel_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
                         "Capping TCP velocity: %.3f -> %.3f m/s", velocity_norm, arm_max_vel_);
    V_tcp_base_commanded_.head<3>() *= (arm_max_vel_ / velocity_norm);
  }
}

// Initialize KDL kinematics from URDF
bool AdmittanceNode::load_kinematics() {
  RCLCPP_INFO(get_logger(), "Loading robot kinematics from URDF");
  
  // Step 1: Get URDF from robot_state_publisher
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(
      this, "/robot_state_publisher");
  
  if (!parameters_client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(get_logger(), "Robot state publisher not found");
    return false;
  }
  
  auto parameters = parameters_client->get_parameters({"robot_description"});
  if (parameters.empty() || parameters[0].get_type() == rclcpp::PARAMETER_NOT_SET) {
    RCLCPP_ERROR(get_logger(), "robot_description parameter not found");
    return false;
  }
  std::string urdf_string = parameters[0].as_string();
  
  // Step 2: Build KDL tree from URDF
  if (!kdl_parser::treeFromString(urdf_string, kdl_tree_)) {
    RCLCPP_ERROR(get_logger(), "Invalid URDF format");
    return false;
  }

  // Step 3: Extract robot arm chain (base → wrist_3 = 6 movable joints)
  if (!kdl_tree_.getChain(params_.base_link, "wrist_3_link", kdl_chain_)) {
    RCLCPP_ERROR(get_logger(), "Cannot find chain: %s -> wrist_3_link", params_.base_link.c_str());
    return false;
  }
  
  // Step 4: Compute tool offset transform (wrist_3 → tool = fixed transform)
  KDL::Chain tool_chain;
  if (!kdl_tree_.getChain("wrist_3_link", params_.tip_link, tool_chain)) {
    RCLCPP_ERROR(get_logger(), "Cannot find tool: wrist_3_link -> %s", params_.tip_link.c_str());
    return false;
  }
  
  KDL::ChainFkSolverPos_recursive tool_fk(tool_chain);
  KDL::JntArray zero_joints(tool_chain.getNrOfJoints());  // 0 joints for fixed transform
  tool_fk.JntToCart(zero_joints, wrist3_to_tool_transform_);
  
  RCLCPP_INFO(get_logger(), "Tool offset: [%.3f, %.3f, %.3f] m",
              wrist3_to_tool_transform_.p.x(), 
              wrist3_to_tool_transform_.p.y(), 
              wrist3_to_tool_transform_.p.z());

  // Step 5: Create solvers and allocate working arrays
  constexpr double PRECISION = 1e-5;
  constexpr int MAX_ITERATIONS = 150;
  constexpr double DAMPING = 0.01;
  
  // Create velocity IK solver with damping
  ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_wdls>(kdl_chain_, PRECISION, MAX_ITERATIONS);
  ik_vel_solver_->setLambda(DAMPING);
  
  // Create forward kinematics solver
  fk_pos_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);

  // Allocate working arrays
  num_joints_ = kdl_chain_.getNrOfJoints();
  q_kdl_.resize(num_joints_);
  v_kdl_.resize(num_joints_);

  RCLCPP_INFO(get_logger(), "Kinematics ready: %zu joints, %s -> %s",
              num_joints_, params_.base_link.c_str(), params_.tip_link.c_str());
  return true;
}

}  // namespace ur_admittance_controller
