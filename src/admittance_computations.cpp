#include "admittance_node.hpp"
#include "ur_admittance_controller/error.hpp"
#include <fmt/core.h>
#include <fmt/ranges.h>
#include <algorithm>
#include <cmath>
#include <rclcpp/parameter_client.hpp>

namespace ur_admittance_controller {
  
void AdmittanceNode::initializeStateVectors() {
  const auto joint_count = params_.joints.size();
  
  // Joint space vectors
  q_current_.resize(joint_count, 0.0);
  q_dot_cmd_.resize(joint_count, 0.0);
  
  // Cartesian space vectors
  F_P_B = Vector6d::Zero();
  V_P_B_commanded = Vector6d::Zero();
  X_BP_error = Vector6d::Zero();
  
  // Poses
  X_BP_current = Eigen::Isometry3d::Identity();
  X_BP_desired = Eigen::Isometry3d::Identity();
  
  // ROS message
  velocity_msg_.data.resize(joint_count);
  
  // Control limits
  workspace_limits_ << -1.0, 1.0, -1.0, 1.0, 0.0, 1.0;
  arm_max_vel_ = 1.5;
  arm_max_acc_ = 1.0;
  admittance_ratio_ = 1.0;
}

void AdmittanceNode::setDefaultEquilibrium() {
  // Declare parameters with fallback defaults
  declare_parameter("equilibrium.position", std::vector<double>{0.49, 0.13, 0.49});
  // Using positive-w convention for quaternion (w,x,y,z)
  declare_parameter("equilibrium.orientation", std::vector<double>{0.00, 0.71, -0.71, 0.00});
  
  // Get parameters - will use YAML values if loaded with --params-file
  auto eq_pos = get_parameter("equilibrium.position").as_double_array();
  auto eq_ori = get_parameter("equilibrium.orientation").as_double_array();
  
  X_BP_desired.translation() << eq_pos[0], eq_pos[1], eq_pos[2];
  // Convert from WXYZ (parameter format) to Eigen's WXYZ constructor format
  X_BP_desired.linear() = Eigen::Quaterniond(eq_ori[0], eq_ori[1], eq_ori[2], eq_ori[3]).toRotationMatrix();
  
  RCLCPP_INFO(get_logger(), "Equilibrium pose set: position=[%.3f, %.3f, %.3f], orientation=[%.3f, %.3f, %.3f, %.3f]", 
               eq_pos[0], eq_pos[1], eq_pos[2],
               eq_ori[0], eq_ori[1], eq_ori[2], eq_ori[3]);
}

void AdmittanceNode::update_admittance_parameters() {
  auto& p = params_.admittance;
  
  // Tier 1: Grouped invariant checks for params
  ENSURE(p.mass.size() == 6 && p.stiffness.size() == 6 && p.damping.size() == 6,
         "Admittance parameter vectors must all have exactly 6 elements");
  ENSURE((Eigen::Map<const Eigen::VectorXd>(p.mass.data(), 6).array() > 0).all(),
         "All mass values must be positive to avoid division by zero");
  
  M_inverse_diag = Eigen::Map<const Eigen::VectorXd>(p.mass.data(), 6).cwiseInverse();
  K_diag = Eigen::Map<const Eigen::VectorXd>(p.stiffness.data(), 6);
  D_diag = Eigen::Map<const Eigen::VectorXd>(p.damping.data(), 6);
  
  // Log admittance parameters on startup/update
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
    "Admittance params - M: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f], "
    "K: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f], "
    "D: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
    p.mass[0], p.mass[1], p.mass[2], p.mass[3], p.mass[4], p.mass[5],
    p.stiffness[0], p.stiffness[1], p.stiffness[2], p.stiffness[3], p.stiffness[4], p.stiffness[5],
    p.damping[0], p.damping[1], p.damping[2], p.damping[3], p.damping[4], p.damping[5]);
}



Status AdmittanceNode::get_X_BP_current() {
  // Tier 1: Single grouped invariant check
  ENSURE(num_joints_ == 6 && q_current_.size() == num_joints_ && q_kdl_.rows() == num_joints_ &&
         fk_pos_solver_ != nullptr,
         "FK preconditions violated: joints must be 6 and solver initialized");
  
  // Transfer joint positions to KDL format for FK computation
  for (size_t i = 0; i < num_joints_; ++i) {
    q_kdl_(i) = q_current_[i];
  }
  
  // Log initial joint configuration once for debugging
  RCLCPP_INFO_ONCE(get_logger(), "FK solver initialized - %zu joints: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
      num_joints_, q_current_[0], q_current_[1], q_current_[2], 
      q_current_[3], q_current_[4], q_current_[5]);
  
  // Step 1: Compute FK from base_link to wrist_3_link (last movable joint)
  if (fk_pos_solver_->JntToCart(q_kdl_, X_BW3) < 0) {
    auto msg = fmt::format("FK failed at q=[{}] rad", 
                          fmt::join(q_current_, ", "));
    
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "%s", msg.c_str());
    return tl::unexpected(make_error(ErrorCode::kKinematicsInitFailed, msg));
  }
  
  // Step 2: Apply fixed tool offset to get actual payload position
  KDL::Frame X_BP = X_BW3 * X_W3P;
  
  // Convert KDL frame to Eigen format for admittance calculations
  X_BP_current.translation() = Eigen::Vector3d(X_BP.p.x(), X_BP.p.y(), X_BP.p.z());
  
  // Extract quaternion and convert to rotation matrix
  double x, y, z, w;
  X_BP.M.GetQuaternion(x, y, z, w);
  X_BP_current.linear() = Eigen::Quaterniond(w, x, y, z).toRotationMatrix();
  
  return {};  // Success
}

void AdmittanceNode::compute_pose_error() {
  // Use same convention as ur3_admittance_controller: current - desired
  X_BP_error.head<3>() = X_BP_current.translation() - X_BP_desired.translation();
  
  Eigen::Quaterniond q_current(X_BP_current.rotation());
  Eigen::Quaterniond q_desired(X_BP_desired.rotation());
  
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
  X_BP_error.tail<3>() = aa_error.axis() * aa_error.angle();
  
  // Log payload error for debugging
  double position_error_norm = X_BP_error.head<3>().norm();
  double orientation_error_norm = X_BP_error.tail<3>().norm();
  
  RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
    "Payload Error - Position: %.4f m [%.3f, %.3f, %.3f], Orientation: %.4f rad [%.3f, %.3f, %.3f]",
    position_error_norm, X_BP_error(0), X_BP_error(1), X_BP_error(2),
    orientation_error_norm, X_BP_error(3), X_BP_error(4), X_BP_error(5));
}


void AdmittanceNode::compute_admittance() {
  // Tier 1: Single grouped invariant for all preconditions
  ENSURE(M_inverse_diag.size() == 6 && D_diag.size() == 6 && K_diag.size() == 6 &&
         !F_P_B.hasNaN() && !V_P_B_commanded.hasNaN() && !X_BP_error.hasNaN(),
         "Admittance computation preconditions violated");
  
  // Scale external wrench by admittance ratio (0-1) for safety/tuning
  Vector6d scaled_wrench = admittance_ratio_ * F_P_B;
  
  // Core admittance equation: M*a + D*v + K*x = F_ext
  // Solving for acceleration: a = M^(-1) * (F_ext - D*v - K*x)
  Vector6d acceleration = M_inverse_diag.array() *
      (scaled_wrench.array() - D_diag.array() * V_P_B_commanded.array() -
       K_diag.array() * X_BP_error.array());
  
  // Debug the actual equation components
  RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
    "Admittance calc: F=[%.2f,%.2f,%.2f] - D*v=[%.2f,%.2f,%.2f] - K*x=[%.2f,%.2f,%.2f] => a=[%.3f,%.3f,%.3f]",
    scaled_wrench(0), scaled_wrench(1), scaled_wrench(2),
    D_diag(0)*V_P_B_commanded(0), D_diag(1)*V_P_B_commanded(1), D_diag(2)*V_P_B_commanded(2),
    K_diag(0)*X_BP_error(0), K_diag(1)*X_BP_error(1), K_diag(2)*X_BP_error(2),
    acceleration(0), acceleration(1), acceleration(2));
  
  // Safety limit: cap translational acceleration to prevent violent motions
  double acc_norm = acceleration.head<3>().norm();
  if (acc_norm > arm_max_acc_) {
    RCLCPP_DEBUG_STREAM_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "Capping high acceleration: " << acc_norm << " -> " << arm_max_acc_ << " m/s²");
    acceleration.head<3>() *= (arm_max_acc_ / acc_norm);
  }
  
  // Safety limit: cap rotational acceleration to prevent violent rotations
  double rot_acc_norm = acceleration.tail<3>().norm();
  double arm_max_rot_acc_ = 2.0;  // rad/s² - conservative limit for safety
  if (rot_acc_norm > arm_max_rot_acc_) {
    RCLCPP_DEBUG_STREAM_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "Capping high rotational acceleration: " << rot_acc_norm << " -> " << arm_max_rot_acc_ << " rad/s²");
    acceleration.tail<3>() *= (arm_max_rot_acc_ / rot_acc_norm);
  }
  
  // Integrate acceleration to get velocity command (simple Euler integration)
  const double dt = control_period_.seconds();
  V_P_B_commanded += acceleration * dt;
  
  RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
    "Payload vel cmd: [%.3f,%.3f,%.3f] m/s (err=[%.3f,%.3f,%.3f])",
    V_P_B_commanded(0), V_P_B_commanded(1), V_P_B_commanded(2),
    X_BP_error(0), X_BP_error(1), X_BP_error(2));
}

// Apply workspace limits to Cartesian motion
void AdmittanceNode::limit_to_workspace() {
  const auto& pos = X_BP_current.translation();
  Vector6d vel_before = V_P_B_commanded;
  
  // Check each axis (X,Y,Z) against configured workspace boundaries
  for (size_t i = 0; i < 3; ++i) {
    const double min = workspace_limits_[i * 2];
    const double max = workspace_limits_[i * 2 + 1];
    
    // Hard limit enforcement: only allow motion away from boundary
    if (pos[i] <= min) V_P_B_commanded[i] = std::max(0.0, V_P_B_commanded[i]);  // Block negative velocity
    if (pos[i] >= max) V_P_B_commanded[i] = std::min(0.0, V_P_B_commanded[i]);  // Block positive velocity
  }
  
  // Debug if velocity was modified
  if ((vel_before.head<3>() - V_P_B_commanded.head<3>()).norm() > 0.001) {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
      "Workspace limits: vel [%.3f,%.3f,%.3f] -> [%.3f,%.3f,%.3f], pos=[%.3f,%.3f,%.3f]",
      vel_before(0), vel_before(1), vel_before(2),
      V_P_B_commanded(0), V_P_B_commanded(1), V_P_B_commanded(2),
      pos(0), pos(1), pos(2));
  }
  
  // Global velocity limit to prevent dangerous speeds
  double velocity_norm = V_P_B_commanded.head<3>().norm();
  if (velocity_norm > arm_max_vel_) {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, 
                          "Capping payload velocity: %.3f -> %.3f m/s", velocity_norm, arm_max_vel_);
    V_P_B_commanded.head<3>() *= (arm_max_vel_ / velocity_norm);
  }
  
  // Global rotational velocity limit to prevent dangerous rotations
  double rot_velocity_norm = V_P_B_commanded.tail<3>().norm();
  double arm_max_rot_vel_ = 3.0;  // rad/s - conservative limit for safety
  if (rot_velocity_norm > arm_max_rot_vel_) {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, 
                          "Capping payload rotational velocity: %.3f -> %.3f rad/s", rot_velocity_norm, arm_max_rot_vel_);
    V_P_B_commanded.tail<3>() *= (arm_max_rot_vel_ / rot_velocity_norm);
  }
}

Status AdmittanceNode::compute_and_pub_joint_velocities() {
  // Tier 1: Single grouped invariant for all IK preconditions
  ENSURE(ik_vel_solver_ != nullptr && V_P_B_commanded.size() == 6 && !V_P_B_commanded.hasNaN() &&
         q_kdl_.rows() == num_joints_,
         "IK preconditions violated: solver, velocity, or joint arrays invalid");
  
  // Pack payload velocity
  KDL::Twist V_P_B;
  V_P_B.vel = KDL::Vector(V_P_B_commanded(0), V_P_B_commanded(1), V_P_B_commanded(2));
  V_P_B.rot = KDL::Vector(V_P_B_commanded(3), V_P_B_commanded(4), V_P_B_commanded(5));
  
  // Get position offset using CACHED X_BW3 transform (no FK needed!)
  KDL::Vector p_PW3_B = KDL::Vector(
    X_BP_current.translation()(0) - X_BW3.p.x(),
    X_BP_current.translation()(1) - X_BW3.p.y(), 
    X_BP_current.translation()(2) - X_BW3.p.z()
  );
  
  // Transform to wrist3 velocity
  KDL::Twist V_W3_B;
  V_W3_B.vel = V_P_B.vel - V_P_B.rot * p_PW3_B;
  V_W3_B.rot = V_P_B.rot;
  
  // Solve IK
  Status ik_status;
  if (ik_vel_solver_->CartToJnt(q_kdl_, V_W3_B, v_kdl_) < 0) {
    auto msg = fmt::format(
        "IK failed: cart-vel=[{:.3f}, {:.3f}, {:.3f}] m/s · "
        "[{:.3f}, {:.3f}, {:.3f}] rad/s at q=[{}] rad",
        V_P_B.vel.x(), V_P_B.vel.y(), V_P_B.vel.z(),
        V_P_B.rot.x(), V_P_B.rot.y(), V_P_B.rot.z(),
        fmt::join(q_current_, ", "));
    
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "%s - executing safety stop", msg.c_str());
    std::fill(q_dot_cmd_.begin(), q_dot_cmd_.end(), 0.0);
    ik_status = tl::unexpected(make_error(ErrorCode::kIKSolverFailed, msg));
  } else {
    // Copy and check for NaN in one pass
    bool hasNaN = false;
    for (size_t i = 0; i < num_joints_; ++i) {
      q_dot_cmd_[i] = v_kdl_(i);
      hasNaN |= std::isnan(v_kdl_(i));
    }
    if (hasNaN) {
      // Create vector of velocity strings, marking NaN values
      std::vector<std::string> vel_strs;
      for (size_t i = 0; i < num_joints_; ++i) {
        vel_strs.push_back(std::isnan(v_kdl_(i)) ? "NaN" : fmt::format("{:.3f}", v_kdl_(i)));
      }
      
      auto msg = fmt::format("IK returned NaN for joint velocities: [{}] rad/s",
                            fmt::join(vel_strs, ", "));
      
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "%s - executing safety stop", msg.c_str());
      std::fill(q_dot_cmd_.begin(), q_dot_cmd_.end(), 0.0);
      ik_status = tl::unexpected(make_error(ErrorCode::kIKSolverFailed, msg));
    }
  }
  
  // Debug IK input/output
  RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
    "IK: Payload vel [%.3f,%.3f,%.3f] m/s -> Joint vel [%.3f,%.3f,%.3f,%.3f,%.3f,%.3f] rad/s",
    V_P_B.vel.x(), V_P_B.vel.y(), V_P_B.vel.z(),
    q_dot_cmd_[0], q_dot_cmd_[1], q_dot_cmd_[2], q_dot_cmd_[3], q_dot_cmd_[4], q_dot_cmd_[5]);
  
  // Always publish velocities (zero on error for safety)
  velocity_msg_.data = q_dot_cmd_;
  velocity_pub_->publish(velocity_msg_);
  
  return ik_status.has_value() ? Status{} : ik_status;
}

// Initialize KDL kinematics from URDF
Status AdmittanceNode::load_kinematics() {
  RCLCPP_INFO_ONCE(get_logger(), "Loading robot kinematics from URDF");
  
  // Step 1: Get URDF from robot_state_publisher
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(
      this, "/robot_state_publisher");
  
  if (!parameters_client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(get_logger(), "Robot state publisher not found");
    return tl::unexpected(make_error(ErrorCode::kKinematicsInitFailed,
                                   "Robot state publisher service not available after 5 seconds"));
  }
  
  auto parameters = parameters_client->get_parameters({"robot_description"});
  if (parameters.empty() || parameters[0].get_type() == rclcpp::PARAMETER_NOT_SET) {
    RCLCPP_ERROR(get_logger(), "robot_description parameter not found");
    return tl::unexpected(make_error(ErrorCode::kKinematicsInitFailed,
                                   "robot_description parameter not set in robot_state_publisher"));
  }
  std::string urdf_string = parameters[0].as_string();
  
  // Step 2: Build KDL tree from URDF
  if (!kdl_parser::treeFromString(urdf_string, kdl_tree_)) {
    RCLCPP_ERROR(get_logger(), "Invalid URDF format");
    return tl::unexpected(make_error(ErrorCode::kKinematicsInitFailed,
                                   "Failed to parse URDF into KDL tree"));
  }

  // Step 3: Extract robot arm chain (base → wrist_3 = 6 movable joints)
  if (!kdl_tree_.getChain(params_.base_link, "wrist_3_link", kdl_chain_)) {
    RCLCPP_ERROR(get_logger(), "Cannot find chain: %s -> wrist_3_link", params_.base_link.c_str());
    return tl::unexpected(make_error(ErrorCode::kKinematicsInitFailed,
                                   "Cannot extract kinematic chain from " + params_.base_link + " to wrist_3_link"));
  }
  
  // Step 4: Compute tool offset transform (wrist_3 → payload = fixed transform)
  KDL::Chain tool_chain;
  if (!kdl_tree_.getChain("wrist_3_link", params_.tip_link, tool_chain)) {
    RCLCPP_ERROR(get_logger(), "Cannot find payload: wrist_3_link -> %s", params_.tip_link.c_str());
    return tl::unexpected(make_error(ErrorCode::kKinematicsInitFailed,
                                   "Cannot extract tool chain from wrist_3_link to " + params_.tip_link));
  }
  
  KDL::ChainFkSolverPos_recursive tool_fk(tool_chain);
  KDL::JntArray zero_joints(tool_chain.getNrOfJoints());  // 0 joints for fixed transform
  tool_fk.JntToCart(zero_joints, X_W3P);
  
  RCLCPP_INFO_ONCE(get_logger(), "Payload offset: [%.3f, %.3f, %.3f] m",
              X_W3P.p.x(), 
              X_W3P.p.y(), 
              X_W3P.p.z());

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
  
  // Tier 1: UR-specific invariant
  ENSURE(num_joints_ == 6, "UR robot kinematic chain must have exactly 6 joints, got " + std::to_string(num_joints_));
  
  q_kdl_.resize(num_joints_);
  v_kdl_.resize(num_joints_);

  RCLCPP_INFO_ONCE(get_logger(), "Kinematics ready: %zu joints, %s -> %s",
              num_joints_, params_.base_link.c_str(), params_.tip_link.c_str());
  return {};  // Success
}


}  // namespace ur_admittance_controller
