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
  pose_error_ = Vector6d::Zero();
  
  // Poses
  X_tcp_base_current_ = Eigen::Isometry3d::Identity();
  X_tcp_base_desired_ = Eigen::Isometry3d::Identity();
  
  // ROS message
  velocity_msg_.data.resize(joint_count);
  
  // Control limits
  workspace_limits_ << -1.0, 1.0, -1.0, 1.0, 0.0, 1.0;
  arm_max_vel_ = 1.5;
  arm_max_acc_ = 1.0;
  admittance_ratio_ = 1.0;
}

void AdmittanceNode::setDefaultEquilibrium() {
  declare_parameter("equilibrium.position", std::vector<double>{0.49, 0.13, 0.49});
  // Using positive-w convention for quaternion (w,x,y,z)
  declare_parameter("equilibrium.orientation", std::vector<double>{0.00, 0.71, -0.71, 0.00});
  
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
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
    "Admittance params - M: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f], "
    "K: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f], "
    "D: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
    p.mass[0], p.mass[1], p.mass[2], p.mass[3], p.mass[4], p.mass[5],
    p.stiffness[0], p.stiffness[1], p.stiffness[2], p.stiffness[3], p.stiffness[4], p.stiffness[5],
    p.damping[0], p.damping[1], p.damping[2], p.damping[3], p.damping[4], p.damping[5]);
}



void AdmittanceNode::get_X_tcp_base_current() {
  // Transfer joint positions to KDL format for FK computation
  for (size_t i = 0; i < num_joints_; ++i) {
    q_kdl_(i) = q_current_[i];
  }
  
  // Log initial joint configuration once for debugging
  RCLCPP_INFO_ONCE(get_logger(), "FK solver initialized - %zu joints: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
      num_joints_, q_current_[0], q_current_[1], q_current_[2], 
      q_current_[3], q_current_[4], q_current_[5]);
  
  // Step 1: Compute FK from base_link to wrist_3_link (last movable joint)
  if (fk_pos_solver_->JntToCart(q_kdl_, wrist3_frame_) < 0) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Forward kinematics solver failed");
    return;
  }
  
  // Step 2: Apply fixed tool offset to get actual TCP position
  KDL::Frame tool_frame = wrist3_frame_ * wrist3_to_tool_transform_;
  
  // Convert KDL frame to Eigen format for admittance calculations
  X_tcp_base_current_.translation() = Eigen::Vector3d(tool_frame.p.x(), tool_frame.p.y(), tool_frame.p.z());
  
  // Extract quaternion and convert to rotation matrix
  double x, y, z, w;
  tool_frame.M.GetQuaternion(x, y, z, w);
  X_tcp_base_current_.linear() = Eigen::Quaterniond(w, x, y, z).toRotationMatrix();
}

void AdmittanceNode::compute_pose_error() {
  // Use same convention as ur3_admittance_controller: current - desired
  pose_error_.head<3>() = X_tcp_base_current_.translation() - X_tcp_base_desired_.translation();
  
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
  pose_error_.tail<3>() = aa_error.axis() * aa_error.angle();
  
  // Log TCP error for debugging
  double position_error_norm = pose_error_.head<3>().norm();
  double orientation_error_norm = pose_error_.tail<3>().norm();
  
  RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
    "TCP Error - Position: %.4f m [%.3f, %.3f, %.3f], Orientation: %.4f rad [%.3f, %.3f, %.3f]",
    position_error_norm, pose_error_(0), pose_error_(1), pose_error_(2),
    orientation_error_norm, pose_error_(3), pose_error_(4), pose_error_(5));
}


void AdmittanceNode::compute_admittance() {
  // Scale external wrench by admittance ratio (0-1) for safety/tuning
  Vector6d scaled_wrench = admittance_ratio_ * Wrench_tcp_base_;
  
  // Core admittance equation: M*a + D*v + K*x = F_ext
  // Solving for acceleration: a = M^(-1) * (F_ext - D*v - K*x)
  Vector6d acceleration = M_inverse_diag_.array() *
      (scaled_wrench.array() - D_diag_.array() * V_tcp_base_commanded_.array() -
       K_diag_.array() * pose_error_.array());
  
  RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
    "Admittance accel: [%.3f, %.3f, %.3f] m/s², norm: %.3f",
    acceleration(0), acceleration(1), acceleration(2), acceleration.head<3>().norm());
  
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
  V_tcp_base_commanded_ += acceleration * dt;
  
  RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
    "TCP velocity cmd: [%.3f, %.3f, %.3f] m/s, norm: %.3f",
    V_tcp_base_commanded_(0), V_tcp_base_commanded_(1), V_tcp_base_commanded_(2),
    V_tcp_base_commanded_.head<3>().norm());
}

// Apply workspace limits to Cartesian motion
void AdmittanceNode::limit_to_workspace() {
  const auto& pos = X_tcp_base_current_.translation();
  
  // Check each axis (X,Y,Z) against configured workspace boundaries
  for (size_t i = 0; i < 3; ++i) {
    const double min = workspace_limits_[i * 2];
    const double max = workspace_limits_[i * 2 + 1];
    
    // Hard limit enforcement: only allow motion away from boundary
    if (pos[i] <= min) V_tcp_base_commanded_[i] = std::max(0.0, V_tcp_base_commanded_[i]);  // Block negative velocity
    if (pos[i] >= max) V_tcp_base_commanded_[i] = std::min(0.0, V_tcp_base_commanded_[i]);  // Block positive velocity
  }
  
  // Global velocity limit to prevent dangerous speeds
  double velocity_norm = V_tcp_base_commanded_.head<3>().norm();
  if (velocity_norm > arm_max_vel_) {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, 
                          "Capping TCP velocity: %.3f -> %.3f m/s", velocity_norm, arm_max_vel_);
    V_tcp_base_commanded_.head<3>() *= (arm_max_vel_ / velocity_norm);
  }
  
  // Global rotational velocity limit to prevent dangerous rotations
  double rot_velocity_norm = V_tcp_base_commanded_.tail<3>().norm();
  double arm_max_rot_vel_ = 3.0;  // rad/s - conservative limit for safety
  if (rot_velocity_norm > arm_max_rot_vel_) {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, 
                          "Capping TCP rotational velocity: %.3f -> %.3f rad/s", rot_velocity_norm, arm_max_rot_vel_);
    V_tcp_base_commanded_.tail<3>() *= (arm_max_rot_vel_ / rot_velocity_norm);
  }
}

void AdmittanceNode::compute_and_pub_joint_velocities() {
  // Pack TCP velocity
  KDL::Twist twist_tcp_base;
  twist_tcp_base.vel = KDL::Vector(V_tcp_base_commanded_(0), V_tcp_base_commanded_(1), V_tcp_base_commanded_(2));
  twist_tcp_base.rot = KDL::Vector(V_tcp_base_commanded_(3), V_tcp_base_commanded_(4), V_tcp_base_commanded_(5));
  
  // Get position offset using CACHED wrist3_frame (no FK needed!)
  KDL::Vector p_diff = KDL::Vector(
    X_tcp_base_current_.translation()(0) - wrist3_frame_.p.x(),
    X_tcp_base_current_.translation()(1) - wrist3_frame_.p.y(), 
    X_tcp_base_current_.translation()(2) - wrist3_frame_.p.z()
  );
  
  // Transform to wrist3 velocity
  KDL::Twist twist_wrist3_base;
  twist_wrist3_base.vel = twist_tcp_base.vel - twist_tcp_base.rot * p_diff;
  twist_wrist3_base.rot = twist_tcp_base.rot;
  
  // Solve IK
  if (ik_vel_solver_->CartToJnt(q_kdl_, twist_wrist3_base, v_kdl_) < 0) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "IK solver failed - executing safety stop");
    std::fill(q_dot_cmd_.begin(), q_dot_cmd_.end(), 0.0);
  } else {
    // Copy and check for NaN in one pass
    bool hasNaN = false;
    for (size_t i = 0; i < num_joints_; ++i) {
      q_dot_cmd_[i] = v_kdl_(i);
      hasNaN |= std::isnan(v_kdl_(i));
    }
    if (hasNaN) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "IK returned NaN - executing safety stop");
      std::fill(q_dot_cmd_.begin(), q_dot_cmd_.end(), 0.0);
    }
  }
  
  // Debug logging
  RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
    "Joint vel cmd: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f] rad/s",
    q_dot_cmd_[0], q_dot_cmd_[1], q_dot_cmd_[2], 
    q_dot_cmd_[3], q_dot_cmd_[4], q_dot_cmd_[5]);
  
  // Publish to hardware
  velocity_msg_.data = q_dot_cmd_;
  velocity_pub_->publish(velocity_msg_);
}

// Initialize KDL kinematics from URDF
bool AdmittanceNode::load_kinematics() {
  RCLCPP_INFO_ONCE(get_logger(), "Loading robot kinematics from URDF");
  
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
  
  RCLCPP_INFO_ONCE(get_logger(), "Tool offset: [%.3f, %.3f, %.3f] m",
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

  RCLCPP_INFO_ONCE(get_logger(), "Kinematics ready: %zu joints, %s -> %s",
              num_joints_, params_.base_link.c_str(), params_.tip_link.c_str());
  return true;
}


}  // namespace ur_admittance_controller
