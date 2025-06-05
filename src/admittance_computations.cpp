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



void AdmittanceNode::update_mass_matrix() {
  M_inverse_diag_ = Eigen::Map<const Eigen::VectorXd>(params_.admittance.mass.data(), 6).cwiseInverse();
}

void AdmittanceNode::update_stiffness_matrix() {
  for (size_t i = 0; i < 6; ++i) {
    K_diag_(i) = params_.admittance.stiffness[i];
  }
}

void AdmittanceNode::update_damping_matrix() {
  for (size_t i = 0; i < 6; ++i) {
    D_diag_(i) = params_.admittance.damping[i];
  }
}

void AdmittanceNode::update_admittance_parameters() {
  update_mass_matrix();
  update_stiffness_matrix();
  update_damping_matrix();
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


}  // namespace ur_admittance_controller
