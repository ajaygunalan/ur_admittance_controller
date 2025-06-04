#include "admittance_node.hpp"
#include "admittance_constants.hpp"
#include <algorithm>
#include <cmath>

namespace ur_admittance_controller {

using namespace constants;

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
  using namespace constants;

  for (size_t i = 0; i < 6; ++i) {
    const double effective_stiffness = (params_.admittance.stiffness[i] <= 0.0)
                                           ? VIRTUAL_STIFFNESS
                                           : params_.admittance.stiffness[i];

    const double damping_value = 2.0 * params_.admittance.damping_ratio[i] *
                                 std::sqrt(params_.admittance.mass[i] * effective_stiffness);

    D_diag_(i) = damping_value;
  }
}

void AdmittanceNode::update_admittance_parameters() {
  update_mass_matrix();
  update_stiffness_matrix();
  update_damping_matrix();
}

bool AdmittanceNode::compute_joint_velocities(const Vector6d& cart_vel) {
  if (cart_vel.hasNaN()) {
    RCLCPP_ERROR(get_logger(), "NaN detected in Cartesian velocity command");
    return false;
  }
  
  for (size_t i = 0; i < num_joints_; ++i) {
    q_kdl_(i) = q_current_[i];
  }

  KDL::Twist cart_twist;
  cart_twist.vel.x(cart_vel(0));
  cart_twist.vel.y(cart_vel(1));
  cart_twist.vel.z(cart_vel(2));
  cart_twist.rot.x(cart_vel(3));
  cart_twist.rot.y(cart_vel(4));
  cart_twist.rot.z(cart_vel(5));

  if (ik_vel_solver_->CartToJnt(q_kdl_, cart_twist, v_kdl_) < 0) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "KDL IK velocity solver failed");
    return false;
  }

  for (size_t i = 0; i < num_joints_; ++i) {
    q_dot_cmd_[i] = v_kdl_(i);
    if (std::isnan(q_dot_cmd_[i])) return false;
  }
  
  return true;
}

void AdmittanceNode::computeForwardKinematics() {
  if (!joint_states_updated_) {
    return;
  }
  
  if (!fk_pos_solver_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Forward kinematics solver not initialized");
    return;
  }
  
  for (size_t i = 0; i < num_joints_; ++i) {
    q_kdl_(i) = q_current_[i];
  }
  
  KDL::Frame tcp_frame;
  int fk_result = fk_pos_solver_->JntToCart(q_kdl_, tcp_frame);
  
  if (fk_result < 0) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                          "Forward kinematics computation failed with error: %d", fk_result);
    return;
  }
  
  X_tcp_base_current_.translation() << tcp_frame.p.x(), 
                                       tcp_frame.p.y(), 
                                       tcp_frame.p.z();
  
  double x, y, z, w;
  tcp_frame.M.GetQuaternion(x, y, z, w);
  Eigen::Quaterniond q(w, x, y, z);
  X_tcp_base_current_.linear() = q.toRotationMatrix();
  
  joint_states_updated_ = false;
  
  static rclcpp::Time last_log_time = get_clock()->now();
  if ((get_clock()->now() - last_log_time).seconds() > 5.0) {
    RCLCPP_DEBUG(get_logger(), "Forward kinematics computed successfully");
    last_log_time = get_clock()->now();
  }
}


}  // namespace ur_admittance_controller
