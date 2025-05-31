// Force/torque sensor and transform management implementation

#include "admittance_node.hpp"

namespace ur_admittance_controller {


// Update force/torque sensor data with filtering and transform
bool AdmittanceNode::updateSensorData()
{
  Vector6d raw_wrench = Vector6d::Zero();
  
  // Get current wrench from callback data
  {
    std::lock_guard<std::mutex> lock(wrench_mutex_);
    raw_wrench(0) = current_wrench_.wrench.force.x;
    raw_wrench(1) = current_wrench_.wrench.force.y;
    raw_wrench(2) = current_wrench_.wrench.force.z;
    raw_wrench(3) = current_wrench_.wrench.torque.x;
    raw_wrench(4) = current_wrench_.wrench.torque.y;
    raw_wrench(5) = current_wrench_.wrench.torque.z;
  }
  
  // Transform to base frame using direct tf2 lookup
  F_sensor_base_ = transformWrench(raw_wrench);
  
  // Apply EMA filter - validation now handled by generate_parameter_library
  wrench_filtered_ = params_.admittance.filter_coefficient * F_sensor_base_ + 
    (1.0 - params_.admittance.filter_coefficient) * wrench_filtered_;
  
  if (wrench_filtered_.hasNaN()) {
    RCLCPP_ERROR(get_logger(), "NaN detected in filtered wrench");
    return false;
  }
  
  return checkDeadband();
}

bool AdmittanceNode::updateTransforms()
{
  // Get current end-effector pose using direct tf2 lookup
  return getCurrentEndEffectorPose(X_base_tip_current_);
}


Vector6d AdmittanceNode::transformWrench(const Vector6d& wrench_sensor_frame)
{
  if (params_.ft_frame == params_.base_link) {
    return wrench_sensor_frame;  // No transform needed
  }
  
  try {
    auto transform = tf_buffer_->lookupTransform(
      params_.base_link, 
      params_.ft_frame,
      tf2::TimePointZero,  // Latest available
      std::chrono::milliseconds(50));  // 50ms timeout
    
    // Compute adjoint matrix for wrench transformation
    Eigen::Isometry3d T = tf2::transformToEigen(transform);
    Eigen::Matrix3d R = T.rotation();
    Eigen::Vector3d t = T.translation();
    
    Matrix6d adjoint = Matrix6d::Zero();
    adjoint.block<3, 3>(0, 0) = R;
    adjoint.block<3, 3>(3, 3) = R;
    
    Eigen::Matrix3d t_cross;
    t_cross << 0, -t.z(), t.y(),
               t.z(), 0, -t.x(),
               -t.y(), t.x(), 0;
    adjoint.block<3, 3>(3, 0) = t_cross * R;
    
    return adjoint * wrench_sensor_frame;
    
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "F/T transform lookup failed: %s, using sensor frame data", ex.what());
    return wrench_sensor_frame;  // Fallback
  }
}

bool AdmittanceNode::getCurrentEndEffectorPose(Eigen::Isometry3d& pose)
{
  try {
    auto transform = tf_buffer_->lookupTransform(
      params_.base_link,
      params_.tip_link, 
      tf2::TimePointZero,
      std::chrono::milliseconds(50));
    
    pose = tf2::transformToEigen(transform);
    return true;
    
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "End-effector transform lookup failed: %s", ex.what());
    return false;
  }
}


// Check if forces exceed deadband threshold
bool AdmittanceNode::checkDeadband()
{
  // Check if any force/torque exceeds threshold
  for (size_t i = 0; i < 6; ++i) {
    if (std::abs(wrench_filtered_(i)) > params_.admittance.min_motion_threshold) {
      return true;
    }
  }
  
  // All forces below threshold - stop motion
  V_base_tip_base_.setZero();
  
  // Update references to hold current position
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    joint_position_references_ = joint_positions_;
  }
  return false;
}

// Publish Cartesian velocity for monitoring
void AdmittanceNode::publishCartesianVelocity()
{
  // Use pre-allocated message for better performance
  cart_vel_msg_.linear.x = V_base_tip_base_(0);
  cart_vel_msg_.linear.y = V_base_tip_base_(1);
  cart_vel_msg_.linear.z = V_base_tip_base_(2);
  cart_vel_msg_.angular.x = V_base_tip_base_(3);
  cart_vel_msg_.angular.y = V_base_tip_base_(4);
  cart_vel_msg_.angular.z = V_base_tip_base_(5);
  cart_vel_pub_->publish(cart_vel_msg_);
}

}  // namespace ur_admittance_controller
