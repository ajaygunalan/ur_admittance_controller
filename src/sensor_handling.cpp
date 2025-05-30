/**
 * @file sensor_interface.cpp
 * @brief Force/torque sensor and transform management implementation
 *
 * This file handles sensor data acquisition, filtering, and coordinate
 * frame transformations for the admittance controller.
 */

#include "admittance_node.hpp"
#include <memory>

namespace ur_admittance_controller {

constexpr double CACHE_VALIDITY_WARNING_TIME = 0.5;

/**
 * @brief Update force/torque sensor data with filtering and transform
 *
 * This method:
 * 1. Reads raw F/T sensor data from hardware interfaces
 * 2. Transforms the wrench to base frame if transform available
 * 3. Applies low-pass filtering
 * 4. Checks deadband
 *
 * @return true if sensor update successful
 */
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
  
  // Transform to base frame if needed
  if (!updateTransforms()) {
    F_sensor_base_ = raw_wrench;  // Fallback to sensor frame
  } else if (transform_base_ft_.isValid()) {
    const auto& transform_data = transform_base_ft_.getTransform();
    F_sensor_base_ = transform_data.adjoint * raw_wrench;
  } else {
    F_sensor_base_ = raw_wrench;
  }
  
  // Common filtering and deadband check
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
  
  if (transform_base_tip_.isValid()) {
    const auto& ee_data = transform_base_tip_.getTransform();
    X_base_tip_current_ = tf2::transformToEigen(ee_data.transform);
  }
  
  auto now = get_clock()->now();
  if ((now - transform_base_ft_.last_update).seconds() > TRANSFORM_TIMEOUT) {
    transform_update_needed_.store(true);
  }
  
  if (!transform_base_ft_.isValid()) {
    return false;
  }
  
  if (transform_base_tip_.isValid()) {
    const auto& ee_data = transform_base_tip_.getTransform();
    X_base_tip_current_ = tf2::transformToEigen(ee_data.transform);
  }
  
  if ((now - transform_base_ft_.last_update).seconds() > CACHE_VALIDITY_WARNING_TIME) {
    transform_update_needed_.store(true);
  }
  
  return true;
}

bool AdmittanceNode::updateTransform_base_tip()
{
  auto now = get_clock()->now();
  
  try {
    if (transform_base_tip_.target_frame.empty()) {
      transform_base_tip_.target_frame = params_.base_link;
      transform_base_tip_.source_frame = params_.tip_link;
    }
    
    geometry_msgs::msg::TransformStamped T_base_tip_msg = 
      tf_buffer_->lookupTransform(
        params_.base_link,
        params_.tip_link,
        tf2_ros::fromMsg(now), tf2::durationFromSec(0.1));
    
    Matrix6d identity_adjoint = Matrix6d::Identity();
    transform_base_tip_.updateTransform(T_base_tip_msg, identity_adjoint, now);
    
    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Tip transform lookup failed: %s", ex.what());
    return false;
  }
}

bool AdmittanceNode::updateTransform_base_ft()
{
  auto now = get_clock()->now();
  
  try {
    if (transform_base_ft_.target_frame.empty()) {
      transform_base_ft_.target_frame = params_.base_link;
      transform_base_ft_.source_frame = params_.ft_frame;
    }
    
    geometry_msgs::msg::TransformStamped T_base_ft_msg = 
      tf_buffer_->lookupTransform(
        params_.base_link,
        params_.ft_frame,
        tf2_ros::fromMsg(now), tf2::durationFromSec(0.1));
    
    Eigen::Matrix3d R = tf2::transformToEigen(T_base_ft_msg).rotation();
    Eigen::Vector3d t = tf2::transformToEigen(T_base_ft_msg).translation();
    
    Matrix6d adjoint = Matrix6d::Zero();
    
    adjoint.block<3, 3>(0, 0) = R;
    
    adjoint.block<3, 3>(3, 3) = R;
    
    Eigen::Matrix3d t_cross;
    t_cross << 0, -t.z(), t.y(),
               t.z(), 0, -t.x(),
               -t.y(), t.x(), 0;
    adjoint.block<3, 3>(3, 0) = t_cross * R;
    
    transform_base_ft_.updateTransform(T_base_ft_msg, adjoint, now);
    
    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "F/T transform lookup failed: %s", ex.what());
    return false;
  }
}

void AdmittanceNode::updateTransformCaches()
{
  bool tip_updated = updateTransform_base_tip();
  bool ft_updated = updateTransform_base_ft();
  bool all_updated = tip_updated && ft_updated;
  
  transform_update_needed_.store(!all_updated);
}

void AdmittanceNode::updateEETransformOnly()
{
  try {
    auto now = get_clock()->now();
    
    if (transform_base_tip_.target_frame.empty()) {
      transform_base_tip_.target_frame = params_.base_link;
      transform_base_tip_.source_frame = params_.tip_link;
    }
    
    auto ee_transform = tf_buffer_->lookupTransform(
      transform_base_tip_.target_frame, 
      transform_base_tip_.source_frame,
      tf2_ros::fromMsg(now), tf2::durationFromSec(0.1));
    
    Matrix6d identity_adjoint = Matrix6d::Identity();
    transform_base_tip_.updateTransform(ee_transform, identity_adjoint, now);
    
    transform_update_needed_.store(false);
    
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "EE transform lookup failed: %s", ex.what());
    transform_update_needed_.store(true);
  }
}


/**
 * @brief Check if forces exceed deadband threshold
 *
 * If all force/torque components are below the threshold,
 * the controller stops motion and holds position.
 *
 * @return true if any force component exceeds threshold
 */
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

/**
 * @brief Publish Cartesian velocity for monitoring
 */
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
