/**
 * @file sensor_processing.cpp
 * @brief Sensor data processing for UR Admittance Controller
 * 
 * This file contains functions for sensor data acquisition,
 * transformation, and filtering.
 */

#include "admittance_controller.hpp"
#include <memory>

namespace ur_admittance_controller {

bool AdmittanceController::updateSensorData()
{
  try {
    // Read F/T sensor data
    Vector6d raw_wrench = Vector6d::Zero();
    for (size_t i = 0; i < 6; ++i) {
      if (ft_indices_[i] >= 0) {
        const auto& interface = state_interfaces_[ft_indices_[i]];
        if (interface.get_optional()) {
          raw_wrench(i) = interface.get_optional().value();
        }
      }
    }
    
    // Update transforms
    if (!updateTransforms()) {
      // Use untransformed wrench if transform fails
      wrench_ = raw_wrench;
      return checkDeadband();
    }
    
    // Apply transform to wrench
    wrench_ = ft_transform_cache_.adjoint.transpose() * raw_wrench;
    
    // Apply filtering
    wrench_filtered_ = params_.admittance.filter_coefficient * wrench_ + 
      (1.0 - params_.admittance.filter_coefficient) * wrench_filtered_;
    
    return checkDeadband();
    
  } catch (const std::exception & e) {
    if (rclcpp::ok()) {
      RCLCPP_ERROR_SKIPFIRST_THROTTLE(rt_logger_, *get_node()->get_clock(), 1000,
        "Exception in updateSensorData: %s", e.what());
    }
    return false;
  }
}

bool AdmittanceController::updateTransforms()
{
  auto now = get_node()->get_clock()->now();
  bool ft_valid = true;
  
  // Update F/T transform if needed
  if (!ft_transform_cache_.valid || 
      (now - ft_transform_cache_.last_update).seconds() > TRANSFORM_TIMEOUT) {
    ft_valid = updateSingleTransform(params_.base_link, params_.ft_frame, 
                                    ft_transform_cache_, now);
  }
  
  // Update EE transform if needed
  if (!ee_transform_cache_.valid || 
      (now - ee_transform_cache_.last_update).seconds() > TRANSFORM_TIMEOUT) {
    try {
      ee_transform_cache_.transform = tf_buffer_->lookupTransform(
        params_.base_link, params_.tip_link, 
        tf2_ros::fromMsg(now), tf2::durationFromSec(0.0));
      current_pose_ = tf2::transformToEigen(ee_transform_cache_.transform);
      ee_transform_cache_.valid = true;
      ee_transform_cache_.last_update = now;
    } catch (const tf2::TransformException &) {
      // Continue with previous pose
    }
  }
  
  // Log if using old cached transforms
  if ((now - ft_transform_cache_.last_update).seconds() > CACHE_VALIDITY_WARNING_TIME) {
    RCLCPP_DEBUG_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "Using cached transform for real-time safety");
  }
  
  return ft_valid;
}

bool AdmittanceController::updateSingleTransform(
    const std::string& target_frame,
    const std::string& source_frame,
    TransformCache& cache,
    const rclcpp::Time& time)
{
  try {
    cache.transform = tf_buffer_->lookupTransform(
      target_frame, source_frame, 
      tf2_ros::fromMsg(time), tf2::durationFromSec(0.0));
    
    // Get rotation and translation
    Eigen::Matrix3d R = tf2::transformToEigen(cache.transform).rotation();
    Eigen::Vector3d t = tf2::transformToEigen(cache.transform).translation();
    
    // Build adjoint matrix for wrench transformation
    cache.adjoint = Matrix6d::Identity();
    cache.adjoint.block<3, 3>(0, 0) = R;
    cache.adjoint.block<3, 3>(3, 3) = R;
    
    // Add translation cross-product term for torque transformation
    Eigen::Matrix3d t_cross;
    t_cross << 0, -t.z(), t.y(),
               t.z(), 0, -t.x(),
               -t.y(), t.x(), 0;
    cache.adjoint.block<3, 3>(0, 3) = t_cross * R;
    
    cache.valid = true;
    cache.last_update = time;
    return true;
    
  } catch (const tf2::TransformException & ex) {
    if (!cache.valid && rclcpp::ok()) {
      RCLCPP_WARN_SKIPFIRST_THROTTLE(rt_logger_, *get_node()->get_clock(), 1000,
        "Transform from %s to %s not available: %s", 
        source_frame.c_str(), target_frame.c_str(), ex.what());
    }
    return false;
  }
}

bool AdmittanceController::checkDeadband()
{
  // Check if any force/torque exceeds threshold
  for (size_t i = 0; i < 6; ++i) {
    if (std::abs(wrench_filtered_(i)) > params_.admittance.min_motion_threshold) {
      return true;
    }
  }
  
  // Below threshold - maintain current position
  cart_twist_.setZero();
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    joint_position_references_[i] = state_interfaces_[pos_state_indices_[i]].get_optional().value();
  }
  return false;
}

void AdmittanceController::publishCartesianVelocity()
{
  if (rt_cart_vel_pub_->trylock()) {
    auto& msg = rt_cart_vel_pub_->msg_;
    msg.linear.x = cart_twist_(0);
    msg.linear.y = cart_twist_(1);
    msg.linear.z = cart_twist_(2);
    msg.angular.x = cart_twist_(3);
    msg.angular.y = cart_twist_(4);
    msg.angular.z = cart_twist_(5);
    rt_cart_vel_pub_->unlockAndPublish();
  }
}

} // namespace ur_admittance_controller