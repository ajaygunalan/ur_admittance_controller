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

// Constants used in this file (should match realtime_control_core.cpp)
constexpr double CACHE_VALIDITY_WARNING_TIME = 0.5;  // seconds

bool AdmittanceController::updateSensorData()
{
  // REAL-TIME SAFE: No exceptions, no memory allocations
  
  // Read F/T sensor data with bounds checking
  Vector6d raw_wrench = Vector6d::Zero();
  for (size_t i = 0; i < 6; ++i) {
    if (ft_indices_[i] >= 0 && static_cast<size_t>(ft_indices_[i]) < state_interfaces_.size()) {
      const auto& interface = state_interfaces_[ft_indices_[i]];
      if (interface.get_value() != std::numeric_limits<double>::quiet_NaN()) {
        raw_wrench(i) = interface.get_value();
        if (std::isnan(raw_wrench(i))) {
          reportRTError(RTErrorType::SENSOR_ERROR);
          return false;
        }
      }
    }
  }
  
  // Update transforms - this is real-time safe now and only uses cached data
  if (!updateTransforms()) {
    // Use untransformed wrench if no valid transform exists
    wrench_ = raw_wrench;
    return checkDeadband();
  }
  
  // Apply correct transform to wrench - real-time safe since adjoint is cached
  if (ft_transform_cache_.isValid()) {
    const auto& transform_data = ft_transform_cache_.getTransform();
    wrench_ = transform_data.adjoint * raw_wrench;
  } else {
    // No valid transform, use untransformed wrench
    wrench_ = raw_wrench;
  }
  
  // Apply filtering
  wrench_filtered_ = params_.admittance.filter_coefficient * wrench_ + 
    (1.0 - params_.admittance.filter_coefficient) * wrench_filtered_;
  
  // Check for NaN values after all operations
  if (wrench_filtered_.hasNaN()) {
    reportRTError(RTErrorType::SENSOR_ERROR);
    return false;
  }
  
  return checkDeadband();
}

bool AdmittanceController::updateTransforms()
{
  // REAL-TIME SAFE: Only check atomic flag, never do lookups
  // This is called from the real-time thread and must be deterministic
  
  // Mark that we need a transform update (will be handled in non-RT context)
  auto now = get_node()->get_clock()->now();
  if ((now - ft_transform_cache_.last_update).seconds() > TRANSFORM_TIMEOUT) {
    transform_update_needed_.store(true);
  }
  
  // Only use valid cached transforms
  if (!ft_transform_cache_.isValid()) {
    // No valid transform yet, but don't report an error if we've never had one
    return false;
  }
  
  // Update the current pose from the cached EE transform if it's valid
  if (ee_transform_cache_.isValid()) {
    const auto& ee_data = ee_transform_cache_.getTransform();
    current_pose_ = tf2::transformToEigen(ee_data.transform);
  }
  
  // Log if using old cached transforms
  if ((now - ft_transform_cache_.last_update).seconds() > CACHE_VALIDITY_WARNING_TIME) {
    transform_update_needed_.store(true);
  }
  
  return true;
}

// Non-real-time transform update method
void AdmittanceController::updateTransformCaches()
{
  // This method is called from a non-RT context and can safely use TF
  if (!transform_update_needed_.load()) {
    return; // No update needed
  }
  
  auto now = get_node()->get_clock()->now();
  bool updated = false;
  
  try {
    // Setup transform names if not already done
    if (ft_transform_cache_.target_frame.empty()) {
      ft_transform_cache_.target_frame = params_.base_link;
      ft_transform_cache_.source_frame = params_.ft_frame;
    }
    
    if (ee_transform_cache_.target_frame.empty()) {
      ee_transform_cache_.target_frame = params_.base_link;
      ee_transform_cache_.source_frame = params_.tip_link;
    }
    
    // Get F/T transform
    auto ft_transform = tf_buffer_->lookupTransform(
      ft_transform_cache_.target_frame, 
      ft_transform_cache_.source_frame,
      tf2_ros::fromMsg(now), tf2::durationFromSec(0.1));
    
    // Get rotation and translation
    Eigen::Matrix3d R = tf2::transformToEigen(ft_transform).rotation();
    Eigen::Vector3d t = tf2::transformToEigen(ft_transform).translation();
    
    // Build correct adjoint matrix for wrench transformation
    Matrix6d adjoint = Matrix6d::Zero();
    
    // Upper-left block: rotation matrix
    adjoint.block<3, 3>(0, 0) = R;
    
    // Lower-right block: rotation matrix
    adjoint.block<3, 3>(3, 3) = R;
    
    // Lower-left block: cross-product term for torque transformation
    Eigen::Matrix3d t_cross;
    t_cross << 0, -t.z(), t.y(),
               t.z(), 0, -t.x(),
               -t.y(), t.x(), 0;
    adjoint.block<3, 3>(3, 0) = t_cross * R;
    
    // Use the atomic double-buffering method for race-condition-free updates
    ft_transform_cache_.updateTransform(ft_transform, adjoint, now);
    
    // Get EE transform
    auto ee_transform = tf_buffer_->lookupTransform(
      ee_transform_cache_.target_frame, 
      ee_transform_cache_.source_frame,
      tf2_ros::fromMsg(now), tf2::durationFromSec(0.1));
    
    // Use the atomic double-buffering method for race-condition-free updates
    // Note: EE transform doesn't need adjoint, so we pass identity matrix
    Matrix6d identity_adjoint = Matrix6d::Identity();
    ee_transform_cache_.updateTransform(ee_transform, identity_adjoint, now);
    
    updated = true;
  } catch (const tf2::TransformException& ex) {
    // Log but don't throw - just keep using the old transforms
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "Transform lookup failed: %s", ex.what());
  }
  
  transform_update_needed_.store(!updated);
}

// This method has been replaced by the non-RT updateTransformCaches() method

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
    joint_position_references_[i] = state_interfaces_[pos_state_indices_[i]].get_value();
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