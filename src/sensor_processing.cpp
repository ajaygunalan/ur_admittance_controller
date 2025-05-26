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
    F_sensor_base_ = raw_wrench;
    return checkDeadband();
  }
  
  // Always apply transform from F/T sensor to base frame
  if (transform_base_ft_.isValid()) {
    const auto& transform_data = transform_base_ft_.getTransform();
    F_sensor_base_ = transform_data.adjoint * raw_wrench;
  } else {
    // No valid transform yet - use raw data as fallback
    F_sensor_base_ = raw_wrench;
  }
  
  // Apply filtering
  wrench_filtered_ = params_.admittance.filter_coefficient * F_sensor_base_ + 
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
  
  // Update current pose from EE transform
  if (transform_base_tip_.isValid()) {
    const auto& ee_data = transform_base_tip_.getTransform();
    X_base_tip_current_ = tf2::transformToEigen(ee_data.transform);
  }
  
  // Mark that we need a transform update (will be handled in non-RT context)
  auto now = get_node()->get_clock()->now();
  if ((now - transform_base_ft_.last_update).seconds() > TRANSFORM_TIMEOUT) {
    transform_update_needed_.store(true);
  }
  
  // Only use valid cached transforms
  if (!transform_base_ft_.isValid()) {
    // No valid transform yet, but don't report an error if we've never had one
    return false;
  }
  
  // Update the current pose from the cached EE transform if it's valid
  if (transform_base_tip_.isValid()) {
    const auto& ee_data = transform_base_tip_.getTransform();
    X_base_tip_current_ = tf2::transformToEigen(ee_data.transform);
  }
  
  // Log if using old cached transforms
  if ((now - transform_base_ft_.last_update).seconds() > CACHE_VALIDITY_WARNING_TIME) {
    transform_update_needed_.store(true);
  }
  
  return true;
}

// Non-real-time transform update methods (notation-compliant)
bool AdmittanceController::updateTransform_base_tip()
{
  // This method is called from a non-RT context and can safely use TF
  auto now = get_node()->get_clock()->now();
  
  try {
    // Setup transform names if not already done
    if (transform_base_tip_.target_frame.empty()) {
      transform_base_tip_.target_frame = params_.base_link;
      transform_base_tip_.source_frame = params_.tip_link;
    }
    
    // Get tip transform with clear variable naming
    geometry_msgs::msg::TransformStamped T_base_tip_msg = 
      tf_buffer_->lookupTransform(
        params_.base_link,  // target frame (to)
        params_.tip_link,   // source frame (from) 
        tf2_ros::fromMsg(now), tf2::durationFromSec(0.1));
    
    // Transform doesn't need adjoint for pose operations
    Matrix6d identity_adjoint = Matrix6d::Identity();
    transform_base_tip_.updateTransform(T_base_tip_msg, identity_adjoint, now);
    
    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "Tip transform lookup failed: %s", ex.what());
    return false;
  }
}

bool AdmittanceController::updateTransform_base_ft()
{
  // This method is called from a non-RT context and can safely use TF
  auto now = get_node()->get_clock()->now();
  
  try {
    // Setup transform names if not already done
    if (transform_base_ft_.target_frame.empty()) {
      transform_base_ft_.target_frame = params_.base_link;
      transform_base_ft_.source_frame = params_.ft_frame;
    }
    
    // Get F/T transform with clear variable naming
    geometry_msgs::msg::TransformStamped T_base_ft_msg = 
      tf_buffer_->lookupTransform(
        params_.base_link,  // target frame (to)
        params_.ft_frame,   // source frame (from)
        tf2_ros::fromMsg(now), tf2::durationFromSec(0.1));
    
    // Get rotation and translation for adjoint computation
    Eigen::Matrix3d R = tf2::transformToEigen(T_base_ft_msg).rotation();
    Eigen::Vector3d t = tf2::transformToEigen(T_base_ft_msg).translation();
    
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
    transform_base_ft_.updateTransform(T_base_ft_msg, adjoint, now);
    
    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "F/T transform lookup failed: %s", ex.what());
    return false;
  }
}

void AdmittanceController::updateTransformCaches()
{
  // Update both transforms, regardless of frame configuration
  bool tip_updated = updateTransform_base_tip();
  bool ft_updated = updateTransform_base_ft();
  bool all_updated = tip_updated && ft_updated;
  
  transform_update_needed_.store(!all_updated);
}

void AdmittanceController::updateEETransformOnly()
{
  // Helper method to update only EE transform when F/T transform not needed
  try {
    auto now = get_node()->get_clock()->now();
    
    // Setup EE transform names if not already done
    if (transform_base_tip_.target_frame.empty()) {
      transform_base_tip_.target_frame = params_.base_link;
      transform_base_tip_.source_frame = params_.tip_link;
    }
    
    // Get EE transform only
    auto ee_transform = tf_buffer_->lookupTransform(
      transform_base_tip_.target_frame, 
      transform_base_tip_.source_frame,
      tf2_ros::fromMsg(now), tf2::durationFromSec(0.1));
    
    // EE transform doesn't need adjoint for force transformation
    Matrix6d identity_adjoint = Matrix6d::Identity();
    transform_base_tip_.updateTransform(ee_transform, identity_adjoint, now);
    
    transform_update_needed_.store(false);
    
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "EE transform lookup failed: %s", ex.what());
    transform_update_needed_.store(true);
  }
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
  V_base_tip_base_.setZero();
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    joint_position_references_[i] = state_interfaces_[pos_state_indices_[i]].get_value();
  }
  return false;
}

void AdmittanceController::publishCartesianVelocity()
{
  if (rt_cart_vel_pub_->trylock()) {
    auto& msg = rt_cart_vel_pub_->msg_;
    msg.linear.x = V_base_tip_base_(0);
    msg.linear.y = V_base_tip_base_(1);
    msg.linear.z = V_base_tip_base_(2);
    msg.angular.x = V_base_tip_base_(3);
    msg.angular.y = V_base_tip_base_(4);
    msg.angular.z = V_base_tip_base_(5);
    rt_cart_vel_pub_->unlockAndPublish();
  }
}

} // namespace ur_admittance_controller