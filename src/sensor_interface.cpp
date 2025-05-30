/**
 * @file sensor_interface.cpp
 * @brief Force/torque sensor and transform management implementation
 *
 * This file handles sensor data acquisition, filtering, and coordinate
 * frame transformations for the admittance controller.
 */

#include "admittance_controller.hpp"
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
bool AdmittanceController::updateSensorData()
{
  Vector6d raw_wrench = Vector6d::Zero();
  
  if (use_topic_mode_) {
    // Read from RT buffer
    const auto* sensor_data = rt_ft_buffer_.readFromRT();
    if (!sensor_data) {
      reportRTError(RTErrorType::SENSOR_ERROR);
      return false;
    }
    
    // Check sequence number for missed updates
    uint64_t current_seq = ft_sequence_number_.load(std::memory_order_acquire);
    uint64_t last_seq = last_processed_sequence_.load(std::memory_order_relaxed);
    
    if (current_seq > last_seq + 1) {
      int missed = static_cast<int>(current_seq - last_seq - 1);
      rtLogWarn(RTLogType::WARN_DEADBAND_ACTIVE, missed);
      
      int total_missed = consecutive_missed_updates_.fetch_add(missed) + missed;
      if (total_missed > params_.fallback_strategy.max_missed_updates) {
        reportRTError(RTErrorType::SENSOR_ERROR);
        return handleFallbackStrategy();
      }
    } else {
      consecutive_missed_updates_.store(0);
    }
    
    last_processed_sequence_.store(current_seq, std::memory_order_relaxed);
    
    // Validate timestamp freshness
    auto now = get_node()->get_clock()->now();
    double age_ms = (now - sensor_data->receive_time).seconds() * 1000.0;
    
    if (age_ms > params_.sensor_interface.topic_config.data_timeout_ms) {
      rtLogWarn(RTLogType::WARN_DEADBAND_ACTIVE, age_ms);
      return handleFallbackStrategy();
    }
    
    // Extract wrench data
    raw_wrench(0) = sensor_data->msg.wrench.force.x;
    raw_wrench(1) = sensor_data->msg.wrench.force.y;
    raw_wrench(2) = sensor_data->msg.wrench.force.z;
    raw_wrench(3) = sensor_data->msg.wrench.torque.x;
    raw_wrench(4) = sensor_data->msg.wrench.torque.y;
    raw_wrench(5) = sensor_data->msg.wrench.torque.z;
    
    // Apply cached transform if available
    if (sensor_data->transform_valid && 
        params_.sensor_interface.topic_config.enable_transform_caching) {
      // Extract rotation for adjoint computation
      Eigen::Matrix3d R = sensor_data->sensor_to_base_transform.rotation();
      Eigen::Vector3d t = sensor_data->sensor_to_base_transform.translation();
      
      // Compute adjoint matrix
      Matrix6d adjoint = Matrix6d::Zero();
      adjoint.block<3,3>(0,0) = R;
      adjoint.block<3,3>(3,3) = R;
      
      Eigen::Matrix3d t_cross;
      t_cross << 0, -t.z(), t.y(),
                 t.z(), 0, -t.x(),
                 -t.y(), t.x(), 0;
      adjoint.block<3,3>(3,0) = t_cross * R;
      
      // Transform wrench to base frame
      F_sensor_base_ = adjoint * raw_wrench;
    } else {
      // Use existing transform lookup mechanism
      if (!updateTransforms()) {
        F_sensor_base_ = raw_wrench;  // Fallback to sensor frame
      } else if (transform_base_ft_.isValid()) {
        const auto& transform_data = transform_base_ft_.getTransform();
        F_sensor_base_ = transform_data.adjoint * raw_wrench;
      } else {
        F_sensor_base_ = raw_wrench;
      }
    }
    
  } else {
    // Existing hardware interface code
    for (size_t i = 0; i < 6; ++i) {
      if (ft_indices_[i] >= 0 && static_cast<size_t>(ft_indices_[i]) < state_interfaces_.size()) {
        const auto& interface = state_interfaces_[ft_indices_[i]];
        auto value = interface.get_optional();
        if (value.has_value() && value.value() != std::numeric_limits<double>::quiet_NaN()) {
          raw_wrench(i) = value.value();
          if (std::isnan(raw_wrench(i))) {
            reportRTError(RTErrorType::SENSOR_ERROR);
            return false;
          }
        }
      }
    }
    
    // Transform for hardware mode
    if (!updateTransforms()) {
      F_sensor_base_ = raw_wrench;
      return checkDeadband();
    }
    
    if (transform_base_ft_.isValid()) {
      const auto& transform_data = transform_base_ft_.getTransform();
      F_sensor_base_ = transform_data.adjoint * raw_wrench;
    } else {
      F_sensor_base_ = raw_wrench;
    }
  }
  
  // Common filtering and deadband check
  wrench_filtered_ = params_.admittance.filter_coefficient * F_sensor_base_ + 
    (1.0 - params_.admittance.filter_coefficient) * wrench_filtered_;
  
  if (wrench_filtered_.hasNaN()) {
    reportRTError(RTErrorType::SENSOR_ERROR);
    return false;
  }
  
  return checkDeadband();
}

bool AdmittanceController::updateTransforms()
{
  
  if (transform_base_tip_.isValid()) {
    const auto& ee_data = transform_base_tip_.getTransform();
    X_base_tip_current_ = tf2::transformToEigen(ee_data.transform);
  }
  
  auto now = get_node()->get_clock()->now();
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

bool AdmittanceController::updateTransform_base_tip()
{
  auto now = get_node()->get_clock()->now();
  
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
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "Tip transform lookup failed: %s", ex.what());
    return false;
  }
}

bool AdmittanceController::updateTransform_base_ft()
{
  auto now = get_node()->get_clock()->now();
  
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
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "F/T transform lookup failed: %s", ex.what());
    return false;
  }
}

void AdmittanceController::updateTransformCaches()
{
  bool tip_updated = updateTransform_base_tip();
  bool ft_updated = updateTransform_base_ft();
  bool all_updated = tip_updated && ft_updated;
  
  transform_update_needed_.store(!all_updated);
}

void AdmittanceController::updateEETransformOnly()
{
  try {
    auto now = get_node()->get_clock()->now();
    
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
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
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
bool AdmittanceController::checkDeadband()
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
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    if (i < pos_state_indices_.size() && i < joint_position_references_.size()) {
      size_t idx = pos_state_indices_[i];
      if (idx < state_interfaces_.size()) {
        auto value = state_interfaces_[idx].get_optional();
        if (value.has_value()) {
          joint_position_references_[i] = value.value();
        }
      }
    }
  }
  return false;
}

/**
 * @brief Publish Cartesian velocity for monitoring
 *
 * Uses real-time safe publisher to avoid blocking RT thread
 */
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

}  // namespace ur_admittance_controller
