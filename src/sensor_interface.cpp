
#include "admittance_controller.hpp"
#include <memory>

namespace ur_admittance_controller {

constexpr double CACHE_VALIDITY_WARNING_TIME = 0.5;

bool AdmittanceController::updateSensorData()
{
  
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


bool AdmittanceController::checkDeadband()
{
  for (size_t i = 0; i < 6; ++i) {
    if (std::abs(wrench_filtered_(i)) > params_.admittance.min_motion_threshold) {
      return true;
    }
  }
  
  V_base_tip_base_.setZero();
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    if (i < pos_state_indices_.size() && i < joint_position_references_.size()) {
      size_t idx = pos_state_indices_[i];
      if (idx < state_interfaces_.size()) {
        joint_position_references_[i] = state_interfaces_[idx].get_value();
      }
    }
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

}
