/**
 * @file realtime_control_core.cpp
 * @brief Real-time control algorithms for UR Admittance Controller
 * 
 * This file contains the high-frequency control loop implementation
 * and all real-time safe operations. Performance critical code only.
 */

#include "admittance_controller.hpp"

namespace ur_admittance_controller {

controller_interface::return_type AdmittanceController::update_reference_from_subscribers(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Update parameters if dynamic
  if (params_.dynamic_parameters) {
    params_ = param_listener_->get_params();
  }
  
  return controller_interface::return_type::OK;
}

controller_interface::return_type AdmittanceController::update_and_write_commands(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  try {
    // Parameter hot-reload for live tuning
    if (params_.dynamic_parameters) {
      auto new_params = param_listener_->get_params();
      
      // Check which parameters have changed
      bool mass_changed = (params_.admittance.mass != new_params.admittance.mass);
      bool stiffness_changed = (params_.admittance.stiffness != new_params.admittance.stiffness);
      bool damping_changed = (params_.admittance.damping_ratio != new_params.admittance.damping_ratio);
      
      // Only update matrices if control parameters changed
      if (mass_changed || stiffness_changed || damping_changed) {
        // Always update params first
        params_ = new_params;
        
        // Update mass matrix if mass changed
        if (mass_changed) {
          for (size_t i = 0; i < 6; ++i) {
            mass_(i, i) = params_.admittance.mass[i];
          }
          // Pre-compute mass inverse only when mass changes
          mass_inverse_ = mass_.inverse();
          RCLCPP_INFO(get_node()->get_logger(), "Mass parameters updated");
        }
        
        // Update stiffness matrix if stiffness changed
        if (stiffness_changed) {
          for (size_t i = 0; i < 6; ++i) {
            stiffness_(i, i) = params_.admittance.stiffness[i];
          }
          RCLCPP_INFO(get_node()->get_logger(), "Stiffness parameters updated");
          
          // Detect stiffness changes
          if (stiffness_changed) {
            stiffness_recently_changed_ = true;
            stiffness_engagement_factor_ = 0.0;
            RCLCPP_INFO(get_node()->get_logger(), "Starting gradual stiffness engagement");
          }
        }
        
        // Update damping matrix if stiffness or damping ratio changed
        // (since damping depends on both stiffness and damping ratio)
        if (stiffness_changed || damping_changed) {
          for (size_t i = 0; i < 6; ++i) {
            // Smooth damping calculation to avoid discontinuity when stiffness changes
            // from zero to non-zero values
            const double stiffness_threshold = 1.0; // N/m or Nm/rad threshold for blending
            double stiffness_value = params_.admittance.stiffness[i];
            
            if (stiffness_value <= 0.0) {
              // Pure admittance mode - direct damping value in Ns/m or Nms/rad
              // Base damping scaled by mass for consistent units
              damping_(i, i) = params_.admittance.damping_ratio[i] * 
                std::sqrt(params_.admittance.mass[i]); // Consistent units Ns/m or Nms/rad
            } 
            else if (stiffness_value >= stiffness_threshold) {
              // Full impedance mode - critical damping formula
              damping_(i, i) = 2.0 * params_.admittance.damping_ratio[i] * 
                std::sqrt(params_.admittance.mass[i] * stiffness_value);
            }
            else {
              // Smooth transition zone - blend between the two formulas
              double blend_factor = stiffness_value / stiffness_threshold; // 0.0 to 1.0
              
              // Calculate both damping values
              double admittance_damping = params_.admittance.damping_ratio[i] * 
                std::sqrt(params_.admittance.mass[i]);
                
              double impedance_damping = 2.0 * params_.admittance.damping_ratio[i] * 
                std::sqrt(params_.admittance.mass[i] * stiffness_value);
              
              // Smoothly blend between the two values
              damping_(i, i) = (1.0 - blend_factor) * admittance_damping + 
                              blend_factor * impedance_damping;
            }
          }
          RCLCPP_INFO(get_node()->get_logger(), "Damping parameters updated with smooth transitions");
        }
      }
    }
    
    // Step 1: Update sensor data and check for early exit conditions
    if (!updateSensorData()) {
      // Either sensor data processing failed or forces below threshold
      // updateSensorData already took appropriate actions
      return controller_interface::return_type::OK;
    }
    
    // Step 2: Compute admittance control output with impedance terms
    Vector6d command_velocity = computeAdmittanceControl(period);
    
    // Check if drift prevention was applied (returns zero velocity)
    if (command_velocity.isZero()) {
      // Drift prevention triggered - nothing more to do
      return controller_interface::return_type::OK;
    }
    
    // Step 3: Convert Cartesian commands to joint space
    if (!convertToJointSpace(command_velocity, period)) {
      // Conversion failed - safe fallback handled in the method
      return controller_interface::return_type::OK;
    }
    
    // Step 4: Apply joint limits (position and velocity)
    applyJointLimits(period);
    
    // Step 5: Publish outputs to downstream controllers and monitoring topics
    publishOutputs(period);
    
    return controller_interface::return_type::OK;
    
  } catch (const std::exception & e) {
    if (rclcpp::ok()) {
      RCLCPP_ERROR_SKIPFIRST_THROTTLE(rt_logger_, *get_node()->get_clock(), 1000,
        "Exception in update: %s", e.what());
    }
    
    // Handle failure gracefully
    return safeStop();
  }
}

void AdmittanceController::publishCartesianVelocity()
{
  // Use realtime publisher with trylock pattern to ensure RT safety
  if (rt_cart_vel_pub_->trylock()) {
    // Access message via msg_ member
    auto& msg = rt_cart_vel_pub_->msg_;
    msg.linear.x = cart_twist_(0);
    msg.linear.y = cart_twist_(1);
    msg.linear.z = cart_twist_(2);
    msg.angular.x = cart_twist_(3);
    msg.angular.y = cart_twist_(4);
    msg.angular.z = cart_twist_(5);
    // Unlock and publish in a non-blocking way
    rt_cart_vel_pub_->unlockAndPublish();
  }
}

Vector6d AdmittanceController::computePoseError()
{
  Vector6d error = Vector6d::Zero();
  
  // Position error (translation difference)
  error.head<3>() = desired_pose_.translation() - current_pose_.translation();
  
  // Orientation error calculation with improved handling of singularities
  // Get rotation matrices
  Eigen::Matrix3d R_current = current_pose_.rotation();
  Eigen::Matrix3d R_desired = desired_pose_.rotation();
  
  // Convert to quaternions for more stable interpolation
  Eigen::Quaterniond q_current(R_current);
  Eigen::Quaterniond q_desired(R_desired);
  
  // Ensure we're taking the shortest path (quaternion has double cover)
  if (q_current.dot(q_desired) < 0) {
    q_current.coeffs() = -q_current.coeffs();
  }
  
  // Calculate the error quaternion
  Eigen::Quaterniond q_error = q_desired * q_current.inverse();
  
  // Convert to scaled axis-angle representation for the controller
  // This avoids the singularity at 180 degrees rotation
  Eigen::AngleAxisd aa_error(q_error);
  
  // Handle the case where angle is very small (near identity rotation)
  const double epsilon = 1e-6;
  if (aa_error.angle() < epsilon) {
    error.tail<3>().setZero();
  } else {
    // Scale axis by angle - this gives a more stable representation
    // and gracefully handles the approaching-pi case
    error.tail<3>() = aa_error.axis() * aa_error.angle();
    
    // For numerical stability, limit the maximum orientation error magnitude
    // This prevents excessive control actions near singularities
    const double max_orientation_error = M_PI * 0.9; // 90% of π
    double error_norm = error.tail<3>().norm();
    if (error_norm > max_orientation_error) {
      error.tail<3>() *= max_orientation_error / error_norm;
    }
  }
  
  return error;
}

bool AdmittanceController::updateSensorData()
{
  const auto now = get_node()->get_clock()->now();

  try {
    // Read force-torque sensor values
    Vector6d raw_wrench = Vector6d::Zero();
    
    // Process each F/T sensor component
    for (size_t i = 0; i < ft_indices_.size() && i < 6; ++i) {
      // Handles both signed and unsupported interfaces safely
      const auto& interface = state_interfaces_[ft_indices_[i]];
      if (interface.get_optional()) {
        raw_wrench[i] = interface.get_optional().value();
      }
    }
    
    // Ensure F/T sensor transform is up-to-date
    if (!ft_transform_cache_.valid || 
        (now - ft_transform_cache_.last_update).seconds() > TRANSFORM_TIMEOUT) {
      try {
        // Look up transform from sensor to base frame
        ft_transform_cache_.transform = tf_buffer_->lookupTransform(
          params_.base_link, params_.ft_frame, 
          tf2_ros::fromMsg(now), tf2::durationFromSec(0.0));
        
        // Get rotation matrix for wrench transformation
        Eigen::Matrix3d rotation = tf2::transformToEigen(ft_transform_cache_.transform).rotation();
        
        // Precalculate adjoint for efficiency
        Eigen::Vector3d translation = 
          tf2::transformToEigen(ft_transform_cache_.transform).translation();
        
        ft_transform_cache_.adjoint = Eigen::MatrixXd::Identity(6, 6);
        ft_transform_cache_.adjoint.block<3, 3>(0, 0) = rotation;
        ft_transform_cache_.adjoint.block<3, 3>(3, 3) = rotation;
        
        // Add translation component using cross product matrix
        Eigen::Matrix3d t_cross;
        t_cross << 0, -translation.z(), translation.y(),
                   translation.z(), 0, -translation.x(),
                   -translation.y(), translation.x(), 0;
        ft_transform_cache_.adjoint.block<3, 3>(0, 3) = t_cross * rotation;
        
        // Mark cache as valid and update timestamp
        ft_transform_cache_.valid = true;
        ft_transform_cache_.last_update = now;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "Could not transform wrench: %s", ex.what());
        return false;
      }
    }
    
    // Update current_pose_ for impedance control
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
        // Continue with previous pose if transform fails
      }
    }
    
    // Apply transform using cached adjoint
    wrench_ = ft_transform_cache_.adjoint.transpose() * raw_wrench;
    
    // Log occasionally when using a cached transform
    if ((now - ft_transform_cache_.last_update).seconds() > 0.5) {
      RCLCPP_DEBUG_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
        "Using cached transform for real-time safety");
    }
    
    // Apply unified filtering
    wrench_filtered_ = params_.admittance.filter_coefficient * wrench_ + 
      (1.0 - params_.admittance.filter_coefficient) * wrench_filtered_;
    
    // Deadband check
    bool motion_above_threshold = false;
    for (size_t i = 0; i < 6; ++i) {
      if (std::abs(wrench_filtered_(i)) > params_.admittance.min_motion_threshold) {
        motion_above_threshold = true;
        break;
      }
    }
    
    if (!motion_above_threshold) {
      cart_twist_.setZero();
      for (size_t i = 0; i < params_.joints.size(); ++i) {
        joint_position_references_[i] = state_interfaces_[pos_state_indices_[i]].get_optional().value();
      }
      return false; // Early exit - no need to continue processing
    }
    
    return true;
  } catch (const std::exception & e) {
    if (rclcpp::ok()) {
      RCLCPP_ERROR_SKIPFIRST_THROTTLE(rt_logger_, *get_node()->get_clock(), 1000,
        "Exception in updateSensorData: %s", e.what());
    }
    return false;
  }
}

Vector6d AdmittanceController::computeAdmittanceControl(const rclcpp::Duration& period)
{
  try {
    // Compute pose error for impedance control
    pose_error_ = computePoseError();
    
    // Safety check: Verify pose error is within safe limits
    double position_error_norm = pose_error_.head<3>().norm();
    double orientation_error_norm = pose_error_.tail<3>().norm();
    
    bool error_within_limits = true;
    
    // Only check error limits when stiffness is fully engaged
    if (stiffness_engagement_factor_ > 0.9 && 
        (position_error_norm > safe_startup_params_.max_position_error || 
         orientation_error_norm > safe_startup_params_.max_orientation_error)) {
      error_within_limits = false;
      
      // If error exceeds limits, log warning and reduce stiffness
      RCLCPP_WARN(get_node()->get_logger(), 
        "SAFETY: Pose error exceeds limits (pos: %.3f > %.3f, orient: %.3f > %.3f). Reducing stiffness.",
        position_error_norm, safe_startup_params_.max_position_error,
        orientation_error_norm, safe_startup_params_.max_orientation_error);
        
      // Trigger gradual re-engagement of stiffness
      stiffness_recently_changed_ = true;
      stiffness_engagement_factor_ = 0.5;  // Reduce to 50% immediately
    }
    
    // Gradually engage stiffness over the configured ramp time
    if (stiffness_recently_changed_) {
      // Only increase if error is within limits
      if (error_within_limits) {
        stiffness_engagement_factor_ += period.seconds() / safe_startup_params_.stiffness_ramp_time;
        if (stiffness_engagement_factor_ >= 1.0) {
          stiffness_engagement_factor_ = 1.0;
          stiffness_recently_changed_ = false;
          RCLCPP_INFO(get_node()->get_logger(), "Stiffness fully engaged");
        }
      }
    }
    
    // Publish pose error for debugging (in a real-time safe way)
    if (rt_pose_error_pub_->trylock()) {
      auto& msg = rt_pose_error_pub_->msg_;
      
      // Linear error (translation)
      msg.linear.x = pose_error_(0);
      msg.linear.y = pose_error_(1);
      msg.linear.z = pose_error_(2);
      
      // Angular error (rotation)
      msg.angular.x = pose_error_(3);
      msg.angular.y = pose_error_(4);
      msg.angular.z = pose_error_(5);
      
      rt_pose_error_pub_->unlockAndPublish();
    }
    
    // Compute admittance with impedance: M*a + D*v + K*e = F_ext
    // Use pre-computed mass inverse for performance
    Vector6d stiffness_force = stiffness_engagement_factor_ * (stiffness_ * pose_error_);
    desired_accel_ = mass_inverse_ * 
      (wrench_filtered_ - damping_ * desired_vel_ - stiffness_force);
    desired_vel_ += desired_accel_ * period.seconds();
    
    // Apply axis enables
    for (size_t i = 0; i < 6; ++i) {
      if (!params_.admittance.enabled_axes[i]) {
        desired_vel_(i) = 0.0;
      }
    }
    
    // Apply Cartesian velocity safety limits
    // Separate linear and angular components
    double linear_velocity_magnitude = desired_vel_.head<3>().norm();
    double angular_velocity_magnitude = desired_vel_.tail<3>().norm();
    
    // Clamp linear velocity if needed
    if (linear_velocity_magnitude > params_.max_linear_velocity && linear_velocity_magnitude > 0) {
      desired_vel_.head<3>() *= (params_.max_linear_velocity / linear_velocity_magnitude);
    }
    
    // Clamp angular velocity if needed
    if (angular_velocity_magnitude > params_.max_angular_velocity && angular_velocity_magnitude > 0) {
      desired_vel_.tail<3>() *= (params_.max_angular_velocity / angular_velocity_magnitude);
    }
    
    Vector6d result_velocity = desired_vel_;
    cart_twist_ = result_velocity;
    
    // DRIFT PREVENTION: Reset positions when nearly stationary to prevent accumulation of numerical errors
    // Use configurable threshold from parameters (default 0.001 = 1mm/s threshold)
    if (cart_twist_.norm() < params_.admittance.drift_reset_threshold) {
      // Reset integration when robot is nearly stationary
      desired_vel_.setZero();
      cart_twist_.setZero();
      
      // Reset to actual positions using C++17 range-based for loop
      size_t i = 0;
      for (const auto& index : pos_state_indices_) {
        joint_positions_[i++] = state_interfaces_[index].get_optional().value();
      }
      
      // Update desired pose to current pose to prevent drift
      desired_pose_ = current_pose_;
      
      RCLCPP_DEBUG(get_node()->get_logger(), "Drift correction applied - resetting to actual position");
      return Vector6d::Zero(); // Return zero velocity
    }
    
    return result_velocity;
  } catch (const std::exception & e) {
    if (rclcpp::ok()) {
      RCLCPP_ERROR_SKIPFIRST_THROTTLE(rt_logger_, *get_node()->get_clock(), 1000,
        "Exception in computeAdmittanceControl: %s", e.what());
    }
    return Vector6d::Zero(); // Return zero velocity on error
  }
}

bool AdmittanceController::convertToJointSpace(
    const Vector6d& cmd_vel, 
    const rclcpp::Duration& period)
{
  try {
    // Convert to joint space using real kinematics
    // Use pre-allocated vector for real-time safety
    for (size_t i = 0; i < params_.joints.size(); ++i) {
      current_pos_[i] = state_interfaces_[pos_state_indices_[i]].get_optional().value();
    }
    
    // CRITICAL FIX: Convert velocity to displacement deltas by multiplying by Δt
    // Use pre-allocated vector for real-time safety
    for (size_t i = 0; i < 6; ++i) {
      cart_displacement_deltas_[i] = cmd_vel(i) * period.seconds();
    }
    
    // Use pre-allocated joint_deltas_ vector for real-time safety
    if (kinematics_ && (*kinematics_)->convert_cartesian_deltas_to_joint_deltas(
          current_pos_, cart_displacement_deltas_, params_.tip_link, joint_deltas_)) {
      
      // Apply joint deltas and limits with proper ordering
      for (size_t i = 0; i < params_.joints.size() && i < joint_deltas_.size(); ++i) {
        // Start with base position + delta
        joint_positions_[i] = current_pos_[i] + joint_deltas_[i];
      }
      return true;
    } else {
      if (rclcpp::ok()) {
        RCLCPP_WARN_SKIPFIRST_THROTTLE(rt_logger_, *get_node()->get_clock(), 1000,
          "Kinematics conversion failed - zeroing motion");
      }
      
      // ERROR HANDLING: Zero motion on kinematics failure
      cart_twist_.setZero();
      desired_vel_.setZero();
      
      // Maintain current position references
      for (size_t i = 0; i < params_.joints.size(); ++i) {
        joint_position_references_[i] = current_pos_[i];
      }
      return false;
    }
  } catch (const std::exception & e) {
    if (rclcpp::ok()) {
      RCLCPP_ERROR_SKIPFIRST_THROTTLE(rt_logger_, *get_node()->get_clock(), 1000,
        "Exception in convertToJointSpace: %s", e.what());
    }
    return false;
  }
}

void AdmittanceController::applyJointLimits(const rclcpp::Duration& period)
{
  try {
    // Apply joint limits with proper ordering
    for (size_t i = 0; i < params_.joints.size() && i < joint_deltas_.size(); ++i) {
      // FIRST: Apply velocity limits
      double velocity = joint_deltas_[i] / period.seconds();
      if (std::abs(velocity) > joint_limits_[i].max_velocity) {
        // Scale down the delta to respect velocity limit
        double scale = joint_limits_[i].max_velocity / std::abs(velocity);
        joint_positions_[i] = current_pos_[i] + joint_deltas_[i] * scale;
      }
      
      // THEN: Apply position limits (always last!)
      joint_positions_[i] = std::clamp(
        joint_positions_[i], 
        joint_limits_[i].min_position, 
        joint_limits_[i].max_position);
      
      // Update reference interfaces
      joint_position_references_[i] = joint_positions_[i];  // ScaledJTC reads these
    }
  } catch (const std::exception & e) {
    if (rclcpp::ok()) {
      RCLCPP_ERROR_SKIPFIRST_THROTTLE(rt_logger_, *get_node()->get_clock(), 1000,
        "Exception in applyJointLimits: %s", e.what());
    }
  }
}

void AdmittanceController::publishOutputs(const rclcpp::Duration& period)
{
  try {
    // Send trajectory message to scaled_joint_trajectory_controller
    trajectory_msgs::msg::JointTrajectory traj_msg;
    traj_msg.joint_names = params_.joints;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    
    // Set positions from calculated joint positions
    point.positions.resize(params_.joints.size());
    for (size_t i = 0; i < params_.joints.size(); ++i) {
      point.positions[i] = joint_positions_[i];
    }
    
    // Add velocities from joint deltas
    point.velocities.resize(params_.joints.size(), 0.0);
    for (size_t i = 0; i < params_.joints.size() && i < joint_deltas_.size(); ++i) {
      point.velocities[i] = joint_deltas_[i] / period.seconds();
    }
    
    // Set small future time from start
    point.time_from_start = rclcpp::Duration::from_seconds(TRANSFORM_TIMEOUT);
    
    // Add point and publish trajectory
    traj_msg.points.push_back(point);
    if (rt_trajectory_pub_->trylock()) {
      rt_trajectory_pub_->msg_ = traj_msg;
      rt_trajectory_pub_->unlockAndPublish();
    }
    
    // Publish Cartesian velocity for monitoring
    publishCartesianVelocity();
  } catch (const std::exception & e) {
    if (rclcpp::ok()) {
      RCLCPP_ERROR_SKIPFIRST_THROTTLE(rt_logger_, *get_node()->get_clock(), 1000,
        "Exception in publishOutputs: %s", e.what());
    }
  }
}

controller_interface::return_type AdmittanceController::safeStop()
{
  try {
    // SAFE FALLBACK: Zero all motion
    cart_twist_.setZero();
    desired_vel_.setZero();
    
    // Send trajectory message with current positions (no motion)
    trajectory_msgs::msg::JointTrajectory traj_msg;
    traj_msg.joint_names = params_.joints;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    
    // Set positions from current joint positions
    point.positions.resize(params_.joints.size());
    for (size_t i = 0; i < params_.joints.size(); ++i) {
      point.positions[i] = current_pos_[i];
      joint_position_references_[i] = current_pos_[i];
    }
    
    // Zero velocities
    point.velocities.resize(params_.joints.size(), 0.0);
    
    // Set small future time from start
    point.time_from_start = rclcpp::Duration::from_seconds(TRANSFORM_TIMEOUT);
    
    // Add point and publish trajectory
    traj_msg.points.push_back(point);
    if (rt_trajectory_pub_->trylock()) {
      rt_trajectory_pub_->msg_ = traj_msg;
      rt_trajectory_pub_->unlockAndPublish();
    }
    
    // Publish zeroed Cartesian velocity
    publishCartesianVelocity();
    
    return controller_interface::return_type::OK;
  } catch (const std::exception & e) {
    if (rclcpp::ok()) {
      RCLCPP_ERROR_SKIPFIRST_THROTTLE(rt_logger_, *get_node()->get_clock(), 1000,
        "Exception in safeStop: %s", e.what());
    }
    return controller_interface::return_type::ERROR;
  }
}

} // namespace ur_admittance_controller
