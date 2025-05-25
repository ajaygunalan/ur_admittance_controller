/**
 * @file realtime_control_core.cpp
 * @brief Real-time control algorithms for UR Admittance Controller
 * 
 * This file contains the high-frequency control loop implementation
 * and all real-time safe operations. Performance critical code only.
 */

 #include "admittance_controller.hpp"
 #include <cstring>  // for memcmp
 
 namespace ur_admittance_controller {
 
 // Constants for improved readability and maintainability
 constexpr double STIFFNESS_REDUCTION_FACTOR = 0.5;
 constexpr double STIFFNESS_ENGAGEMENT_THRESHOLD = 0.9;
 constexpr double CACHE_VALIDITY_WARNING_TIME = 0.5;  // seconds
 constexpr double STIFFNESS_BLEND_THRESHOLD = 1.0;   // N/m or Nm/rad for smooth transition
 constexpr double QUATERNION_EPSILON = 1e-6;
 constexpr double MAX_ORIENTATION_ERROR = M_PI * 0.9;  // 90% of π to avoid singularities
 
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
  // Step 1: Check for parameter updates - can't fail
  checkParameterUpdates();
  
  // Step 2: Update sensor data and check for early exit conditions
  if (!updateSensorData()) {
    // Sensor data update failed - already logged in updateSensorData
    return controller_interface::return_type::OK; // Continue to next cycle
  }
  
  // Step 3: Update transforms
  if (!updateTransforms()) {
    // Transform update failed - already logged in updateTransforms
    return controller_interface::return_type::OK; // Continue to next cycle
  }
  
  // Step 4: Deadband check
  if (!checkDeadband()) {
    // Below deadband - no error, just skip calculation
    return controller_interface::return_type::OK;
  }
  
  // Step 5: Compute admittance control outputs
  Vector6d cmd_vel;
  if (!computeAdmittanceControl(period, cmd_vel)) {
    reportRTError(RTErrorType::CONTROL_ERROR);
    return safeStop();
  }
  
  // Step 6: Convert to joint space velocities
  if (!convertToJointSpace(cmd_vel, period)) {
    reportRTError(RTErrorType::KINEMATICS_ERROR);
    return safeStop();
  }
  
  // Step 7: Apply joint limits - failure is critical
  if (!applyJointLimits(period)) {
    reportRTError(RTErrorType::JOINT_LIMITS_ERROR);
    return safeStop(); // Changed to stop if limits can't be applied properly
  }
  
  // Step 8: Update controller references
  updateReferenceInterfaces();
  
  // Step 9: Publish monitoring data - non-critical
  publishMonitoringData();
  
  // Process any errors that occurred in a non-RT context
  processNonRTErrors();
  
  return controller_interface::return_type::OK; 
 }
 
 void AdmittanceController::checkParameterUpdates()
 {
   if (!params_.dynamic_parameters) return;
   
   auto new_params = param_listener_->get_params();
   
   // Use bit flags for efficient change detection
   uint8_t changes = 0;
   constexpr uint8_t MASS_CHANGED = 1;
   constexpr uint8_t STIFFNESS_CHANGED = 2;
   constexpr uint8_t DAMPING_CHANGED = 4;
   
   // Check for changes using memcmp for arrays
   if (std::memcmp(params_.admittance.mass.data(), new_params.admittance.mass.data(), 
                   sizeof(double) * 6) != 0) {
     changes |= MASS_CHANGED;
   }
   if (std::memcmp(params_.admittance.stiffness.data(), new_params.admittance.stiffness.data(), 
                   sizeof(double) * 6) != 0) {
     changes |= STIFFNESS_CHANGED;
   }
   if (std::memcmp(params_.admittance.damping_ratio.data(), new_params.admittance.damping_ratio.data(), 
                   sizeof(double) * 6) != 0) {
     changes |= DAMPING_CHANGED;
   }
   
   if (changes == 0) {
     params_ = new_params;  // Update non-control parameters
     return;
   }
   
   // Update parameters
   params_ = new_params;
   
   // Update matrices based on changes
   if (changes & MASS_CHANGED) {
     updateMassMatrix();
   }
   
   if (changes & STIFFNESS_CHANGED) {
     updateStiffnessMatrix();
     // Trigger gradual engagement
     stiffness_recently_changed_ = true;
     stiffness_engagement_factor_ = 0.0;
     RCLCPP_INFO(get_node()->get_logger(), "Starting gradual stiffness engagement");
   }
   
   if (changes & (STIFFNESS_CHANGED | DAMPING_CHANGED)) {
     updateDampingMatrix();
   }
 }
 
 void AdmittanceController::updateMassMatrix()
 {
   for (size_t i = 0; i < 6; ++i) {
     mass_(i, i) = params_.admittance.mass[i];
   }
   mass_inverse_ = mass_.inverse();
   RCLCPP_INFO(get_node()->get_logger(), "Mass parameters updated");
 }
 
 void AdmittanceController::updateStiffnessMatrix()
 {
   for (size_t i = 0; i < 6; ++i) {
     stiffness_(i, i) = params_.admittance.stiffness[i];
   }
   RCLCPP_INFO(get_node()->get_logger(), "Stiffness parameters updated");
 }
 
 void AdmittanceController::updateDampingMatrix()
 {
   // Smooth damping calculation to avoid discontinuity when stiffness changes
   for (size_t i = 0; i < 6; ++i) {
     double stiffness_value = params_.admittance.stiffness[i];
     
     if (stiffness_value <= 0.0) {
       // Pure admittance mode - direct damping value
       // Scale by sqrt(mass) for consistent units [Ns/m or Nms/rad]
       damping_(i, i) = params_.admittance.damping_ratio[i] * 
         std::sqrt(params_.admittance.mass[i]);
     } 
     else if (stiffness_value >= STIFFNESS_BLEND_THRESHOLD) {
       // Full impedance mode - critical damping formula
       damping_(i, i) = 2.0 * params_.admittance.damping_ratio[i] * 
         std::sqrt(params_.admittance.mass[i] * stiffness_value);
     }
     else {
       // Smooth transition zone - blend between the two formulas
       double blend_factor = stiffness_value / STIFFNESS_BLEND_THRESHOLD;
       
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
 
 bool AdmittanceController::updateSensorData()
 {
   // Implementation moved to sensor_processing.cpp
   return false;
 }
 
void AdmittanceController::processNonRTErrors()
{
  // 1. Update transform caches in non-RT context if needed
  // This is safe to do here since this is explicitly outside the RT control path
  updateTransformCaches();
  
  // 2. Process any real-time errors in a non-real-time context
  RTErrorType error = last_rt_error_.exchange(RTErrorType::NONE);
  
  if (error != RTErrorType::NONE && rclcpp::ok()) {
    // Handle error without real-time violations
    switch (error) {
      case RTErrorType::UPDATE_ERROR:
        RCLCPP_ERROR(get_node()->get_logger(), "Real-time error occurred in update loop");
        break;
      case RTErrorType::SENSOR_ERROR:
        RCLCPP_ERROR(get_node()->get_logger(), "Real-time error in sensor data processing");
        break;
      case RTErrorType::TRANSFORM_ERROR:
        RCLCPP_ERROR(get_node()->get_logger(), "Transform lookup failed in real-time context");
        // Explicitly request transform update on error
        transform_update_needed_.store(true);
        break;
      case RTErrorType::CONTROL_ERROR:
        RCLCPP_ERROR(get_node()->get_logger(), "Control computation error in real-time context");
        break;
      case RTErrorType::KINEMATICS_ERROR:
        RCLCPP_ERROR(get_node()->get_logger(), "Kinematics error in real-time context");
        break;
      case RTErrorType::JOINT_LIMITS_ERROR:
        RCLCPP_ERROR(get_node()->get_logger(), "Joint limits error in real-time context");
        break;
      default:
        RCLCPP_ERROR(get_node()->get_logger(), "Unknown real-time error occurred");
        break;
    }
  }
}
 
 void AdmittanceController::updateReferenceInterfaces()
 {
   // Update reference interfaces for downstream controller
   for (size_t i = 0; i < params_.joints.size(); ++i) {
     joint_position_references_[i] = joint_positions_[i];
   }
   
   // Note: Removed redundant trajectory publishing
   // The reference interfaces are sufficient for controller chaining
 }
 
 void AdmittanceController::publishMonitoringData()
 {
   publishCartesianVelocity();
 }
 
 controller_interface::return_type AdmittanceController::safeStop()
 {
   try {
     // Zero all velocities
     cart_twist_.setZero();
     desired_vel_.setZero();
     
     // Maintain current positions
     for (size_t i = 0; i < params_.joints.size(); ++i) {
       joint_position_references_[i] = state_interfaces_[pos_state_indices_[i]].get_optional().value();
     }
     
     return controller_interface::return_type::OK;
     
   } catch (const std::exception &) {
     // Real-time safe error reporting without formatting or memory allocation
     reportRTError(RTErrorType::CONTROL_ERROR);
     return controller_interface::return_type::ERROR;
   }
 }
 
 // Function moved to sensor_processing.cpp
void AdmittanceController::publishCartesianVelocity()
{
  // Implementation moved to sensor_processing.cpp
}

 } // namespace ur_admittance_controller