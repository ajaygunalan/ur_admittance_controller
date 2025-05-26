/**
 * @file realtime_control_core.cpp
 * @brief Real-time control algorithms for UR Admittance Controller
 * 
 * This file contains the high-frequency control loop implementation
 * and all real-time safe operations. Performance critical code only.
 */

 #include "admittance_controller.hpp"
 #include "admittance_constants.hpp"
 #include "matrix_utilities.hpp"
 #include <cstring>  // for memcmp
 
 namespace ur_admittance_controller {
 
 // Use centralized constants
 using namespace constants;
 
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
 
// REAL-TIME SAFE: This method runs in the RT thread and only reads from the buffer
void AdmittanceController::checkParameterUpdates()
{
  if (!params_.dynamic_parameters) return;

  // Signal that we need a parameter update in the non-RT thread
  parameter_update_needed_.store(true);
  
  // Read the latest parameters from the RT buffer
  const auto* latest_params = param_buffer_.readFromRT();
  
  // If no parameters are available yet, return
  if (!latest_params) return;
  
  // Safely copy the parameters (fast operation, no allocations)
  params_ = *latest_params;
}

 // Non-real-time parameter update function
void AdmittanceController::prepareParameterUpdate()
{
  if (!parameter_update_needed_.load()) return;
  
  // Get latest parameters from the parameter server
  auto new_params = param_listener_->get_params();
  
  // Validate parameters before using them
  if (!validateParameters(new_params)) {
    RCLCPP_ERROR(get_node()->get_logger(), 
      "Parameter validation failed - keeping previous parameters");
    parameter_update_needed_.store(false);
    return;
  }
  
  // Proper floating-point comparison with epsilon tolerance
  // Check for changes using proper floating-point comparison
  bool mass_changed = false;
  bool stiffness_changed = false;
  bool damping_changed = false;
  
  for (size_t i = 0; i < 6; ++i) {
    if (!utils::areEqual(params_.admittance.mass[i], new_params.admittance.mass[i])) {
      mass_changed = true;
      break;
    }
  }
  
  for (size_t i = 0; i < 6; ++i) {
    if (!utils::areEqual(params_.admittance.stiffness[i], new_params.admittance.stiffness[i])) {
      stiffness_changed = true;
      break;
    }
  }
  
  for (size_t i = 0; i < 6; ++i) {
    if (!utils::areEqual(params_.admittance.damping_ratio[i], new_params.admittance.damping_ratio[i])) {
      damping_changed = true;
      break;
    }
  }
  
  // If no changes, just write the params to buffer and return
  if (!mass_changed && !stiffness_changed && !damping_changed) {
    param_buffer_.writeFromNonRT(new_params);
    parameter_update_needed_.store(false);
    return;
  }
  
  // Update matrices based on changes - this can safely log since we're non-RT
  if (mass_changed) {
    updateMassMatrix(new_params, true);
  }
  
  if (stiffness_changed) {
    updateStiffnessMatrix(new_params, true);
    // Trigger gradual engagement
    stiffness_recently_changed_ = true;
    stiffness_engagement_factor_ = 0.0;
    RCLCPP_INFO(get_node()->get_logger(), "Starting gradual stiffness engagement");
  }
  
  if (stiffness_changed || damping_changed) {
    updateDampingMatrix(new_params, true);
  }
  
  // Write updated parameters to the RT buffer
  param_buffer_.writeFromNonRT(new_params);
  parameter_update_needed_.store(false);
}

// Update mass matrix - can be called from non-RT context only
void AdmittanceController::updateMassMatrix(const ur_admittance_controller::Params& params, bool log_changes)
{
  // Convert parameter vector to array for utility function
  std::array<double, 6> mass_array;
  for (size_t i = 0; i < 6; ++i) {
    mass_array[i] = params.admittance.mass[i];
    mass_(i, i) = mass_array[i];
  }
  
  // Use centralized mass inverse computation with stability checks
  mass_inverse_ = utils::computeMassInverse(mass_array);
  
  if (log_changes) {
    // Check condition number for logging
    double max_mass = mass_.diagonal().maxCoeff();
    double min_mass = mass_.diagonal().minCoeff();
    double condition_number = max_mass / min_mass;
    
    if (condition_number > MAX_CONDITION_NUMBER || min_mass <= 0.0) {
      RCLCPP_WARN(get_node()->get_logger(), 
        "Mass matrix regularized (condition number: %.2e, min mass: %.6f)", 
        condition_number, min_mass);
    } else {
      RCLCPP_INFO(get_node()->get_logger(), 
        "Mass parameters updated (condition number: %.2e)", condition_number);
    }
  }
}

 // Legacy method - forwards to the new implementation
void AdmittanceController::updateMassMatrix()
{
  // Forward to the new implementation with current params
  updateMassMatrix(params_, false);
}

 // Update stiffness matrix - can be called from non-RT context only
void AdmittanceController::updateStiffnessMatrix(const ur_admittance_controller::Params& params, bool log_changes)
{
  for (size_t i = 0; i < 6; ++i) {
    stiffness_(i, i) = params.admittance.stiffness[i];
  }
  
  if (log_changes) {
    RCLCPP_INFO(get_node()->get_logger(), "Stiffness parameters updated");
  }
}

// Legacy method - forwards to the new implementation
void AdmittanceController::updateStiffnessMatrix()
{
  // Forward to the new implementation with current params
  updateStiffnessMatrix(params_, false);
}

 // Update damping matrix - can be called from non-RT context only
void AdmittanceController::updateDampingMatrix(const ur_admittance_controller::Params& params, bool log_changes)
{
  // Convert parameter vectors to arrays for utility function
  std::array<double, 6> mass_array, stiffness_array, damping_ratio_array;
  
  for (size_t i = 0; i < 6; ++i) {
    mass_array[i] = params.admittance.mass[i];
    stiffness_array[i] = params.admittance.stiffness[i]; 
    damping_ratio_array[i] = params.admittance.damping_ratio[i];
  }
  
  // Use centralized damping computation
  damping_ = utils::computeDampingMatrix(mass_array, stiffness_array, damping_ratio_array);
  
  if (log_changes) {
    RCLCPP_INFO(get_node()->get_logger(), 
      "Damping parameters updated with mathematically correct formulas");
  }
}

// Legacy method - forwards to the new implementation
void AdmittanceController::updateDampingMatrix()
{
  // Forward to the new implementation with current params
  updateDampingMatrix(params_, false);
}

 // Function implementation moved to sensor_processing.cpp
 // This stub is removed - the actual implementation is in sensor_processing.cpp
 
void AdmittanceController::processNonRTErrors()
{
  // 1. Process RT logs first
  processRTLogs();
  
  // 2. Update transform caches in non-RT context if needed
  // This is safe to do here since this is explicitly outside the RT control path
  updateTransformCaches();
  
  // 3. Update parameters in non-RT context if needed
  // This handles all parameter updates and logging in non-RT context
  prepareParameterUpdate();
  
  // 4. Process any real-time errors in a non-real-time context
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
   // First update our reference interfaces (exported to downstream controllers)
   for (size_t i = 0; i < params_.joints.size(); ++i) {
     joint_position_references_[i] = joint_positions_[i];
   }
   
   // Now write to the claimed downstream controller reference interfaces
   // This is critical for proper controller chaining
   for (size_t i = 0; i < command_interfaces_.size(); ++i) {
     // Use the mapping to get the correct joint index for each command interface
     const size_t joint_idx = cmd_interface_to_joint_index_[i];
     
     // Write the joint position reference to the downstream controller interface
     command_interfaces_[i].set_value(joint_position_references_[joint_idx]);
   }
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
       joint_position_references_[i] = state_interfaces_[pos_state_indices_[i]].get_value();
     }
     
     return controller_interface::return_type::OK;
     
   } catch (const std::exception &) {
     // Real-time safe error reporting without formatting or memory allocation
     reportRTError(RTErrorType::CONTROL_ERROR);
     return controller_interface::return_type::ERROR;
   }
 }
 
 // Function implementation moved to sensor_processing.cpp
 // This stub is removed - the actual implementation is in sensor_processing.cpp

// Parameter validation methods are implemented in parameter_validation.cpp

 } // namespace ur_admittance_controller