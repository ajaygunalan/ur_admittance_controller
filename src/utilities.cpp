/**
 * @file utilities.cpp
 * @brief Utility functions for UR Admittance Controller
 * 
 * This file contains RT-safe logging, parameter validation utilities,
 * and matrix helper functions.
 */

#include "admittance_controller.hpp"
#include "matrix_utilities.hpp"

namespace ur_admittance_controller {

//=============================================================================
// Real-Time Safe Logging Implementation
//=============================================================================

void AdmittanceController::processRTLogs()
{
  // Process all pending RT log messages in non-RT context
  RTLogMessage msg;
  while (rt_log_buffer_.pop(msg)) {
    // Convert to actual ROS logging based on level
    
    // Special case: handle string literal messages first
    if (msg.literal_msg != nullptr) {
      switch (msg.level) {
        case LogLevel::DEBUG:
          RCLCPP_DEBUG(get_node()->get_logger(), "[RT-LOG] %s", msg.literal_msg);
          break;
        case LogLevel::INFO:
          RCLCPP_INFO(get_node()->get_logger(), "[RT-LOG] %s", msg.literal_msg);
          break;
        case LogLevel::WARN:
          RCLCPP_WARN(get_node()->get_logger(), "[RT-LOG] %s", msg.literal_msg);
          break;
        case LogLevel::ERROR:
          RCLCPP_ERROR(get_node()->get_logger(), "[RT-LOG] %s", msg.literal_msg);
          break;
        case LogLevel::FATAL:
          RCLCPP_FATAL(get_node()->get_logger(), "[RT-LOG] %s", msg.literal_msg);
          break;
      }
      continue;
    }
    
    // Handle enum-based message types with appropriate formatting
    std::string formatted_message;
    
    switch (msg.type) {
      // System messages
      case RTLogType::SYSTEM_INITIALIZED:
        formatted_message = "System initialized";
        break;
      case RTLogType::SYSTEM_SHUTDOWN:
        formatted_message = "System shutdown";
        break;
        
      // Errors
      case RTLogType::ERROR_SENSOR_READ_FAILED:
        formatted_message = "Sensor read failed";
        break;
      case RTLogType::ERROR_TRANSFORM_INVALID:
        formatted_message = "Transform invalid or expired";
        break;
      case RTLogType::ERROR_KINEMATICS_FAILED:
        formatted_message = "Kinematics computation failed";
        break;
      case RTLogType::ERROR_JOINT_LIMITS:
        formatted_message = "Joint limits exceeded";
        break;
      case RTLogType::ERROR_CONTROL_COMPUTATION:
        formatted_message = "Control computation error";
        break;
        
      // Warnings
      case RTLogType::WARN_POSE_ERROR_LIMIT:
        formatted_message = "Pose error exceeds safety limits, reducing stiffness engagement";
        break;
      case RTLogType::WARN_VELOCITY_LIMITED:
        formatted_message = "Velocity limited for safety";
        break;
      case RTLogType::WARN_DEADBAND_ACTIVE:
        formatted_message = "Force below deadband threshold, no motion";
        break;
        
      // Info
      case RTLogType::INFO_STIFFNESS_ENGAGED:
        formatted_message = "Stiffness engagement completed";
        break;
      case RTLogType::INFO_DRIFT_RESET:
        formatted_message = "Drift compensation reset applied";
        break;
      case RTLogType::INFO_PARAMETER_UPDATED:
        formatted_message = "Parameters updated in RT thread";
        break;

      // Parameterized messages
      case RTLogType::PARAM_FORCE_READING:
        formatted_message = "Force reading: " + std::to_string(msg.param1) + " N";
        break;
      case RTLogType::PARAM_JOINT_POSITION:
        formatted_message = "Joint " + std::to_string(static_cast<int>(msg.param1)) + 
                          " position: " + std::to_string(msg.param2);
        break;
      case RTLogType::PARAM_CART_VELOCITY:
        formatted_message = "Cartesian velocity component " + std::to_string(static_cast<int>(msg.param1)) + 
                          ": " + std::to_string(msg.param2);
        break;
        
      default:
        formatted_message = "Unknown message type: " + std::to_string(static_cast<int>(msg.type));
        break;
    }
    
    // Log the formatted message
    switch (msg.level) {
      case LogLevel::DEBUG:
        RCLCPP_DEBUG(get_node()->get_logger(), "[RT-LOG] %s", formatted_message.c_str());
        break;
      case LogLevel::INFO:
        RCLCPP_INFO(get_node()->get_logger(), "[RT-LOG] %s", formatted_message.c_str());
        break;
      case LogLevel::WARN:
        RCLCPP_WARN(get_node()->get_logger(), "[RT-LOG] %s", formatted_message.c_str());
        break;
      case LogLevel::ERROR:
        RCLCPP_ERROR(get_node()->get_logger(), "[RT-LOG] %s", formatted_message.c_str());
        break;
      case LogLevel::FATAL:
        RCLCPP_FATAL(get_node()->get_logger(), "[RT-LOG] %s", formatted_message.c_str());
        break;
    }
  }
}

//=============================================================================
// Parameter Validation Utilities
//=============================================================================

bool validateAdmittanceParameters(const Params& params, rclcpp::Logger logger)
{
  // Validate mass parameters
  for (size_t i = 0; i < 6; ++i) {
    if (params.admittance.mass[i] <= 0.0) {
      RCLCPP_ERROR(logger, "Mass parameter [%zu] must be positive, got: %f", 
                   i, params.admittance.mass[i]);
      return false;
    }
  }
  
  // Validate stiffness parameters
  for (size_t i = 0; i < 6; ++i) {
    if (params.admittance.stiffness[i] < 0.0) {
      RCLCPP_ERROR(logger, "Stiffness parameter [%zu] must be non-negative, got: %f",
                   i, params.admittance.stiffness[i]);
      return false;
    }
  }
  
  // Validate damping ratio
  for (size_t i = 0; i < 6; ++i) {
    if (params.admittance.damping_ratio[i] < 0.0 || params.admittance.damping_ratio[i] > 2.0) {
      RCLCPP_ERROR(logger, "Damping ratio [%zu] must be between 0 and 2, got: %f",
                   i, params.admittance.damping_ratio[i]);
      return false;
    }
  }
  
  // Validate filter coefficient
  if (params.admittance.filter_coefficient < 0.0 || params.admittance.filter_coefficient > 1.0) {
    RCLCPP_ERROR(logger, "Filter coefficient must be between 0 and 1, got: %f",
                 params.admittance.filter_coefficient);
    return false;
  }
  
  // Velocity limits validation removed - structure not present in current params
  
  // Control timestep validation removed - not in current params structure
  
  return true;
}

bool validateJointLimits(const std::vector<double>& positions,
                        const std::vector<double>& lower_limits,
                        const std::vector<double>& upper_limits,
                        rclcpp::Logger logger)
{
  if (positions.size() != lower_limits.size() || positions.size() != upper_limits.size()) {
    RCLCPP_ERROR(logger, "Joint limit size mismatch");
    return false;
  }
  
  for (size_t i = 0; i < positions.size(); ++i) {
    if (positions[i] < lower_limits[i] || positions[i] > upper_limits[i]) {
      RCLCPP_WARN(logger, "Joint %zu position %f outside limits [%f, %f]",
                  i, positions[i], lower_limits[i], upper_limits[i]);
      return false;
    }
  }
  
  return true;
}

//=============================================================================
// Matrix Helper Functions
//=============================================================================

Matrix6d computeCriticalDamping(const Matrix6d& K, const Matrix6d& M, double damping_ratio)
{
  // Simple critical damping: D = 2 * damping_ratio * sqrt(K * M)
  Matrix6d D = Matrix6d::Zero();
  
  for (size_t i = 0; i < 6; ++i) {
    if (K(i,i) > 0 && M(i,i) > 0) {
      D(i,i) = 2.0 * damping_ratio * std::sqrt(K(i,i) * M(i,i));
    }
  }
  
  return D;
}

bool isMatrixPositiveDefinite(const Matrix6d& matrix)
{
  // Use Eigen's built-in check for positive definiteness
  Eigen::LLT<Matrix6d> llt(matrix);
  return llt.info() == Eigen::Success;
}

void applyVelocityLimits(Vector6d& velocity, double max_linear, double max_angular)
{
  // Limit linear velocities
  for (size_t i = 0; i < 3; ++i) {
    velocity(i) = std::clamp(velocity(i), -max_linear, max_linear);
  }
  
  // Limit angular velocities
  for (size_t i = 3; i < 6; ++i) {
    velocity(i) = std::clamp(velocity(i), -max_angular, max_angular);
  }
}

//=============================================================================
// Transform Utilities
//=============================================================================

Matrix6d computeAdjointMatrix(const Eigen::Isometry3d& transform)
{
  // Extract rotation and translation
  Eigen::Matrix3d R = transform.rotation();
  Eigen::Vector3d t = transform.translation();
  
  // Build adjoint matrix for wrench transformation
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
  
  return adjoint;
}

} // namespace ur_admittance_controller