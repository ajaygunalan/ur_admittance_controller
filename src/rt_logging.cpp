/**
 * @file rt_logging.cpp
 * @brief Real-time safe logging implementation for UR Admittance Controller
 * 
 * This file contains the implementation of RT-safe logging methods
 * that defer actual logging to non-RT context.
 */

#include "admittance_controller.hpp"

namespace ur_admittance_controller {

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

} // namespace ur_admittance_controller
