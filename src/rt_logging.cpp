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
  LogMessage msg;
  while (rt_log_buffer_.pop(msg)) {
    // Convert to actual ROS logging based on level
    auto duration_since_epoch = msg.timestamp.time_since_epoch();
    auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(duration_since_epoch);
    
    switch (msg.level) {
      case LogLevel::DEBUG:
        RCLCPP_DEBUG(get_node()->get_logger(), "[RT-LOG] %s", msg.message.c_str());
        break;
      case LogLevel::INFO:
        RCLCPP_INFO(get_node()->get_logger(), "[RT-LOG] %s", msg.message.c_str());
        break;
      case LogLevel::WARN:
        RCLCPP_WARN(get_node()->get_logger(), "[RT-LOG] %s", msg.message.c_str());
        break;
      case LogLevel::ERROR:
        RCLCPP_ERROR(get_node()->get_logger(), "[RT-LOG] %s", msg.message.c_str());
        break;
      case LogLevel::FATAL:
        RCLCPP_FATAL(get_node()->get_logger(), "[RT-LOG] %s", msg.message.c_str());
        break;
    }
  }
}

} // namespace ur_admittance_controller
