/**
 * @file plugin_export.cpp
 * @brief Plugin registration for UR Admittance Controller
 * 
 * This file contains the ROS 2 plugin registration macro required for
 * the controller loader to discover and instantiate this controller.
 */

#include "admittance_controller.hpp"
#include "pluginlib/class_list_macros.hpp"

// Register the controller as a plugin
PLUGINLIB_EXPORT_CLASS(
  ur_admittance_controller::AdmittanceController, 
  controller_interface::ChainableControllerInterface)
