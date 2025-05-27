
#include "admittance_controller.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ur_admittance_controller::AdmittanceController, 
  controller_interface::ChainableControllerInterface)

