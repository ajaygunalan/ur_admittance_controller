/**
 * @file parameter_validation.cpp
 * @brief Parameter validation for UR Admittance Controller
 * 
 * This file contains comprehensive parameter validation methods
 * to ensure controller safety and stability.
 */

#include "admittance_controller.hpp"
#include <cmath>
#include <string>

namespace ur_admittance_controller {

bool AdmittanceController::validateParameters(const ur_admittance_controller::Params& params) const
{
  bool valid = true;
  
  // Convert std::vector<double> to std::array<double, 6> for validation
  std::array<double, 6> mass_array{};
  std::array<double, 6> stiffness_array{};
  std::array<double, 6> damping_array{};
  
  // Safely copy with bounds checking
  for (size_t i = 0; i < 6 && i < params.admittance.mass.size(); ++i) {
    mass_array[i] = params.admittance.mass[i];
  }
  for (size_t i = 0; i < 6 && i < params.admittance.stiffness.size(); ++i) {
    stiffness_array[i] = params.admittance.stiffness[i];
  }
  for (size_t i = 0; i < 6 && i < params.admittance.damping_ratio.size(); ++i) {
    damping_array[i] = params.admittance.damping_ratio[i];
  }
  
  // Validate mass parameters
  if (!validateMassParameters(mass_array)) {
    valid = false;
  }
  
  // Validate stiffness parameters
  if (!validateStiffnessParameters(stiffness_array)) {
    valid = false;
  }
  
  // Validate damping parameters
  if (!validateDampingParameters(damping_array)) {
    valid = false;
  }
  
  // Validate velocity limits
  if (!validateVelocityLimits(params.max_linear_velocity, params.max_angular_velocity)) {
    valid = false;
  }
  
  // Validate filter coefficient
  if (params.admittance.filter_coefficient < 0.01 || params.admittance.filter_coefficient > 0.99) {
    logParameterValidationError("filter_coefficient", 
      "Value must be between 0.01 and 0.99 for stability");
    valid = false;
  }
  
  // Validate motion threshold
  if (params.admittance.min_motion_threshold < 0.01 || params.admittance.min_motion_threshold > 50.0) {
    logParameterValidationError("min_motion_threshold", 
      "Value must be between 0.01 and 50.0 N/Nm");
    valid = false;
  }
  
  return valid;
}

bool AdmittanceController::validateMassParameters(const std::array<double, 6>& mass) const
{
  constexpr double MIN_MASS_TRANSLATIONAL = 0.1;    // kg
  constexpr double MAX_MASS_TRANSLATIONAL = 100.0;  // kg
  constexpr double MIN_MASS_ROTATIONAL = 0.01;      // kg⋅m²
  constexpr double MAX_MASS_ROTATIONAL = 10.0;      // kg⋅m²
  
  bool valid = true;
  
  // Check translational masses (X, Y, Z)
  for (size_t i = 0; i < 3; ++i) {
    if (std::isnan(mass[i]) || std::isinf(mass[i])) {
      logParameterValidationError("mass[" + std::to_string(i) + "]", 
        "Value cannot be NaN or infinite");
      valid = false;
    } else if (mass[i] <= 0.0) {
      logParameterValidationError("mass[" + std::to_string(i) + "]", 
        "Mass must be positive");
      valid = false;
    } else if (mass[i] < MIN_MASS_TRANSLATIONAL || mass[i] > MAX_MASS_TRANSLATIONAL) {
      logParameterValidationError("mass[" + std::to_string(i) + "]", 
        "Translational mass must be between " + std::to_string(MIN_MASS_TRANSLATIONAL) + 
        " and " + std::to_string(MAX_MASS_TRANSLATIONAL) + " kg");
      valid = false;
    }
  }
  
  // Check rotational masses (Rx, Ry, Rz)
  for (size_t i = 3; i < 6; ++i) {
    if (std::isnan(mass[i]) || std::isinf(mass[i])) {
      logParameterValidationError("mass[" + std::to_string(i) + "]", 
        "Value cannot be NaN or infinite");
      valid = false;
    } else if (mass[i] <= 0.0) {
      logParameterValidationError("mass[" + std::to_string(i) + "]", 
        "Mass must be positive");
      valid = false;
    } else if (mass[i] < MIN_MASS_ROTATIONAL || mass[i] > MAX_MASS_ROTATIONAL) {
      logParameterValidationError("mass[" + std::to_string(i) + "]", 
        "Rotational mass must be between " + std::to_string(MIN_MASS_ROTATIONAL) + 
        " and " + std::to_string(MAX_MASS_ROTATIONAL) + " kg⋅m²");
      valid = false;
    }
  }
  
  return valid;
}

bool AdmittanceController::validateStiffnessParameters(const std::array<double, 6>& stiffness) const
{
  constexpr double MAX_STIFFNESS_TRANSLATIONAL = 2000.0;  // N/m
  constexpr double MAX_STIFFNESS_ROTATIONAL = 200.0;     // Nm/rad
  
  bool valid = true;
  
  // Check translational stiffness (X, Y, Z)
  for (size_t i = 0; i < 3; ++i) {
    if (std::isnan(stiffness[i]) || std::isinf(stiffness[i])) {
      logParameterValidationError("stiffness[" + std::to_string(i) + "]", 
        "Value cannot be NaN or infinite");
      valid = false;
    } else if (stiffness[i] < 0.0) {
      logParameterValidationError("stiffness[" + std::to_string(i) + "]", 
        "Stiffness must be non-negative");
      valid = false;
    } else if (stiffness[i] > MAX_STIFFNESS_TRANSLATIONAL) {
      logParameterValidationError("stiffness[" + std::to_string(i) + "]", 
        "Translational stiffness must not exceed " + std::to_string(MAX_STIFFNESS_TRANSLATIONAL) + " N/m");
      valid = false;
    }
  }
  
  // Check rotational stiffness (Rx, Ry, Rz)
  for (size_t i = 3; i < 6; ++i) {
    if (std::isnan(stiffness[i]) || std::isinf(stiffness[i])) {
      logParameterValidationError("stiffness[" + std::to_string(i) + "]", 
        "Value cannot be NaN or infinite");
      valid = false;
    } else if (stiffness[i] < 0.0) {
      logParameterValidationError("stiffness[" + std::to_string(i) + "]", 
        "Stiffness must be non-negative");
      valid = false;
    } else if (stiffness[i] > MAX_STIFFNESS_ROTATIONAL) {
      logParameterValidationError("stiffness[" + std::to_string(i) + "]", 
        "Rotational stiffness must not exceed " + std::to_string(MAX_STIFFNESS_ROTATIONAL) + " Nm/rad");
      valid = false;
    }
  }
  
  return valid;
}

bool AdmittanceController::validateDampingParameters(const std::array<double, 6>& damping_ratio) const
{
  constexpr double MIN_DAMPING_RATIO = 0.1;   // Minimum for stability
  constexpr double MAX_DAMPING_RATIO = 10.0;  // Maximum reasonable value
  
  bool valid = true;
  
  for (size_t i = 0; i < 6; ++i) {
    if (std::isnan(damping_ratio[i]) || std::isinf(damping_ratio[i])) {
      logParameterValidationError("damping_ratio[" + std::to_string(i) + "]", 
        "Value cannot be NaN or infinite");
      valid = false;
    } else if (damping_ratio[i] <= 0.0) {
      logParameterValidationError("damping_ratio[" + std::to_string(i) + "]", 
        "Damping ratio must be positive");
      valid = false;
    } else if (damping_ratio[i] < MIN_DAMPING_RATIO) {
      logParameterValidationError("damping_ratio[" + std::to_string(i) + "]", 
        "Damping ratio below " + std::to_string(MIN_DAMPING_RATIO) + " may cause instability");
      valid = false;
    } else if (damping_ratio[i] > MAX_DAMPING_RATIO) {
      logParameterValidationError("damping_ratio[" + std::to_string(i) + "]", 
        "Damping ratio above " + std::to_string(MAX_DAMPING_RATIO) + " is excessive");
      valid = false;
    }
  }
  
  return valid;
}

bool AdmittanceController::validateVelocityLimits(double max_linear, double max_angular) const
{
  constexpr double MIN_LINEAR_VEL = 0.001;   // m/s
  constexpr double MAX_LINEAR_VEL = 2.0;     // m/s
  constexpr double MIN_ANGULAR_VEL = 0.001;  // rad/s
  constexpr double MAX_ANGULAR_VEL = 3.14;   // rad/s
  
  bool valid = true;
  
  if (std::isnan(max_linear) || std::isinf(max_linear) || 
      max_linear < MIN_LINEAR_VEL || max_linear > MAX_LINEAR_VEL) {
    logParameterValidationError("max_linear_velocity", 
      "Must be between " + std::to_string(MIN_LINEAR_VEL) + " and " + 
      std::to_string(MAX_LINEAR_VEL) + " m/s");
    valid = false;
  }
  
  if (std::isnan(max_angular) || std::isinf(max_angular) || 
      max_angular < MIN_ANGULAR_VEL || max_angular > MAX_ANGULAR_VEL) {
    logParameterValidationError("max_angular_velocity", 
      "Must be between " + std::to_string(MIN_ANGULAR_VEL) + " and " + 
      std::to_string(MAX_ANGULAR_VEL) + " rad/s");
    valid = false;
  }
  
  return valid;
}

void AdmittanceController::logParameterValidationError(
  const std::string& parameter_name, const std::string& reason) const
{
  RCLCPP_ERROR(get_node()->get_logger(), 
    "Parameter validation failed for '%s': %s", 
    parameter_name.c_str(), reason.c_str());
}

} // namespace ur_admittance_controller
