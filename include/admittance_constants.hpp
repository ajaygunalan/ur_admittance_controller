/**
 * @file admittance_constants.hpp
 * @brief Consolidated constants for UR Admittance Controller
 * 
 * This header centralizes all magic numbers and constants used throughout
 * the controller to improve maintainability and reduce duplication.
 * 
 * @author Generated Code Improvement
 * @date 2025
 */

#ifndef UR_ADMITTANCE_CONTROLLER__ADMITTANCE_CONSTANTS_HPP_
#define UR_ADMITTANCE_CONTROLLER__ADMITTANCE_CONSTANTS_HPP_

#include <cmath>

namespace ur_admittance_controller::constants {

// ================================
// Control Algorithm Constants
// ================================

/// Factor for reducing stiffness when safety limits are exceeded
constexpr double STIFFNESS_REDUCTION_FACTOR = 0.5;

/// Threshold for full stiffness engagement (0.0 to 1.0)
constexpr double STIFFNESS_ENGAGEMENT_THRESHOLD = 0.9;

/// Stiffness value threshold for smooth blending between admittance/impedance modes
constexpr double STIFFNESS_BLEND_THRESHOLD = 1.0;  // N/m or Nm/rad

/// Virtual stiffness for pure admittance mode to ensure dimensional correctness
constexpr double VIRTUAL_STIFFNESS = 10.0;  // N/m or Nm/rad

// ================================
// Numerical Precision Constants
// ================================

/// Epsilon for quaternion operations and small angle handling
constexpr double QUATERNION_EPSILON = 1e-6;

/// Epsilon for floating-point parameter comparison
constexpr double PARAMETER_EPSILON = 1e-12;

/// Maximum orientation error to avoid singularities (90% of π)
constexpr double MAX_ORIENTATION_ERROR = M_PI * 0.9;

/// Maximum condition number for matrix inversion
constexpr double MAX_CONDITION_NUMBER = 1e12;

/// Regularization factor for numerical stability
constexpr double REGULARIZATION_FACTOR = 1e-8;

// ================================
// Physical Limits & Safety
// ================================

/// Maximum translational stiffness values (N/m)
constexpr double MAX_STIFFNESS_TRANSLATIONAL = 2000.0;

/// Maximum rotational stiffness values (Nm/rad)
constexpr double MAX_STIFFNESS_ROTATIONAL = 200.0;

/// Minimum damping ratio for stability
constexpr double MIN_DAMPING_RATIO = 0.1;

/// Maximum reasonable damping ratio
constexpr double MAX_DAMPING_RATIO = 10.0;

/// Minimum translational mass (kg)
constexpr double MIN_MASS_TRANSLATIONAL = 0.1;

/// Maximum translational mass (kg)
constexpr double MAX_MASS_TRANSLATIONAL = 100.0;

/// Minimum rotational mass (kg⋅m²)
constexpr double MIN_MASS_ROTATIONAL = 0.01;

/// Maximum rotational mass (kg⋅m²)
constexpr double MAX_MASS_ROTATIONAL = 10.0;

// ================================
// Time & Performance Constants
// ================================

/// Time threshold for cache validity warnings (seconds)
constexpr double CACHE_VALIDITY_WARNING_TIME = 0.5;

/// Maximum reasonable control loop period (seconds)
constexpr double MAX_CONTROL_PERIOD = 0.1;

/// Minimum control loop period (seconds) 
constexpr double MIN_CONTROL_PERIOD = 1e-6;

// ================================
// Default Safe Startup Parameters
// ================================

/// Default trajectory duration for safe startup (seconds)
constexpr double DEFAULT_TRAJECTORY_DURATION = 5.0;

/// Default stiffness engagement time (seconds)
constexpr double DEFAULT_STIFFNESS_RAMP_TIME = 2.0;

/// Default maximum safe position error (meters)
constexpr double DEFAULT_MAX_POSITION_ERROR = 0.15;

/// Default maximum safe orientation error (radians)
constexpr double DEFAULT_MAX_ORIENTATION_ERROR = 0.5;

// ================================
// Velocity Limits
// ================================

/// Minimum linear velocity (m/s)
constexpr double MIN_LINEAR_VEL = 0.001;

/// Maximum linear velocity (m/s)
constexpr double MAX_LINEAR_VEL = 2.0;

/// Minimum angular velocity (rad/s)
constexpr double MIN_ANGULAR_VEL = 0.001;

/// Maximum angular velocity (rad/s)
constexpr double MAX_ANGULAR_VEL = 3.14;

}  // namespace ur_admittance_controller::constants

#endif  // UR_ADMITTANCE_CONTROLLER__ADMITTANCE_CONSTANTS_HPP_
