/**
 * @file admittance_constants.hpp
 * @brief Constants and configuration parameters for the UR admittance controller
 *
 * This file contains all compile-time constants used throughout the admittance
 * controller implementation. These values define physical limits, safety thresholds,
 * and numerical parameters for the control algorithms.
 *
 * @author UR Robotics Team
 * @date 2024
 */

#ifndef UR_ADMITTANCE_CONTROLLER__ADMITTANCE_CONSTANTS_HPP_
#define UR_ADMITTANCE_CONTROLLER__ADMITTANCE_CONSTANTS_HPP_

#include <cmath>

namespace ur_admittance_controller::constants {

// ============================================================================
// Stiffness Control Parameters
// ============================================================================

/** @brief Factor to reduce stiffness when pose error exceeds safety limits */
constexpr double STIFFNESS_REDUCTION_FACTOR = 0.5;

/** @brief Threshold above which stiffness engagement is considered active (0-1) */
constexpr double STIFFNESS_ENGAGEMENT_THRESHOLD = 0.9;

/** @brief Stiffness value below which blending with virtual stiffness occurs */
constexpr double STIFFNESS_BLEND_THRESHOLD = 1.0;

/** @brief Virtual stiffness used for pure admittance mode (N/m or Nm/rad) */
constexpr double VIRTUAL_STIFFNESS = 10.0;

// ============================================================================
// Numerical Precision and Limits
// ============================================================================

/** @brief Minimum quaternion magnitude to avoid singularities */
constexpr double QUATERNION_EPSILON = 1e-6;

/** @brief Epsilon for floating-point parameter comparisons */
constexpr double PARAMETER_EPSILON = 1e-12;

/** @brief Maximum allowed orientation error (radians) - safety limit */
constexpr double MAX_ORIENTATION_ERROR = M_PI * 0.9;

/** @brief Maximum condition number for matrix inversion stability */
constexpr double MAX_CONDITION_NUMBER = 1e12;

/** @brief Regularization factor added to diagonal for numerical stability */
constexpr double REGULARIZATION_FACTOR = 1e-8;

// ============================================================================
// Physical Parameter Limits
// ============================================================================

/** @brief Maximum translational stiffness (N/m) */
constexpr double MAX_STIFFNESS_TRANSLATIONAL = 2000.0;

/** @brief Maximum rotational stiffness (Nm/rad) */
constexpr double MAX_STIFFNESS_ROTATIONAL = 200.0;

/** @brief Minimum damping ratio (critically damped = 1.0) */
constexpr double MIN_DAMPING_RATIO = 0.1;

/** @brief Maximum damping ratio (overdamped > 1.0) */
constexpr double MAX_DAMPING_RATIO = 10.0;

/** @brief Minimum virtual mass for translation (kg) */
constexpr double MIN_MASS_TRANSLATIONAL = 0.1;

/** @brief Maximum virtual mass for translation (kg) */
constexpr double MAX_MASS_TRANSLATIONAL = 100.0;

/** @brief Minimum virtual inertia for rotation (kg⋅m²) */
constexpr double MIN_MASS_ROTATIONAL = 0.01;

/** @brief Maximum virtual inertia for rotation (kg⋅m²) */
constexpr double MAX_MASS_ROTATIONAL = 10.0;

// ============================================================================
// Timing and Control Loop Parameters
// ============================================================================

/** @brief Time after which transform cache staleness triggers warning (seconds) */
constexpr double CACHE_VALIDITY_WARNING_TIME = 0.5;

/** @brief Maximum allowed control loop period (seconds) */
constexpr double MAX_CONTROL_PERIOD = 0.1;

/** @brief Minimum allowed control loop period (seconds) */
constexpr double MIN_CONTROL_PERIOD = 1e-6;

// ============================================================================
// Safe Startup Parameters
// ============================================================================

/** @brief Default duration for startup trajectory generation (seconds) */
constexpr double DEFAULT_TRAJECTORY_DURATION = 5.0;

/** @brief Default time to ramp up stiffness after error recovery (seconds) */
constexpr double DEFAULT_STIFFNESS_RAMP_TIME = 2.0;

/** @brief Default maximum position error before safety response (meters) */
constexpr double DEFAULT_MAX_POSITION_ERROR = 0.15;

/** @brief Default maximum orientation error before safety response (radians) */
constexpr double DEFAULT_MAX_ORIENTATION_ERROR = 0.5;

// ============================================================================
// Velocity Limits
// ============================================================================

/** @brief Minimum linear velocity magnitude (m/s) - below this is considered zero */
constexpr double MIN_LINEAR_VEL = 0.001;

/** @brief Maximum allowed linear velocity (m/s) - safety limit */
constexpr double MAX_LINEAR_VEL = 2.0;

/** @brief Minimum angular velocity magnitude (rad/s) - below this is considered zero */
constexpr double MIN_ANGULAR_VEL = 0.001;

/** @brief Maximum allowed angular velocity (rad/s) - safety limit */
constexpr double MAX_ANGULAR_VEL = 3.14;

}  // namespace ur_admittance_controller::constants

#endif  // UR_ADMITTANCE_CONTROLLER__ADMITTANCE_CONSTANTS_HPP_

