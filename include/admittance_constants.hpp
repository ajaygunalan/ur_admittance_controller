// Constants and configuration parameters for the UR admittance controller

#ifndef UR_ADMITTANCE_CONTROLLER__ADMITTANCE_CONSTANTS_HPP_
#define UR_ADMITTANCE_CONTROLLER__ADMITTANCE_CONSTANTS_HPP_

#include <cmath>

namespace ur_admittance_controller::constants {

// Stiffness Control Parameters
constexpr double STIFFNESS_BLEND_THRESHOLD = 1.0;
constexpr double VIRTUAL_STIFFNESS = 10.0;

// Numerical Precision and Limits
constexpr double QUATERNION_EPSILON = 1e-6;
constexpr double PARAMETER_EPSILON = 1e-12;
constexpr double MAX_ORIENTATION_ERROR = M_PI * 0.9;
constexpr double MAX_CONDITION_NUMBER = 1e12;
constexpr double REGULARIZATION_FACTOR = 1e-8;

// Pose Error Safety Limits
constexpr double MAX_SAFE_POSITION_ERROR = 0.5;     // 50cm maximum position jump
constexpr double MAX_SAFE_ORIENTATION_ERROR = 0.5;  // ~28.6° maximum orientation jump

// Control Thread Timing - Optimized for trajectory streaming
constexpr double TARGET_CONTROL_RATE_HZ = 100.0;           // 100Hz for stable trajectory streaming
constexpr double MIN_CONTROL_PERIOD_SEC = 1.0 / TARGET_CONTROL_RATE_HZ;  // 0.01s (10ms)
constexpr double MIN_CONTROL_PERIOD_NS = MIN_CONTROL_PERIOD_SEC * 1e9;   // 10,000,000 ns


}  // namespace ur_admittance_controller::constants

#endif  // UR_ADMITTANCE_CONTROLLER__ADMITTANCE_CONSTANTS_HPP_

