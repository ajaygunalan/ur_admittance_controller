// Constants and configuration parameters for the UR admittance controller

#ifndef UR_ADMITTANCE_CONTROLLER__ADMITTANCE_CONSTANTS_HPP_
#define UR_ADMITTANCE_CONTROLLER__ADMITTANCE_CONSTANTS_HPP_

#include <cmath>

namespace ur_admittance_controller::constants {

// Stiffness Control Parameters
constexpr double STIFFNESS_REDUCTION_FACTOR = 0.5;
constexpr double STIFFNESS_ENGAGEMENT_THRESHOLD = 0.9;
constexpr double STIFFNESS_BLEND_THRESHOLD = 1.0;
constexpr double VIRTUAL_STIFFNESS = 10.0;

// Numerical Precision and Limits
constexpr double QUATERNION_EPSILON = 1e-6;
constexpr double PARAMETER_EPSILON = 1e-12;
constexpr double MAX_ORIENTATION_ERROR = M_PI * 0.9;
constexpr double MAX_CONDITION_NUMBER = 1e12;
constexpr double REGULARIZATION_FACTOR = 1e-8;


}  // namespace ur_admittance_controller::constants

#endif  // UR_ADMITTANCE_CONTROLLER__ADMITTANCE_CONSTANTS_HPP_

