
#ifndef UR_ADMITTANCE_CONTROLLER__ADMITTANCE_CONSTANTS_HPP_
#define UR_ADMITTANCE_CONTROLLER__ADMITTANCE_CONSTANTS_HPP_

#include <cmath>

namespace ur_admittance_controller::constants {


constexpr double STIFFNESS_REDUCTION_FACTOR = 0.5;

constexpr double STIFFNESS_ENGAGEMENT_THRESHOLD = 0.9;

constexpr double STIFFNESS_BLEND_THRESHOLD = 1.0;

constexpr double VIRTUAL_STIFFNESS = 10.0;


constexpr double QUATERNION_EPSILON = 1e-6;

constexpr double PARAMETER_EPSILON = 1e-12;

constexpr double MAX_ORIENTATION_ERROR = M_PI * 0.9;

constexpr double MAX_CONDITION_NUMBER = 1e12;

constexpr double REGULARIZATION_FACTOR = 1e-8;


constexpr double MAX_STIFFNESS_TRANSLATIONAL = 2000.0;

constexpr double MAX_STIFFNESS_ROTATIONAL = 200.0;

constexpr double MIN_DAMPING_RATIO = 0.1;

constexpr double MAX_DAMPING_RATIO = 10.0;

constexpr double MIN_MASS_TRANSLATIONAL = 0.1;

constexpr double MAX_MASS_TRANSLATIONAL = 100.0;

constexpr double MIN_MASS_ROTATIONAL = 0.01;

constexpr double MAX_MASS_ROTATIONAL = 10.0;


constexpr double CACHE_VALIDITY_WARNING_TIME = 0.5;

constexpr double MAX_CONTROL_PERIOD = 0.1;

constexpr double MIN_CONTROL_PERIOD = 1e-6;


constexpr double DEFAULT_TRAJECTORY_DURATION = 5.0;

constexpr double DEFAULT_STIFFNESS_RAMP_TIME = 2.0;

constexpr double DEFAULT_MAX_POSITION_ERROR = 0.15;

constexpr double DEFAULT_MAX_ORIENTATION_ERROR = 0.5;


constexpr double MIN_LINEAR_VEL = 0.001;

constexpr double MAX_LINEAR_VEL = 2.0;

constexpr double MIN_ANGULAR_VEL = 0.001;

constexpr double MAX_ANGULAR_VEL = 3.14;

}

#endif

