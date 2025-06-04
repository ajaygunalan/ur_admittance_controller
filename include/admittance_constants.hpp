// Constants and configuration parameters for the UR admittance controller

#ifndef UR_ADMITTANCE_CONTROLLER__ADMITTANCE_CONSTANTS_HPP_
#define UR_ADMITTANCE_CONTROLLER__ADMITTANCE_CONSTANTS_HPP_

#include <cmath>

namespace ur_admittance_controller::constants {

// Stiffness Control Parameters
constexpr double VIRTUAL_STIFFNESS = 10.0;  // Used when stiffness is zero

}  // namespace ur_admittance_controller::constants

#endif  // UR_ADMITTANCE_CONTROLLER__ADMITTANCE_CONSTANTS_HPP_

