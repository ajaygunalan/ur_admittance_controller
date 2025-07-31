#pragma once

#include <ur_admittance_controller/utilities/types.hpp>
#include <ur_admittance_controller/utilities/spatial_math.hpp>

namespace ur_admittance_controller {
namespace algorithms {

/**
 * @brief Apply gravity compensation to raw wrench
 * 
 * Implements the compensation equation from Yu et al. 2022:
 * F_compensated = F_raw - F_gravity - F_bias
 * 
 * @param wrench_raw Raw wrench from sensor in sensor frame
 * @param X_TB Transform matching lookupTransform(tool, base) order
 * @param params Calibration parameters
 * @return Compensated wrench in sensor frame
 */
inline Wrench6d compensateWrench(
    const Wrench6d& wrench_raw,
    const Transform& X_TB,
    const GravityCompensationParams& params) {
  
  // Extract rotation matching lookupTransform(tool, base) order
  const Matrix3d& R_TB = X_TB.rotation();
  
  // Transform gravity from base to sensor frame: R_SE * R_TB * f_gravity_B
  Force3d f_gravity_S = params.R_SE * R_TB * params.f_gravity_B;
  
  // Compute torque due to gravity
  Torque3d t_gravity_S = params.p_SCoM_S.cross(f_gravity_S);
  
  // Build gravity wrench
  Wrench6d wrench_gravity;
  wrench_gravity.head<3>() = f_gravity_S;
  wrench_gravity.tail<3>() = t_gravity_S;
  
  // Build bias wrench
  Wrench6d wrench_bias;
  wrench_bias.head<3>() = params.f_bias_S;
  wrench_bias.tail<3>() = params.t_bias_S;
  
  // Apply compensation
  return wrench_raw - wrench_gravity - wrench_bias;
}

/**
 * @brief Transform compensated wrench from sensor to base frame
 * 
 * @param wrench_sensor Wrench in sensor frame
 * @param X_BS Transform from base to sensor
 * @return Wrench in base frame
 */
inline Wrench6d transformWrenchToBase(
    const Wrench6d& wrench_sensor,
    const Transform& X_BS) {
  return spatial::transformWrench(wrench_sensor, X_BS.inverse());
}

/**
 * @brief Complete wrench processing pipeline
 * 
 * 1. Apply gravity compensation
 * 2. Transform to base frame
 * 3. Scale by admittance ratio
 * 
 * @param wrench_raw Raw sensor wrench
 * @param X_TB Transform matching lookupTransform(tool, base) order
 * @param X_BS Transform from base to sensor
 * @param params Calibration parameters
 * @param admittance_ratio Scaling factor (0-1)
 * @return Processed wrench in base frame
 */
inline Wrench6d processWrench(
    const Wrench6d& wrench_raw,
    const Transform& X_TB,
    const Transform& X_BS,
    const GravityCompensationParams& params,
    double admittance_ratio) {
  
  // Apply compensation
  Wrench6d wrench_compensated = compensateWrench(wrench_raw, X_TB, params);
  
  // Transform to base frame
  Wrench6d wrench_base = transformWrenchToBase(wrench_compensated, X_BS);
  
  // Apply scaling
  return admittance_ratio * wrench_base;
}

} // namespace algorithms
} // namespace ur_admittance_controller