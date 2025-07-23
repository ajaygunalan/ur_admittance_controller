#pragma once

// LROM algorithm implementation for force/torque sensor calibration
// Based on: Yu et al. "Bias Estimation and Gravity Compensation for Wrist-Mounted Force/Torque Sensor"
// IEEE Sensors Journal, 2022

/**
 * Paper-to-Drake Mapping (Yu et al. 2022):
 * - s F (6D)    -> F_S_S_raw  : Raw wrench from sensor
 * - s F (force) -> f_S_S_raw  : Force component only
 * - sT          -> t_S_S_raw  : Torque component only
 * - s_e R       -> R_SE       : Rotation from E to S
 * - e_b R       -> R_EB       : Rotation from B to E
 * - b_g R       -> R_BG       : Rotation from G to B
 * - s F0        -> f_bias_S   : Force bias in S
 * - sT0         -> t_bias_S   : Torque bias in S
 * - s_g P       -> p_SCoM_S   : Tool CoM position from S, in S
 * - Fb          -> f_gravity_B: Gravity force in B
 * 
 * Note: In paper notation s_g P, the 'g' refers to "gravity center" 
 *       (center of mass), not the gravity frame G.
 */

#include "calibration_types.hpp"
#include "ur_admittance_controller/error.hpp"
#include <Eigen/Dense>
#include <vector>
#include <utility>

namespace ur_admittance_controller {

/**
 * @brief Estimates gravitational force in base frame using LROM algorithm
 * 
 * Implements Section 1 of the paper (Equations 24-34):
 * Yu et al. "Bias Estimation and Gravity Compensation for Wrist-Mounted Force/Torque Sensor"
 * IEEE Sensors Journal, 2022
 * 
 * Uses constrained least squares to find F_b without needing to know R_SE.
 * 
 * @param samples Calibration samples containing force readings and robot poses
 * @return Result containing gravitational force vector in base frame (F_b) or error
 * @note Tier 2 callers can use .value() to throw on error
 */
Result<Force3d> estimateGravitationalForceInBaseFrame(
    const std::vector<CalibrationSample>& samples);

/**
 * @brief Estimates sensor-to-endeffector rotation and force bias using Procrustes alignment
 * 
 * Implements Section 2 of the paper (Equations 37-39):
 * Yu et al. "Bias Estimation and Gravity Compensation for Wrist-Mounted Force/Torque Sensor"
 * IEEE Sensors Journal, 2022
 * 
 * Solves the 3D point set alignment problem to find R_SE and force bias.
 * 
 * @param samples Calibration samples with force readings and robot poses
 * @param f_gravity_B Estimated gravity vector in base frame (F_b from Section 1)
 * @return Result containing pair of (R_SE, f_bias_S) - rotation and bias in sensor frame
 * @note Tier 2 callers can use .value() to throw on error
 */
Result<std::pair<Matrix3d, Force3d>> estimateSensorRotationAndForceBias(
    const std::vector<CalibrationSample>& samples,
    const Force3d& f_gravity_B);

/**
 * @brief Estimates tool center of mass and torque bias in sensor frame
 * 
 * Implements the torque identification model from:
 * Yu et al. "Bias Estimation and Gravity Compensation for Wrist-Mounted Force/Torque Sensor"
 * IEEE Sensors Journal, 2022, Section III-C
 * 
 * @param samples Calibration samples with force/torque readings
 * @param f_bias_S Previously estimated force bias (F0 from Step 2)
 * @return Result containing pair of (p_SCoM_S, t_bias_S) in sensor frame
 * @note Tier 2 callers can use .value() to throw on error
 */
Result<std::pair<Vector3d, Torque3d>> estimateCOMAndTorqueBias(
    const std::vector<CalibrationSample>& samples,
    const Force3d& f_bias_S);

/**
 * @brief Estimates the rotation from gravity frame to robot base frame
 * 
 * Implements robot installation bias estimation from:
 * Yu et al. "Bias Estimation and Gravity Compensation for Wrist-Mounted Force/Torque Sensor" 
 * IEEE Sensors Journal, 2022, Section III-D
 * 
 * @param f_gravity_B Gravity vector in base frame (Fb from Step 1)
 * @return Result containing rotation matrix from gravity frame to base frame
 * @note Potential singularity when robot base is horizontal (F_z ≈ 0)
 * @note Tier 2 callers can use .value() to throw on error
 */
Result<Matrix3d> estimateRobotInstallationBias(const Force3d& f_gravity_B);

/**
 * @brief Extracts tool mass from gravity vector magnitude
 * 
 * From Equation (48): mg = ||Fb||
 * 
 * @param f_gravity_B Gravity vector in base frame
 * @return Result containing tool mass in kg
 * @note Tier 2 callers can use .value() to throw on error
 */
Result<double> extractToolMass(const Force3d& f_gravity_B);

} // namespace ur_admittance_controller