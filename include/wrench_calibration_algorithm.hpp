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
#include <Eigen/Dense>
#include <vector>
#include <utility>

namespace ur_admittance_controller {

/**
 * @brief Estimates gravity vector and sensor-to-endeffector rotation using LROM algorithm
 * 
 * Implements the Limited Robot Orientation Method (LROM) from:
 * Yu et al. "Bias Estimation and Gravity Compensation for Wrist-Mounted Force/Torque Sensor"
 * IEEE Sensors Journal, 2022, Section III-A.2
 * 
 * @param samples Calibration samples containing force readings and robot poses
 * @return Pair of (f_gravity_B, R_SE)
 * @throws std::invalid_argument if insufficient samples provided
 */
std::pair<Force3d, Matrix3d> estimateGravityAndRotation(
    const std::vector<CalibrationSample>& samples);

/**
 * @brief Estimates the constant force bias in the sensor frame
 * 
 * Implements force bias estimation from:
 * Yu et al. "Bias Estimation and Gravity Compensation for Wrist-Mounted Force/Torque Sensor"
 * IEEE Sensors Journal, 2022, Section III-B
 * 
 * @param samples Calibration samples with force readings and robot poses
 * @param f_gravity_B Estimated gravity vector in base frame (Fb from Step 1)
 * @param R_SE Sensor-to-endeffector rotation (R_SE from Step 1)
 * @return Force bias vector in sensor frame
 */
Force3d estimateForceBias(
    const std::vector<CalibrationSample>& samples,
    const Force3d& f_gravity_B,
    const Matrix3d& R_SE);

/**
 * @brief Estimates tool center of mass and torque bias in sensor frame
 * 
 * Implements the torque identification model from:
 * Yu et al. "Bias Estimation and Gravity Compensation for Wrist-Mounted Force/Torque Sensor"
 * IEEE Sensors Journal, 2022, Section III-C
 * 
 * @param samples Calibration samples with force/torque readings
 * @param f_bias_S Previously estimated force bias (F0 from Step 2)
 * @return Pair of (p_SCoM_S, t_bias_S) in sensor frame
 */
std::pair<Vector3d, Torque3d> estimateCOMAndTorqueBias(
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
 * @return Rotation matrix from gravity frame to base frame
 * @note Potential singularity when robot base is horizontal (F_z ≈ 0)
 */
Matrix3d estimateRobotInstallationBias(const Force3d& f_gravity_B);

/**
 * @brief Extracts tool mass from gravity vector magnitude
 * 
 * From Equation (48): mg = ||Fb||
 * 
 * @param f_gravity_B Gravity vector in base frame
 * @return Tool mass in kg
 */
double extractToolMass(const Force3d& f_gravity_B);


} // namespace ur_admittance_controller