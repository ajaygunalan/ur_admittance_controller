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

#include <vector>
#include <utility>
#include <stdexcept>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/SVD>

#include <utilities/types.hpp>
#include <utilities/error.hpp>

namespace ur_admittance_controller {

// Estimate gravity in base frame using LROM constrained least squares (Yu et al. Eq. 24-34)
// Returns F_b from calibration samples without knowing R_SE
Result<Force3d> estimateGravitationalForceInBaseFrame(
    const std::vector<CalibrationSample>& samples);

// Estimate sensor rotation R_SE and force bias using Procrustes alignment (Yu et al. Eq. 37-39)
// Solves 3D point set alignment from calibration samples and gravity estimate
Result<std::pair<Matrix3d, Force3d>> estimateSensorRotationAndForceBias(
    const std::vector<CalibrationSample>& samples,
    const Force3d& f_gravity_B);

// Estimate tool center of mass and torque bias (Yu et al. Section III-C)
// Uses torque samples and previously estimated force bias
Result<std::pair<Vector3d, Torque3d>> estimateCOMAndTorqueBias(
    const std::vector<CalibrationSample>& samples,
    const Force3d& f_bias_S);

// Estimate robot installation bias - rotation from gravity to base frame (Yu et al. Section III-D)
// Warning: Singularity when robot base is horizontal (F_z ≈ 0)
Result<Matrix3d> estimateRobotInstallationBias(const Force3d& f_gravity_B);

// Extract tool mass from gravity magnitude: mg = ||Fb|| (Eq. 48)
Result<double> extractToolMass(const Force3d& f_gravity_B);

} // namespace ur_admittance_controller