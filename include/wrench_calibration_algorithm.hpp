#pragma once

// LROM algorithm implementation for force/torque sensor calibration
// Based on: Yu et al. "Bias Estimation and Gravity Compensation for Wrist-Mounted Force/Torque Sensor"
// IEEE Sensors Journal, 2022

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
 * @return Pair of (gravity_vector_in_base_frame, rotation_sensor_to_endeffector)
 * @throws std::invalid_argument if insufficient samples provided
 */
std::pair<Vector3d, Matrix3d> estimateGravityAndRotation(
    const std::vector<CalibrationSample>& samples);

/**
 * @brief Estimates the constant force bias in the sensor frame
 * 
 * Implements force bias estimation from:
 * Yu et al. "Bias Estimation and Gravity Compensation for Wrist-Mounted Force/Torque Sensor"
 * IEEE Sensors Journal, 2022, Section III-B
 * 
 * @param samples Calibration samples with force readings and robot poses
 * @param gravity_in_base Estimated gravity vector in base frame (Fb from Step 1)
 * @param rotation_s_to_e Sensor-to-endeffector rotation (R_SE from Step 1)
 * @return Force bias vector in sensor frame
 */
Vector3d estimateForceBias(
    const std::vector<CalibrationSample>& samples,
    const Vector3d& gravity_in_base,
    const Matrix3d& rotation_s_to_e);

/**
 * @brief Estimates tool center of mass and torque bias in sensor frame
 * 
 * Implements the torque identification model from:
 * Yu et al. "Bias Estimation and Gravity Compensation for Wrist-Mounted Force/Torque Sensor"
 * IEEE Sensors Journal, 2022, Section III-C
 * 
 * @param samples Calibration samples with force/torque readings
 * @param force_bias Previously estimated force bias (F0 from Step 2)
 * @return Pair of (center_of_mass_position, torque_bias) in sensor frame
 */
std::pair<Vector3d, Vector3d> estimateCOMAndTorqueBias(
    const std::vector<CalibrationSample>& samples,
    const Vector3d& force_bias);

/**
 * @brief Estimates the rotation from gravity frame to robot base frame
 * 
 * Implements robot installation bias estimation from:
 * Yu et al. "Bias Estimation and Gravity Compensation for Wrist-Mounted Force/Torque Sensor" 
 * IEEE Sensors Journal, 2022, Section III-D
 * 
 * @param gravity_in_base Gravity vector in base frame (Fb from Step 1)
 * @return Rotation matrix from gravity frame to base frame
 * @note Potential singularity when robot base is horizontal (F_z ≈ 0)
 */
Matrix3d estimateRobotInstallationBias(const Vector3d& gravity_in_base);

/**
 * @brief Extracts tool mass from gravity vector magnitude
 * 
 * From Equation (48): mg = ||Fb||
 * 
 * @param gravity_in_base Gravity vector in base frame
 * @return Tool mass in kg
 */
double extractToolMass(const Vector3d& gravity_in_base);


} // namespace ur_admittance_controller