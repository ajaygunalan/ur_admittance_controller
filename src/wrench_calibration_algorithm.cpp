/**
 * @file wrench_calibration_algorithm.cpp
 * @brief LROM algorithm implementation for force/torque sensor calibration
 * 
 * This file implements the Limited Robot Orientation Method (LROM) and related
 * algorithms for force/torque sensor bias estimation and gravity compensation.
 * All functions are based on the mathematical formulations presented in the
 * reference paper below.
 * 
 * @reference Yu, Yongqiang, Ran Shi, and Yunjiang Lou. 
 *           "Bias Estimation and Gravity Compensation for Wrist-Mounted Force/Torque Sensor"
 *           IEEE Sensors Journal, vol. 22, no. 18, pp. 17625-17634, 2022.
 *           DOI: 10.1109/JSEN.2022.3195503
 * 
 * @author Pulse Robotics Team (Ajay and Mardava)
 * @date 2024-2025
 */

#include "calibration_types.hpp"
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <stdexcept>
#include <cmath>

namespace ur_admittance_controller {

/**
 * @brief Estimates gravity vector and sensor-to-endeffector rotation using LROM algorithm
 * 
 * Implements the Limited Robot Orientation Method (LROM) as described in Section III-A.2.
 * Uses constrained least squares with SO(3) normalization (Equations 26, 32-34).
 * 
 * @param samples Calibration samples containing force readings and robot poses
 * @return Pair of (gravity_vector_in_base_frame, rotation_sensor_to_endeffector)
 * @throws std::invalid_argument if insufficient samples provided
 */
std::pair<Vector3d, Matrix3d> estimateGravityAndRotation(
    const std::vector<CalibrationSample>& samples)
{
    // Validate minimum samples for overdetermined system
    if (samples.size() < 6) {
        throw std::invalid_argument("LROM requires at least 6 samples for unique solution");
    }

    const size_t n = samples.size();
    const size_t rows = 3 * n;  // 3 force components per sample

    // Build constraint matrices A6 and A9 from Eq. (26) in Yu et al.
    // A6 relates [gravity_base; force_bias] to force measurements
    // A9 relates rotation matrix elements to force measurements
    Eigen::MatrixXd A6 = Eigen::MatrixXd::Zero(rows, 6);
    Eigen::MatrixXd A9 = Eigen::MatrixXd::Zero(rows, 9);

    // Populate matrices for each force measurement
    for (size_t i = 0; i < n; ++i) {
        const size_t row_offset = 3 * i;
        const auto& force = samples[i].F_S_S_raw.head<3>();
        const auto& R_EB = samples[i].X_EB.rotation();

        // A6 left block: -R_EB (negative rotation from base to end-effector)
        A6.block<3, 3>(row_offset, 0) = -R_EB;
        
        // A6 right block: -I (negative identity for force bias term)
        A6.block<3, 3>(row_offset, 3) = -Matrix3d::Identity();

        // A9: Force readings arranged for rotation matrix estimation
        // Matches indexing from old implementation exactly
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                A9(row_offset + row, 3 * row + col) = force[col];
            }
        }
    }

    // Solve constrained least squares problem with SO(3) constraint (Eq. 32)
    // Constraint: ||sqrt(3) * I9 * y||² = 1 ensures rotation matrix normalization
    const double sqrt3 = std::sqrt(3.0);
    const auto I9 = sqrt3 * Eigen::Matrix<double, 9, 9>::Identity();
    
    // Compute (A6ᵀA6)⁻¹ for null space projection
    const auto A6_inv = (A6.transpose() * A6).inverse();
    
    // Form H matrix: projects A9 onto null space of A6
    const auto H = A9 * I9 - A6 * A6_inv * A6.transpose() * A9 * I9;

    // Find minimum eigenvalue eigenvector of HᵀH via SVD
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H.transpose() * H, 
                                          Eigen::ComputeThinU | Eigen::ComputeThinV);
    
    // Manual minimum search to match old implementation exactly
    const auto& singular_values = svd.singularValues();
    int min_idx = 0;
    for (int i = 1; i < singular_values.size(); ++i) {
        if (singular_values[i] < singular_values[min_idx]) {
            min_idx = i;
        }
    }

    // Extract optimal solution from eigenvector (Eq. 33)
    const auto y_opt = svd.matrixU().col(min_idx);
    
    // Recover x6 (gravity + bias) and x9 (vectorized rotation)
    const auto x6 = -(A6_inv * A6.transpose() * A9 * I9) * y_opt;
    const auto x9 = I9 * y_opt;

    // Extract gravity vector ensuring downward direction (Eq. 34)
    // Convention: gravity points down (negative z in base frame)
    const Vector3d gravity = x6(2) < 0 ? x6.head<3>() : Vector3d(-x6.head<3>());

    // Reconstruct 3x3 rotation matrix from vectorized form
    Matrix3d R_raw;
    R_raw << x9.segment<3>(0).transpose(),  // Row 1
             x9.segment<3>(3).transpose(),  // Row 2
             x9.segment<3>(6).transpose();  // Row 3

    // Project to SO(3) to ensure valid rotation matrix
    // Removes numerical errors: ensures orthogonality and det(R) = 1
    Eigen::JacobiSVD<Matrix3d> rot_svd(R_raw, 
                                       Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Matrix3d rotation = rot_svd.matrixU() * rot_svd.matrixV().transpose();

    // Note: Old code doesn't handle reflection case (det < 0)
    // Maintaining exact compatibility by not adding this check

    return {gravity, rotation};
}

/**
 * @brief Estimates the constant force bias in the sensor frame
 * 
 * Implements force bias estimation as described in Section III-B.
 * - Equation (37): Computes averages of force readings and rotation matrices
 * - Equation (39): f_bias_s = force_avg - R_SE * R_EB_avg * f_gravity_base
 * 
 * @param samples Calibration samples with force readings and robot poses
 * @param gravity_in_base Estimated gravity vector in base frame (Fb from Step 1)
 * @param rotation_s_to_e Sensor-to-endeffector rotation (R_SE from Step 1)
 * @return Force bias vector in sensor frame
 */
Vector3d estimateForceBias(
    const std::vector<CalibrationSample>& samples,
    const Vector3d& gravity_in_base,
    const Matrix3d& rotation_s_to_e)
{
    // ===== Step 1: Average force readings (Eq. 37) =====
    // Compute: F̄ = (1/n) * Σ F_i
    Vector3d force_readings_avg = Vector3d::Zero();
    
    for (const auto& sample : samples) {
        force_readings_avg += sample.F_S_S_raw.head<3>();
    }
    force_readings_avg /= static_cast<double>(samples.size());

    // ===== Step 2: Average robot transformations (Eq. 37) =====
    // Compute: R̄_EB = (1/n) * Σ R_EB_i
    Matrix3d rotation_EB_avg = Matrix3d::Zero();
    
    for (const auto& sample : samples) {
        rotation_EB_avg += sample.X_EB.rotation();
    }
    rotation_EB_avg /= static_cast<double>(samples.size());

    // ===== Step 3: Calculate force bias (Eq. 39) =====
    // F0 = F̄ - R_SE * R̄_EB * Fb
    // Where:
    //   F0 = force bias in sensor frame
    //   F̄ = average of force readings
    //   R_SE = sensor-to-endeffector rotation
    //   R̄_EB = average base-to-endeffector rotation
    //   Fb = gravity vector in base frame
    const Vector3d force_bias = 
        force_readings_avg - (rotation_s_to_e * rotation_EB_avg * gravity_in_base);

    return force_bias;
}


/**
 * @brief Estimates tool center of mass and torque bias in sensor frame
 * 
 * Implements the torque identification model as described in Section III-C.
 * - Equation (43): T = -(F - F0)× * p_g + T0
 * - Equation (44-45): Least squares solution for [p_g; T0]
 * 
 * The negative sign on force is critical: it represents the reaction force
 * at the sensor due to the tool's weight (Newton's 3rd law).
 * 
 * @param samples Calibration samples with force/torque readings
 * @param force_bias Previously estimated force bias (F0 from Step 2)
 * @return Pair of (center_of_mass_position, torque_bias) in sensor frame
 */
std::pair<Vector3d, Vector3d> estimateCOMAndTorqueBias(
    const std::vector<CalibrationSample>& samples,
    const Vector3d& force_bias)
{
    const size_t n = samples.size();
    const size_t rows = 3 * n;  // 3 torque components per sample

    // ===== Build linear system from Eq. (44) =====
    // System: C * y = b
    // Where:
    //   C = [-(F1-F0)×  I3]
    //       [-(F2-F0)×  I3]
    //       [    ...      ]
    //   y = [p_g; T0]  (6×1 vector: COM position + torque bias)
    //   b = [T1; T2; ...] (measured torques)
    
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(rows, 6);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(rows);

    // Populate matrices for each measurement
    for (size_t i = 0; i < n; ++i) {
        const auto& sample = samples[i];
        const size_t row_offset = 3 * i;
        
        // Extract measurements
        const Vector3d force = sample.F_S_S_raw.head<3>();
        const Vector3d torque = sample.F_S_S_raw.tail<3>();
        
        // Bias-compensated force with sign convention from Eq. (43)
        // Negative sign: reaction force at sensor due to tool weight
        const Vector3d force_compensated = -(force - force_bias);
        
        // C matrix left block: skew-symmetric matrix for cross product
        Matrix3d skew_matrix;
        skew_matrix <<           0, -force_compensated(2),  force_compensated(1),
                       force_compensated(2),           0, -force_compensated(0),
                      -force_compensated(1),  force_compensated(0),           0;
        C.block<3, 3>(row_offset, 0) = skew_matrix;
        
        // C matrix right block: identity for torque bias terms
        C.block<3, 3>(row_offset, 3) = Matrix3d::Identity();
        
        // b vector: measured torques
        b.segment<3>(row_offset) = torque;
    }

    // ===== Solve least squares problem (Eq. 45) =====
    // y* = (C'C)^(-1) * C'b
    const auto CtC = C.transpose() * C;
    const auto Ctb = C.transpose() * b;
    const Eigen::VectorXd solution = CtC.inverse() * Ctb;

    // ===== Extract results =====
    const Vector3d center_of_mass = solution.head<3>();  // p_g
    const Vector3d torque_bias = solution.tail<3>();     // T0

    return {center_of_mass, torque_bias};
}

/**
 * @brief Estimates the rotation from gravity frame to robot base frame
 * 
 * Implements robot installation bias estimation as described in Section III-D.
 * - Equation (46): Rotation matrix in Tait-Bryan angles R_BG = Rz(0)Ry(β)Rx(α)
 * - Equation (48): Extract α and β angles from gravity vector components
 * 
 * This accounts for the robot base not being perfectly aligned with gravity,
 * which is common in industrial installations (see Fig. 1 in the paper).
 * 
 * @param gravity_in_base Gravity vector in base frame (Fb from Step 1)
 * @return Rotation matrix from gravity frame to base frame
 * @note Potential singularity when robot base is horizontal (F_z ≈ 0)
 */
Matrix3d estimateRobotInstallationBias(const Vector3d& gravity_in_base)
{
    // ===== Extract gravity components =====
    const double fbx = gravity_in_base(0);  // X component
    const double fby = gravity_in_base(1);  // Y component  
    const double fbz = gravity_in_base(2);  // Z component
    
    // ===== Compute Tait-Bryan angles (Eq. 48) =====
    // β = arctan2(fbx, fbz)
    const double beta = std::atan2(fbx, fbz);
    
    // α = arctan2(-fby*cos(β), fbz)
    // Note: Potential singularity if fbz ≈ 0 (robot base horizontal)
    const double alpha = std::atan2(-fby * std::cos(beta), fbz);
    
    // ===== Build rotation matrix R_BG (Eq. 46) =====
    // R_BG = Rz(0) * Ry(β) * Rx(α)
    // Since γ = 0, the matrix simplifies to:
    Matrix3d R_BG;
    
    // Row 1
    R_BG(0, 0) = std::cos(beta);                                    // cos(β)
    R_BG(0, 1) = std::sin(alpha) * std::sin(beta);                 // sin(α)sin(β)
    R_BG(0, 2) = std::cos(alpha) * std::sin(beta);                 // cos(α)sin(β)
    
    // Row 2
    R_BG(1, 0) = 0.0;                                               // 0
    R_BG(1, 1) = std::cos(alpha);                                   // cos(α)
    R_BG(1, 2) = -std::sin(alpha);                                  // -sin(α)
    
    // Row 3
    R_BG(2, 0) = -std::sin(beta);                                   // -sin(β)
    R_BG(2, 1) = std::sin(alpha) * std::cos(beta);                 // sin(α)cos(β)
    R_BG(2, 2) = std::cos(alpha) * std::cos(beta);                 // cos(α)cos(β)
    
    return R_BG;
}

/**
 * @brief Extracts tool mass from gravity vector magnitude
 * 
 * Implements Equation (48): mg = ||Fb||
 * 
 * @param gravity_in_base Gravity vector in base frame
 * @return Tool mass in kg
 */
double extractToolMass(const Vector3d& gravity_in_base)
{
    const double gravity_magnitude = gravity_in_base.norm();
    const double g = 9.81;  // Standard gravity [m/s²]
    return gravity_magnitude / g;
}

} // namespace ur_admittance_controller