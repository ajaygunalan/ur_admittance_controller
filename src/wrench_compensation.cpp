// Wrench compensation implementations - gravity/bias compensation and LROM calibration
#include "wrench_compensation.hpp"
#include <Eigen/SVD>
#include <rclcpp/logging.hpp>
#include <algorithm>

namespace ur_admittance_controller {

// Yu et al. bias and gravity compensation (matching pulse_force_estimation exactly)
Wrench GravityCompensator::compensate(
    const Wrench& f_raw_s,
    const Transform& X_EB,
    const JointState&) const
{
    // Extract rotation matrix R_EB (pulse_force_estimation line 227)
    const Matrix3d R_EB = X_EB.rotation();
    
    // Compute gravity force in sensor frame (pulse_force_estimation line 227)
    const Vector3d f_grav_s = params_.R_PP * R_EB * params_.F_gravity_B;
    
    // Apply Yu et al. compensation (pulse_force_estimation line 232-233)
    Wrench ft_proc_s;
    
    // Force compensation: f_compensated = f_raw - f_grav - f_bias
    ft_proc_s.head<3>() = f_raw_s.head<3>() - f_grav_s - params_.F_bias_P;
    
    // Torque compensation: t_compensated = t_raw - (p_CoM × f_grav) - t_bias  
    const Vector3d gravity_torque = skew_symmetric(params_.p_CoM_P) * f_grav_s;
    ft_proc_s.tail<3>() = f_raw_s.tail<3>() - gravity_torque - params_.T_bias_P;
    
    return ft_proc_s;
}

// Convert vector to skew-symmetric matrix for cross product operations
Matrix3d GravityCompensator::skew_symmetric(const Vector3d& v) {
    return (Matrix3d() << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0).finished();
}

// LROMCalibrator - Limited Robot Orientation Method for F/T sensor calibration
CalibrationResult LROMCalibrator::calibrate(
    const std::vector<CalibrationSample>& samples)
{
    CalibrationResult result;
    
    // Validate input - expecting all samples (10 per pose)
    if (samples.size() < NUM_CALIBRATION_POSES * SAMPLES_PER_POSE) {
        result.error_message = "Insufficient calibration samples";
        return result;
    }
    
    // LROM algorithm steps (Yu et al. paper)
    auto [gravity_in_base, rotation_s_to_e] = estimateGravityAndRotation(samples);
    auto force_bias = estimateForceBias(samples, gravity_in_base, rotation_s_to_e);
    auto [com_in_sensor, torque_bias] = estimateCOMAndTorqueBias(samples, force_bias);
    
    // Fill result
    result.params.F_gravity_B = gravity_in_base;
    result.params.R_PP = rotation_s_to_e;
    result.params.F_bias_P = force_bias;
    result.params.p_CoM_P = com_in_sensor;
    result.params.T_bias_P = torque_bias;
    result.params.tool_mass_kg = gravity_in_base.norm() / 9.81;
    result.params.num_poses_collected = samples.size();
    
    // Convert rotation matrix to quaternion [x,y,z,w] format
    Eigen::Quaterniond q(rotation_s_to_e);
    result.params.quaternion_sensor_to_endeffector = {{q.x(), q.y(), q.z(), q.w()}};
    
    // Validate results
    auto [force_rmse, torque_rmse] = computeResiduals(samples, result.params);
    result.force_fit_rmse = force_rmse;
    result.torque_fit_rmse = torque_rmse;
    
    result.success = true;
    
    return result;
}

// Step 1: Estimate gravity vector and sensor-to-endeffector rotation
std::pair<Vector3d, Matrix3d> LROMCalibrator::estimateGravityAndRotation(
    const std::vector<CalibrationSample>& samples) const
{
    const size_t n = samples.size();
    
    // Build constraint matrices
    Eigen::MatrixXd A6 = Eigen::MatrixXd::Zero(3 * n, 6);
    Eigen::MatrixXd A9 = Eigen::MatrixXd::Zero(3 * n, 9);
    
    for (size_t i = 0; i < n; ++i) {
        const auto row = 3 * i;
        const auto& force = samples[i].F_P_P_raw.head<3>();
        
        A6.block<3, 3>(row, 0) = -samples[i].X_PB.rotation();
        A6.block<3, 3>(row, 3) = -Matrix3d::Identity();
        
        // Build A9 following Yu et al.
        for (int j = 0; j < 3; ++j) {
            A9.block<1, 3>(row + j, 3 * j) = force.transpose();
        }
    }
    
    // Constrained least squares with SO(3) constraint
    const auto I9 = std::sqrt(3.0) * Eigen::MatrixXd::Identity(9, 9);
    const auto A6_inv = (A6.transpose() * A6).inverse();
    const auto H = A9 * I9 - A6 * A6_inv * A6.transpose() * A9 * I9;
    
    // Find minimum eigenvalue eigenvector
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H.transpose() * H, Eigen::ComputeThinU);
    const auto& sigma = svd.singularValues();
    const int min_idx = std::distance(sigma.data(), 
        std::min_element(sigma.data(), sigma.data() + sigma.size()));
    
    // Recover solution
    const auto y_opt = svd.matrixU().col(min_idx);
    const auto x6 = -(A6_inv * A6.transpose() * A9 * I9) * y_opt;
    const auto x9 = I9 * y_opt;
    
    // Extract gravity (ensure downward)
    const Vector3d gravity = x6(2) < 0 ? x6.head<3>() : Vector3d(-x6.head<3>());
    
    // Reconstruct and project rotation to SO(3)
    Matrix3d R;
    R << x9.segment<3>(0).transpose(),
         x9.segment<3>(3).transpose(), 
         x9.segment<3>(6).transpose();
    
    Eigen::JacobiSVD<Matrix3d> rot_svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    
    return {gravity, rot_svd.matrixU() * rot_svd.matrixV().transpose()};
}

// Step 2: Estimate constant force bias in sensor frame
// Bias = mean(F_measured - R_PP * R_BP * m*g)
// Critical: Uses ALL samples (not per-pose averages) for better statistical accuracy
Vector3d LROMCalibrator::estimateForceBias(
    const std::vector<CalibrationSample>& samples,
    const Vector3d& gravity_in_base,
    const Matrix3d& rotation_s_to_e) const
{
    Vector3d force_sum = Vector3d::Zero();
    Vector3d gravity_sum = Vector3d::Zero();
    
    for (const auto& sample : samples) {
        const Vector3d force = sample.F_P_P_raw.head<3>();
        const Matrix3d R_PB = sample.X_PB.rotation();
        // Transform gravity to sensor frame for each sample's orientation
        const Vector3d gravity_in_sensor = rotation_s_to_e * R_PB * gravity_in_base;
        
        force_sum += force;
        gravity_sum += gravity_in_sensor;
    }
    
    const size_t n = samples.size();
    return (force_sum - gravity_sum) / static_cast<double>(n);
}

// Step 3: Estimate tool center of mass and torque bias
// Solves: T = p_CoM × F_gravity + T_bias via least squares
std::pair<Vector3d, Vector3d> LROMCalibrator::estimateCOMAndTorqueBias(
    const std::vector<CalibrationSample>& samples,
    const Vector3d& force_bias) const
{
    const size_t n = samples.size();
    
    // Linear system: torque = COM × force + bias
    // Matrix form: [F×  I₃] × [p_CoM; T_bias] = T_measured
    Eigen::MatrixXd C(3 * n, 6);
    Eigen::VectorXd b(3 * n);
    
    for (size_t i = 0; i < n; ++i) {
        const auto& sample = samples[i];
        const Vector3d torque = sample.F_P_P_raw.tail<3>();
        const Vector3d force  = sample.F_P_P_raw.head<3>();
        
        // Critical: Negate to get reaction force at sensor (Newton's 3rd law)
        const Vector3d compensated_force = -(force - force_bias);
        
        const size_t row_idx = 3 * i;
        
        C.block<3, 3>(row_idx, 0) = GravityCompensator::skew_symmetric(compensated_force);  // p_CoM cross product matrix
        C.block<3, 3>(row_idx, 3) = Matrix3d::Identity();                                  // torque bias terms
        b.segment<3>(row_idx)      = torque;                                               // measured torques
    }
    
    // Solve normal equations: x = (C'C)⁻¹C'b
    const Eigen::VectorXd solution = (C.transpose() * C).inverse() * C.transpose() * b;
    
    const Vector3d com_in_sensor = solution.head<3>();  // First 3: center of mass
    const Vector3d torque_bias   = solution.tail<3>();  // Last 3: torque bias
    
    return {com_in_sensor, torque_bias};
}

// Validate calibration by computing residual errors
std::pair<double, double> LROMCalibrator::computeResiduals(
    const std::vector<CalibrationSample>& samples,
    const GravityCompensationParams& params) const
{
    double force_error_sum = 0.0;
    double torque_error_sum = 0.0;
    
    GravityCompensator compensator(params);
    
    for (const auto& sample : samples) {
        const Wrench compensated = compensator.compensate(
            sample.F_P_P_raw,
            sample.X_PB);
        
        force_error_sum += compensated.head<3>().squaredNorm();
        torque_error_sum += compensated.tail<3>().squaredNorm();
    }
    
    const size_t n = samples.size();
    const double force_rmse = std::sqrt(force_error_sum / n);
    const double torque_rmse = std::sqrt(torque_error_sum / n);
    
    return {force_rmse, torque_rmse};
}

// Note: ROS message conversion functions removed - inlined at call sites for clarity



} // namespace ur_admittance_controller