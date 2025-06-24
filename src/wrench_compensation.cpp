// Wrench compensation implementations - gravity/bias compensation and LROM calibration
#include "wrench_compensation.hpp"
#include <Eigen/SVD>
#include <rclcpp/logging.hpp>
#include <algorithm>

namespace ur_admittance_controller {

// Apply sensor bias correction
Wrench GravityCompensator::applyBiasCorrection(const Wrench& raw) const
{
    Wrench corrected;
    corrected.head<3>() = raw.head<3>() - params_.F_bias_P;
    corrected.tail<3>() = raw.tail<3>() - params_.T_bias_P;
    return corrected;
}

// Apply gravity compensation
Wrench GravityCompensator::applyGravityCompensation(const Wrench& wrench, const Transform& X_BP) const
{
    // Get rotation from end-effector to base
    const Matrix3d R_EB = X_BP.rotation();
    
    // Compute gravity force in payload frame (Yu et al. Eq. 10)
    // F_gravity_P = R_PP * R_PE * R_EB * F_gravity_B
    // Since P≈E for UR robots, R_PE ≈ I, so F_gravity_P = R_PP * R_EB * F_gravity_B
    const Vector3d F_gravity_P = params_.R_PP * R_EB * params_.F_gravity_B;
    
    // Subtract gravity effects from forces
    const Vector3d force_P = wrench.head<3>() - F_gravity_P;
    
    // Gravity torque: T = p_CoM × F_gravity
    const Vector3d gravity_torque = params_.p_CoM_P.cross(F_gravity_P);
    const Vector3d torque_P = wrench.tail<3>() - gravity_torque;
    
    // Combine and return
    Wrench compensated;
    compensated << force_P, torque_P;
    return compensated;
}

// Combined compensation method (for backward compatibility with calibration)
Wrench GravityCompensator::compensate(
    const Wrench& F_P_P_raw,
    const Transform& X_EB,
    const JointState&) const
{
    // Step 1: Apply bias correction
    const Wrench bias_corrected = applyBiasCorrection(F_P_P_raw);
    
    // Step 2: Apply gravity compensation
    return applyGravityCompensation(bias_corrected, X_EB);
}

// Convert vector to skew-symmetric matrix for cross product operations
Matrix3d GravityCompensator::skewSymmetric(const Vector3d& v) {
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
    auto [com_in_sensor, torque_bias] = estimateCOMAndTorqueBias(
        samples, gravity_in_base, rotation_s_to_e, force_bias);
    
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
        
        A6.block<3, 3>(row, 0) = -samples[i].X_EB.rotation();
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
// Bias = mean(F_measured - R_PP * R_EB * m*g)
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
        const Matrix3d R_EB = sample.X_EB.rotation();
        // Transform gravity to sensor frame for each sample's orientation
        const Vector3d gravity_in_sensor = rotation_s_to_e * R_EB * gravity_in_base;
        
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
    [[maybe_unused]] const Vector3d& gravity_in_base,
    [[maybe_unused]] const Matrix3d& rotation_s_to_e,
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
        
        C.block<3, 3>(row_idx, 0) = GravityCompensator::skewSymmetric(compensated_force);  // p_CoM cross product matrix
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
            sample.X_EB);
        
        force_error_sum += compensated.head<3>().squaredNorm();
        torque_error_sum += compensated.tail<3>().squaredNorm();
    }
    
    const size_t n = samples.size();
    const double force_rmse = std::sqrt(force_error_sum / n);
    const double torque_rmse = std::sqrt(torque_error_sum / n);
    
    return {force_rmse, torque_rmse};
}

// Utility functions for ROS2 message conversion and transformation

// Extract Eigen wrench from ROS message
Wrench extractWrench(const geometry_msgs::msg::WrenchStamped& msg) {
    Wrench w;
    // Direct extraction without ATI corrections (matching pulse_force_estimation)
    w << msg.wrench.force.x,
         msg.wrench.force.y, 
         msg.wrench.force.z,
         msg.wrench.torque.x,
         msg.wrench.torque.y, 
         msg.wrench.torque.z;
    return w;
}

// Transform wrench between frames
Wrench transformWrench(const Wrench& F_P_P, const Matrix3d& R_BP) {
    Wrench F_P_B;
    F_P_B.head<3>() = R_BP * F_P_P.head<3>();  // Transform force
    F_P_B.tail<3>() = R_BP * F_P_P.tail<3>();  // Transform torque
    return F_P_B;
}

// Fill ROS message from Eigen wrench
void fillWrenchMsg(geometry_msgs::msg::Wrench& msg, const Wrench& wrench) {
    msg.force.x = wrench[0]; msg.force.y = wrench[1]; msg.force.z = wrench[2];
    msg.torque.x = wrench[3]; msg.torque.y = wrench[4]; msg.torque.z = wrench[5];
}

// YAML helper - write 3D vector
void writeVec3Yaml(YAML::Emitter& out, const char* key, const Vector3d& v) {
    out << YAML::Key << key << YAML::Value << YAML::Flow << YAML::BeginSeq << v.x() << v.y() << v.z() << YAML::EndSeq;
}

// YAML helper - read 3D vector
Vector3d readVec3Yaml(const YAML::Node& node, const std::string& key) {
    auto v = node[key].as<std::vector<double>>();
    return {v[0], v[1], v[2]};
}

// Factory function - create compensator from calibration file
std::unique_ptr<WrenchCompensator> createCompensator(
    const std::string& type,
    const std::string& calibration_file)
{
    if (type != "gravity_bias") 
        throw std::runtime_error("Unknown compensator type: " + type);
        
    // Load calibration parameters from YAML
    YAML::Node config = YAML::LoadFile(calibration_file);
    GravityCompensationParams params;
    params.tool_mass_kg = config["tool_mass_kg"].as<double>();
    params.p_CoM_P = readVec3Yaml(config, "tool_center_of_mass");
    params.F_gravity_B = readVec3Yaml(config, "gravity_in_base_frame");
    params.F_bias_P = readVec3Yaml(config, "force_bias");
    params.T_bias_P = readVec3Yaml(config, "torque_bias");
    
    // Load 3x3 rotation matrix
    auto rot_yaml = config["rotation_sensor_to_endeffector"];
    for (int i = 0; i < 3; ++i) {
        auto row = rot_yaml[i].as<std::vector<double>>();
        params.R_PP.row(i) = Eigen::Map<const Eigen::RowVector3d>(row.data());
    }
    
    // Load quaternion [x,y,z,w] format
    auto q_yaml = config["quaternion_sensor_to_endeffector"].as<std::vector<double>>();
    params.quaternion_sensor_to_endeffector = {{q_yaml[0], q_yaml[1], q_yaml[2], q_yaml[3]}};
    
    params.num_poses_collected = config["num_poses"].as<size_t>();
    
    return std::make_unique<GravityCompensator>(params);
}

} // namespace ur_admittance_controller