// LROM algorithm implementation - Yu et al. IEEE Sensors Journal 2022
#include "wrench_calibration_node.hpp"

namespace ur_admittance_controller {

// LROM constrained least squares (Eq. 24-34 in Yu et al.)
// Internal helper functions
namespace {

Eigen::Vector3d estimate_gravity(
    const std::vector<CalibrationSample>& samples)
{
    if (samples.size() < 6) {
        throw std::runtime_error("LROM requires at least 6 samples");
    }

    const size_t n = samples.size();
    const size_t rows = 3 * n;

    // Build constraint matrices (Eq. 26)
    Eigen::MatrixXd A6 = Eigen::MatrixXd::Zero(rows, 6);
    Eigen::MatrixXd A9 = Eigen::MatrixXd::Zero(rows, 9);

    for (size_t i = 0; i < n; ++i) {
        const size_t row_offset = 3 * i;
        const auto& force = samples[i].wrench_raw.head<3>();
        const auto& R_TB = samples[i].transform_TB.rotation();

        A6.block<3, 3>(row_offset, 0) = -R_TB;
        A6.block<3, 3>(row_offset, 3) = -Eigen::Matrix3d::Identity();

        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                A9(row_offset + row, 3 * row + col) = force[col];
            }
        }
    }

    // SO(3) constraint (Eq. 32)
    const auto I9 = std::sqrt(3.0) * Eigen::Matrix<double, 9, 9>::Identity();

    Eigen::MatrixXd gram_A6 = A6.transpose() * A6;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_A6(gram_A6);
    double cond_A6 = svd_A6.singularValues()(0) / svd_A6.singularValues()(svd_A6.singularValues().size()-1);

    if (cond_A6 > 1e8) {
        throw std::runtime_error("Calibration matrix ill-conditioned. Need more diverse poses.");
    }

    const auto A6_inv = gram_A6.inverse();
    const auto H = A9 * I9 - A6 * A6_inv * A6.transpose() * A9 * I9;

    // Find minimum eigenvalue eigenvector (Eq. 33)
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H.transpose() * H,
                                          Eigen::ComputeThinU | Eigen::ComputeThinV);

    const auto& singular_values = svd.singularValues();
    int min_idx = 0;
    for (int i = 1; i < singular_values.size(); ++i) {
        if (singular_values[i] < singular_values[min_idx]) {
            min_idx = i;
        }
    }

    const auto y_opt = svd.matrixU().col(min_idx);
    const auto x6 = -(A6_inv * A6.transpose() * A9 * I9) * y_opt;

    // Ensure downward gravity (Eq. 34)
    const Eigen::Vector3d gravity = x6(2) < 0 ? x6.head<3>() : Eigen::Vector3d(-x6.head<3>());

    return gravity;
}

// Procrustes alignment (Eq. 37-39)
std::pair<Eigen::Matrix3d, Eigen::Vector3d> estimate_sensor_params(
    const std::vector<CalibrationSample>& samples,
    const Eigen::Vector3d& gravity_in_base)
{
    const size_t n = samples.size();

    // Compute averages (Eq. 37)
    Eigen::Vector3d force_readings_avg = Eigen::Vector3d::Zero();
    Eigen::Matrix3d rotation_TB_avg = Eigen::Matrix3d::Zero();

    for (const auto& sample : samples) {
        force_readings_avg += sample.wrench_raw.head<3>();
        rotation_TB_avg += sample.transform_TB.rotation();
    }
    force_readings_avg /= static_cast<double>(n);
    rotation_TB_avg /= static_cast<double>(n);

    // Build D matrix (Eq. 38)
    Eigen::Matrix3d D = Eigen::Matrix3d::Zero();
    for (const auto& sample : samples) {
        const Eigen::Vector3d force_centered = sample.wrench_raw.head<3>() - force_readings_avg;
        const Eigen::Matrix3d rotation_centered = sample.transform_TB.rotation() - rotation_TB_avg;
        D += rotation_centered * gravity_in_base * force_centered.transpose();
    }

    // SVD solution for rotation
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(D, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
    const double det = (svd.matrixU() * svd.matrixV().transpose()).determinant();
    if (det < 0) {
        correction(2, 2) = -1;
    }

    const Eigen::Matrix3d R_SE = svd.matrixU() * correction * svd.matrixV().transpose();

    // Force bias (Eq. 39)
    const Eigen::Vector3d force_bias =
        force_readings_avg - (R_SE * rotation_TB_avg * gravity_in_base);

    return std::make_pair(R_SE, force_bias);
}


// Least squares for COM and torque bias (Eq. 43-45)
std::pair<Eigen::Vector3d, Eigen::Vector3d> estimate_com_and_torque(
    const std::vector<CalibrationSample>& samples,
    const Eigen::Vector3d& force_bias)
{
    const size_t n = samples.size();
    const size_t rows = 3 * n;

    // Build linear system: C * y = b (Eq. 44)
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(rows, 6);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(rows);

    for (size_t i = 0; i < n; ++i) {
        const auto& sample = samples[i];
        const size_t row_offset = 3 * i;

        const Eigen::Vector3d force = sample.wrench_raw.head<3>();
        const Eigen::Vector3d torque = sample.wrench_raw.tail<3>();

        // Compensated force (Eq. 43)
        const Eigen::Vector3d force_compensated = -(force - force_bias);

        // Skew-symmetric matrix
        Eigen::Matrix3d skew_matrix;
        skew_matrix <<           0, -force_compensated(2),  force_compensated(1),
                       force_compensated(2),           0, -force_compensated(0),
                      -force_compensated(1),  force_compensated(0),           0;
        C.block<3, 3>(row_offset, 0) = skew_matrix;
        C.block<3, 3>(row_offset, 3) = Eigen::Matrix3d::Identity();

        b.segment<3>(row_offset) = torque;
    }

    // Solve least squares (Eq. 45)
    const auto CtC = C.transpose() * C;
    const auto Ctb = C.transpose() * b;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd_CtC(CtC);
    double cond_CtC = svd_CtC.singularValues()(0) / svd_CtC.singularValues()(svd_CtC.singularValues().size()-1);

    if (cond_CtC > 1e8) {
        throw std::runtime_error("Torque calibration matrix ill-conditioned.");
    }

    const Eigen::VectorXd solution = CtC.inverse() * Ctb;

    return std::make_pair(solution.head<3>(), solution.tail<3>());
}

} // anonymous namespace

// ============================================================================
// Main Calibration Pipeline
// ============================================================================

CalibrationResult compute_calibration(const std::vector<CalibrationSample>& samples) {
    // Step 1: Estimate gravitational force in base frame
    auto gravity = estimate_gravity(samples);
    
    // Step 2: Estimate sensor rotation and force bias
    auto [R_SE, force_bias] = estimate_sensor_params(samples, gravity);
    
    // Step 3: Estimate center of mass and torque bias
    auto [com, torque_bias] = estimate_com_and_torque(samples, force_bias);
    
    // Step 4: Calculate tool mass
    double tool_mass = gravity.norm() / constants::GRAVITY;
    
    return CalibrationResult{
        R_SE,
        gravity,
        force_bias,
        torque_bias,
        com,
        tool_mass
    };
}

} // namespace ur_admittance_controller