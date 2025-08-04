// LROM algorithm implementation - Yu et al. IEEE Sensors Journal 2022
#include "wrench_calibration_algorithm.hpp"

namespace ur_admittance_controller {

// LROM constrained least squares (Eq. 24-34 in Yu et al.)
Result<Vector3d> estimateGravitationalForceInBaseFrame(
    const std::vector<CalibrationSample>& samples)
{
    if (samples.size() < 6) {
        return tl::unexpected(MakeError(ErrorCode::kInvalidConfiguration,
                                       "LROM requires at least 6 samples"));
    }

    const size_t n = samples.size();
    const size_t rows = 3 * n;

    // Build constraint matrices (Eq. 26)
    Eigen::MatrixXd A6 = Eigen::MatrixXd::Zero(rows, 6);
    Eigen::MatrixXd A9 = Eigen::MatrixXd::Zero(rows, 9);

    for (size_t i = 0; i < n; ++i) {
        const size_t row_offset = 3 * i;
        const auto& force = samples[i].F_S_S_raw.head<3>();
        const auto& R_TB = samples[i].X_TB.rotation();

        A6.block<3, 3>(row_offset, 0) = -R_TB;
        A6.block<3, 3>(row_offset, 3) = -Matrix3d::Identity();

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
        return tl::unexpected(MakeError(ErrorCode::kCalibrationFailed,
            "Calibration matrix ill-conditioned. Need more diverse poses."));
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
    const Vector3d gravity = x6(2) < 0 ? x6.head<3>() : Vector3d(-x6.head<3>());

    return gravity;
}

// Procrustes alignment (Eq. 37-39)
Result<std::pair<Matrix3d, Vector3d>> estimateSensorRotationAndForceBias(
    const std::vector<CalibrationSample>& samples,
    const Vector3d& gravity_in_base)
{
    const size_t n = samples.size();

    // Compute averages (Eq. 37)
    Vector3d force_readings_avg = Vector3d::Zero();
    Matrix3d rotation_TB_avg = Matrix3d::Zero();

    for (const auto& sample : samples) {
        force_readings_avg += sample.F_S_S_raw.head<3>();
        rotation_TB_avg += sample.X_TB.rotation();
    }
    force_readings_avg /= static_cast<double>(n);
    rotation_TB_avg /= static_cast<double>(n);

    // Build D matrix (Eq. 38)
    Matrix3d D = Matrix3d::Zero();
    for (const auto& sample : samples) {
        const Vector3d force_centered = sample.F_S_S_raw.head<3>() - force_readings_avg;
        const Matrix3d rotation_centered = sample.X_TB.rotation() - rotation_TB_avg;
        D += rotation_centered * gravity_in_base * force_centered.transpose();
    }

    // SVD solution for rotation
    Eigen::JacobiSVD<Matrix3d> svd(D, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Matrix3d correction = Matrix3d::Identity();
    const double det = (svd.matrixU() * svd.matrixV().transpose()).determinant();
    if (det < 0) {
        correction(2, 2) = -1;
    }

    const Matrix3d R_SE = svd.matrixU() * correction * svd.matrixV().transpose();

    // Force bias (Eq. 39)
    const Vector3d force_bias =
        force_readings_avg - (R_SE * rotation_TB_avg * gravity_in_base);

    return std::make_pair(R_SE, force_bias);
}


// Least squares for COM and torque bias (Eq. 43-45)
Result<std::pair<Vector3d, Vector3d>> estimateCOMAndTorqueBias(
    const std::vector<CalibrationSample>& samples,
    const Vector3d& force_bias)
{
    const size_t n = samples.size();
    const size_t rows = 3 * n;

    // Build linear system: C * y = b (Eq. 44)
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(rows, 6);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(rows);

    for (size_t i = 0; i < n; ++i) {
        const auto& sample = samples[i];
        const size_t row_offset = 3 * i;

        const Vector3d force = sample.F_S_S_raw.head<3>();
        const Vector3d torque = sample.F_S_S_raw.tail<3>();

        // Compensated force (Eq. 43)
        const Vector3d force_compensated = -(force - force_bias);

        // Skew-symmetric matrix
        Matrix3d skew_matrix;
        skew_matrix <<           0, -force_compensated(2),  force_compensated(1),
                       force_compensated(2),           0, -force_compensated(0),
                      -force_compensated(1),  force_compensated(0),           0;
        C.block<3, 3>(row_offset, 0) = skew_matrix;
        C.block<3, 3>(row_offset, 3) = Matrix3d::Identity();

        b.segment<3>(row_offset) = torque;
    }

    // Solve least squares (Eq. 45)
    const auto CtC = C.transpose() * C;
    const auto Ctb = C.transpose() * b;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd_CtC(CtC);
    double cond_CtC = svd_CtC.singularValues()(0) / svd_CtC.singularValues()(svd_CtC.singularValues().size()-1);

    if (cond_CtC > 1e8) {
        return tl::unexpected(MakeError(ErrorCode::kCalibrationFailed,
            "Torque calibration matrix ill-conditioned."));
    }

    const Eigen::VectorXd solution = CtC.inverse() * Ctb;

    return std::make_pair(solution.head<3>(), solution.tail<3>());
}

// Tait-Bryan angles (Eq. 46-48)
Result<Matrix3d> estimateRobotInstallationBias(const Vector3d& gravity_in_base)
{
    const double fbx = gravity_in_base(0);
    const double fby = gravity_in_base(1);
    const double fbz = gravity_in_base(2);

    // Compute angles (Eq. 48)
    const double beta = std::atan2(fbx, fbz);
    const double alpha = std::atan2(-fby * std::cos(beta), fbz);

    // Build rotation matrix R_BG = Rz(0)*Ry(β)*Rx(α)
    Matrix3d R_BG;
    R_BG << std::cos(beta), std::sin(alpha) * std::sin(beta), std::cos(alpha) * std::sin(beta),
            0.0, std::cos(alpha), -std::sin(alpha),
            -std::sin(beta), std::sin(alpha) * std::cos(beta), std::cos(alpha) * std::cos(beta);

    return R_BG;
}

// Extracts tool mass: mg = ||Fb||
inline constexpr double kStandardGravity = 9.80665;  // CODATA standard [m/s²]

Result<double> extractToolMass(const Vector3d& gravity_in_base)
{
    return gravity_in_base.norm() / kStandardGravity;
}

} // namespace ur_admittance_controller