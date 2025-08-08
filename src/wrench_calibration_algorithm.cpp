#include "wrench_calibration_node.hpp"

using namespace ur_admittance_controller;

namespace ur_admittance_controller {

Force WrenchCalibrationNode::estimateGravitationalForceInBaseFrame(const std::vector<CalibrationSample>& samples) {
    const size_t n = samples.size();
    const size_t rows = 3 * n;

    Eigen::MatrixXd A6 = Eigen::MatrixXd::Zero(rows, 6);
    Eigen::MatrixXd A9 = Eigen::MatrixXd::Zero(rows, 9);

    for (size_t i = 0; i < n; ++i) {
        const size_t row_offset = 3 * i;
        const Eigen::Vector3d& force = samples[i].wrench_raw.head<3>();
        const Eigen::Matrix3d& R_TB = samples[i].transform_TB.rotation();

        A6.block<3, 3>(row_offset, 0) = -R_TB;
        A6.block<3, 3>(row_offset, 3) = -Eigen::Matrix3d::Identity();

        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                A9(row_offset + row, 3 * row + col) = force[col];
            }
        }
    }

    const Eigen::Matrix<double, 9, 9> I9 = std::sqrt(3.0) * Eigen::Matrix<double, 9, 9>::Identity();

    const Eigen::MatrixXd A6_inv = (A6.transpose() * A6).inverse();
    const Eigen::MatrixXd H = A9 * I9 - A6 * A6_inv * A6.transpose() * A9 * I9;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H.transpose() * H,
                                          Eigen::ComputeThinU | Eigen::ComputeThinV);

    const Eigen::VectorXd& singular_values = svd.singularValues();
    auto min_iter = std::min_element(singular_values.data(), singular_values.data() + singular_values.size());
    int min_idx = std::distance(singular_values.data(), min_iter);

    const Eigen::VectorXd y_opt = svd.matrixU().col(min_idx);
    const Eigen::VectorXd x6 = -(A6_inv * A6.transpose() * A9 * I9) * y_opt;

    const Eigen::Vector3d gravity_vec = x6(2) < 0 ? x6.head<3>() : Eigen::Vector3d(-x6.head<3>());

    return Force(gravity_vec);
}

std::pair<Eigen::Matrix3d, Force> WrenchCalibrationNode::estimateSensorRotationAndForceBias(const std::vector<CalibrationSample>& samples, const Force& gravity_in_base) {
    const size_t n = samples.size();

    Eigen::Vector3d force_readings_avg = std::accumulate(
        samples.begin(), samples.end(), Eigen::Vector3d::Zero().eval(),
        [](const Eigen::Vector3d& acc, const CalibrationSample& s) {
            return (acc + s.wrench_raw.head<3>()).eval();
        }) / static_cast<double>(n);
    
    Eigen::Matrix3d rotation_TB_avg = std::accumulate(
        samples.begin(), samples.end(), Eigen::Matrix3d::Zero().eval(),
        [](const Eigen::Matrix3d& acc, const CalibrationSample& s) {
            return (acc + s.transform_TB.rotation()).eval();
        }) / static_cast<double>(n);

    Eigen::Matrix3d D = Eigen::Matrix3d::Zero();
    for (const CalibrationSample& sample : samples) {
        const Eigen::Vector3d force_centered = sample.wrench_raw.head<3>() - force_readings_avg;
        const Eigen::Matrix3d rotation_centered = sample.transform_TB.rotation() - rotation_TB_avg;
        D += rotation_centered * gravity_in_base.N * force_centered.transpose();
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(D, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
    const double det = (svd.matrixU() * svd.matrixV().transpose()).determinant();
    if (det < 0) {
        correction(2, 2) = -1;
    }

    const Eigen::Matrix3d R_SE = svd.matrixU() * correction * svd.matrixV().transpose();

    const Eigen::Vector3d force_bias_vec =
        force_readings_avg - (R_SE * rotation_TB_avg * gravity_in_base.N);

    return std::make_pair(R_SE, Force(force_bias_vec));
}

std::pair<Eigen::Vector3d, Torque> WrenchCalibrationNode::estimateCOMAndTorqueBias(const std::vector<CalibrationSample>& samples, const Force& force_bias) {
    const size_t n = samples.size();
    const size_t rows = 3 * n;

    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(rows, 6);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(rows);

    for (size_t i = 0; i < n; ++i) {
        const CalibrationSample& sample = samples[i];
        const size_t row_offset = 3 * i;

        const Eigen::Vector3d force = sample.wrench_raw.head<3>();
        const Eigen::Vector3d torque = sample.wrench_raw.tail<3>();

        const Eigen::Vector3d force_compensated = -(force - force_bias.N);
        
        C.block<3, 3>(row_offset, 0) = makeSkewSymmetric(force_compensated);
        C.block<3, 3>(row_offset, 3) = Eigen::Matrix3d::Identity();

        b.segment<3>(row_offset) = torque;
    }

    const Eigen::VectorXd solution = (C.transpose() * C).inverse() * (C.transpose() * b);

    return std::make_pair(solution.head<3>(), Torque(solution.tail<3>()));
}

}