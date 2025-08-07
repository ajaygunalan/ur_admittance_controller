#include "wrench_calibration_node.hpp"

using namespace ur_admittance_controller;

namespace ur_admittance_controller {

Eigen::Vector3d WrenchCalibrationNode::estimateGravitationalForceInBaseFrame(const std::vector<CalibrationSample>& samples) {
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
    int min_idx = 0;
    for (int i = 1; i < singular_values.size(); ++i) {
        if (singular_values[i] < singular_values[min_idx]) {
            min_idx = i;
        }
    }

    const Eigen::VectorXd y_opt = svd.matrixU().col(min_idx);
    const Eigen::VectorXd x6 = -(A6_inv * A6.transpose() * A9 * I9) * y_opt;

    const Eigen::Vector3d gravity = x6(2) < 0 ? x6.head<3>() : Eigen::Vector3d(-x6.head<3>());

    return gravity;
}

std::pair<Eigen::Matrix3d, Eigen::Vector3d> WrenchCalibrationNode::estimateSensorRotationAndForceBias(const std::vector<CalibrationSample>& samples, const Eigen::Vector3d& gravity_in_base) {
    const size_t n = samples.size();

    Eigen::Vector3d force_readings_avg = Eigen::Vector3d::Zero();
    Eigen::Matrix3d rotation_TB_avg = Eigen::Matrix3d::Zero();

    for (const CalibrationSample& sample : samples) {
        force_readings_avg += sample.wrench_raw.head<3>();
        rotation_TB_avg += sample.transform_TB.rotation();
    }
    force_readings_avg /= static_cast<double>(n);
    rotation_TB_avg /= static_cast<double>(n);

    Eigen::Matrix3d D = Eigen::Matrix3d::Zero();
    for (const CalibrationSample& sample : samples) {
        const Eigen::Vector3d force_centered = sample.wrench_raw.head<3>() - force_readings_avg;
        const Eigen::Matrix3d rotation_centered = sample.transform_TB.rotation() - rotation_TB_avg;
        D += rotation_centered * gravity_in_base * force_centered.transpose();
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(D, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
    const double det = (svd.matrixU() * svd.matrixV().transpose()).determinant();
    if (det < 0) {
        correction(2, 2) = -1;
    }

    const Eigen::Matrix3d R_SE = svd.matrixU() * correction * svd.matrixV().transpose();

    const Eigen::Vector3d force_bias =
        force_readings_avg - (R_SE * rotation_TB_avg * gravity_in_base);

    return std::make_pair(R_SE, force_bias);
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> WrenchCalibrationNode::estimateCOMAndTorqueBias(const std::vector<CalibrationSample>& samples, const Eigen::Vector3d& force_bias) {
    const size_t n = samples.size();
    const size_t rows = 3 * n;

    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(rows, 6);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(rows);

    for (size_t i = 0; i < n; ++i) {
        const CalibrationSample& sample = samples[i];
        const size_t row_offset = 3 * i;

        const Eigen::Vector3d force = sample.wrench_raw.head<3>();
        const Eigen::Vector3d torque = sample.wrench_raw.tail<3>();

        const Eigen::Vector3d force_compensated = -(force - force_bias);

        Eigen::Matrix3d skew_matrix;
        skew_matrix <<           0, -force_compensated(2),  force_compensated(1),
                       force_compensated(2),           0, -force_compensated(0),
                      -force_compensated(1),  force_compensated(0),           0;
        C.block<3, 3>(row_offset, 0) = skew_matrix;
        C.block<3, 3>(row_offset, 3) = Eigen::Matrix3d::Identity();

        b.segment<3>(row_offset) = torque;
    }

    const Eigen::VectorXd solution = (C.transpose() * C).inverse() * (C.transpose() * b);

    return std::make_pair(solution.head<3>(), solution.tail<3>());
}

}