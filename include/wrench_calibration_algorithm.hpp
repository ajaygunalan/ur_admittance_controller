#pragma once

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include "wrench_calibration_node.hpp"

namespace ur_admittance_controller {

Force3d estimateGravitationalForceInBaseFrame(
    const std::vector<CalibrationSample>& samples);

std::pair<Matrix3d, Force3d> estimateSensorRotationAndForceBias(
    const std::vector<CalibrationSample>& samples,
    const Force3d& f_gravity_B);

std::pair<Vector3d, Torque3d> estimateCOMAndTorqueBias(
    const std::vector<CalibrationSample>& samples,
    const Force3d& f_bias_S);

Matrix3d estimateRobotInstallationBias(const Force3d& f_gravity_B);

double extractToolMass(const Force3d& f_gravity_B);

}