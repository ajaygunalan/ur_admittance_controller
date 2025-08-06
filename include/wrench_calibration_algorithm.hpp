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

#include <tl/expected.hpp>

#include "wrench_calibration_node.hpp"

namespace ur_admittance_controller {

Result<Force3d> estimateGravitationalForceInBaseFrame(
    const std::vector<CalibrationSample>& samples);

Result<std::pair<Matrix3d, Force3d>> estimateSensorRotationAndForceBias(
    const std::vector<CalibrationSample>& samples,
    const Force3d& f_gravity_B);

Result<std::pair<Vector3d, Torque3d>> estimateCOMAndTorqueBias(
    const std::vector<CalibrationSample>& samples,
    const Force3d& f_bias_S);

Result<Matrix3d> estimateRobotInstallationBias(const Force3d& f_gravity_B);

Result<double> extractToolMass(const Force3d& f_gravity_B);

}