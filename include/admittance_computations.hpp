#pragma once

#include <fmt/core.h>
#include <fmt/format.h>
#include <kdl_parser/kdl_parser.hpp>

// Pull in shared types (Vector6d, Result, KinematicsComponents, etc.)
#include "admittance_node_new.hpp"

namespace ur_admittance_controller {

namespace kinematics {
    // Parse URDF and build KDL structures; also computes wrist->tool offset.
    Result<KinematicsComponents> InitializeFromUrdf(
        const urdf::Model& urdf_model,
        const std::string& base_link,
        const std::string& tip_link);
}

// Core algebra helpers used by the node's control cycle.
Vector6d ComputeAdmittanceAcceleration(
    const Vector6d& external_wrench,
    const Vector6d& velocity_state,
    const Vector6d& position_state,
    const Vector6d& mass_inverse,
    const Vector6d& damping,
    const Vector6d& stiffness);

Vector6d LimitAcceleration(
    const Vector6d& acceleration,
    double max_linear_acc,
    double max_angular_acc);

Vector6d IntegrateVelocity(
    const Vector6d& current_velocity,
    const Vector6d& acceleration,
    double dt);

Result<Eigen::Isometry3d> ComputeForwardKinematics(
    const std::vector<double>& q_joints,
    KDL::ChainFkSolverPos_recursive* fk_solver,
    const KDL::Frame& tool_offset);

Result<std::vector<double>> ComputeInverseKinematicsVelocity(
    const Vector6d& V_tool,
    const Eigen::Isometry3d& X_tool,
    const KDL::Frame& X_wrist,
    const std::vector<double>& q_current,
    KDL::ChainIkSolverVel_wdls* ik_solver);

Vector6d ComputePoseError(
    const Eigen::Isometry3d& X_current,
    const Eigen::Isometry3d& X_desired);

std::pair<double, double> GetPoseErrorNorms(const Vector6d& pose_error);

Vector6d ApplyWorkspaceLimits(
    const Vector6d& velocity,
    const Eigen::Vector3d& current_position,
    const Vector6d& workspace_limits);

Vector6d LimitVelocityMagnitude(
    const Vector6d& velocity,
    double max_linear_vel,
    double max_angular_vel);

}  // namespace ur_admittance_controller
