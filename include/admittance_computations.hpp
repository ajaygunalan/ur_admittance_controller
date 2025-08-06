#pragma once

#include <fmt/core.h>
#include <fmt/format.h>
#include <kdl_parser/kdl_parser.hpp>

#include "admittance_node.hpp"

namespace ur_admittance_controller {

namespace kinematics {
    Result<KinematicsComponents> InitializeFromUrdf(
        const urdf::Model& urdf_model,
        const std::string& base_link,
        const std::string& tip_link);
}

inline Vector6d ComputeAdmittanceAcceleration(
    const Vector6d& external_wrench,
    const Vector6d& velocity_commanded,
    const Vector6d& pose_error,
    const Vector6d& mass_inverse,
    const Vector6d& damping,
    const Vector6d& stiffness);

inline Vector6d LimitAcceleration(
    const Vector6d& acceleration,
    double max_linear_acc,
    double max_angular_acc);

inline Vector6d IntegrateVelocity(
    const Vector6d& current_velocity,
    const Vector6d& acceleration,
    double dt);

inline Result<Eigen::Isometry3d> ComputeForwardKinematics(
    const std::vector<double>& q_joints,
    KDL::ChainFkSolverPos_recursive* fk_solver,
    const KDL::Frame& tool_offset);

inline Result<std::vector<double>> ComputeInverseKinematicsVelocity(
    const Vector6d& V_tool,
    const Eigen::Isometry3d& X_tool,
    const KDL::Frame& X_wrist,
    const std::vector<double>& q_current,
    KDL::ChainIkSolverVel_wdls* ik_solver);

inline Vector6d ComputePoseError(
    const Eigen::Isometry3d& X_current,
    const Eigen::Isometry3d& X_desired);

inline std::pair<double, double> GetPoseErrorNorms(const Vector6d& pose_error);

inline Vector6d ApplyWorkspaceLimits(
    const Vector6d& velocity,
    const Eigen::Vector3d& current_position,
    const Vector6d& workspace_limits);

inline Vector6d LimitVelocityMagnitude(
    const Vector6d& velocity,
    double max_linear_vel,
    double max_angular_vel);

}