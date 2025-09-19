#pragma once
// Critical insights: (i) Wrench input is already in PROBE (P). (ii) All 6D vectors use [linear; angular] ordering to match KDL::Twist. (iii) Mapping to WORLD uses adjoints built from the desired pose.

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "admittance_node.hpp"  // shared types

namespace ur_admittance_controller {

// ------- Algebra -------
inline Matrix3d Skew(const Vector3d& p) {
  Matrix3d S;
  S <<     0.0, -p.z(),  p.y(),
        p.z(),    0.0, -p.x(),
       -p.y(),  p.x(),    0.0;
  return S;
}

// Twist adjoint for [linear; angular] ordering: [v_W; w_W] = [[R, p^R],[0,R]] [v_B; w_B]
Matrix6d AdTwist(const Matrix3d& R, const Vector3d& p);

// Pose-adjoint (for small pose offsets): diag(R, R); twist-adjoint above.
struct Adjoints {
  Matrix6d Ad_pose;   // diag(R_des, R_des)
  Matrix6d Ad_twist;  // [[R_des, p^R_des],[0, R_des]]
};
Adjoints BuildAdjointsFromDesired(const Matrix3d& R_des, const Vector3d& p_des);

// ------- Admittance ODE (P-frame) -------
Vector6d ComputeAdmittanceAcceleration(
    const Vector6d& F_ext_P,
    const Vector6d& xdot_P,
    const Vector6d& x_P,
    const Vector6d& M_inv_diag,
    const Vector6d& D_diag,
    const Vector6d& K_diag);

Vector6d IntegrateVelocity(const Vector6d& v, const Vector6d& a, double dt);

// ------- Kinematics -------
namespace kinematics {
struct KinematicsComponents {
    KDL::Tree tree;
    KDL::Chain robot_chain;
    KDL::Frame probe_offset;  // wrist_3_link -> probe (TCP)
    size_t num_joints;
};

Result<KinematicsComponents> InitializeFromUrdf(
    const urdf::Model& urdf_model,
    const std::string& base_link,
    const std::string& probe_link);
} // namespace kinematics

Result<Eigen::Isometry3d> ComputeForwardKinematics(
    const std::vector<double>& q_joints,
    KDL::ChainFkSolverPos_recursive* fk_solver,
    const KDL::Frame& probe_offset);

Result<std::vector<double>> ComputeInverseKinematicsVelocity(
    const Vector6d& V_probe_W,
    const Eigen::Isometry3d& X_probe_W,
    const KDL::Frame& X_wrist_W,
    const std::vector<double>& q_current,
    KDL::ChainIkSolverVel_wdls* ik_solver);

// ------- Pose & limits -------
Vector6d ComputePoseError(
    const Eigen::Isometry3d& X_cmd_W,
    const Eigen::Isometry3d& X_meas_W);

std::pair<double, double> GetPoseErrorNorms(const Vector6d& pose_error);

Vector6d ApplyWorkspaceLimits(
    const Vector6d& velocity_W,
    const Eigen::Vector3d& current_position_W,
    const Vector6d& workspace_limits);

Vector6d LimitVelocityMagnitude(
    const Vector6d& velocity,
    double max_linear_vel,
    double max_angular_vel);

} // namespace ur_admittance_controller
