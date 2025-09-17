
#pragma once
/**
 * @file admittance_computations.hpp
 * @brief Math/kinematics utilities used by AdmittanceNode (clean, pseudocode-accurate).
 *
 * Design goals (per pseudocode):
 *   - The admittance ODE runs in B_des axes.
 *   - Wrench is re-expressed into B_des using the proper force adjoint.
 *   - Offsets are mapped to world with the pose-adjoint diag(R_des, R_des).
 *   - Offset rates are mapped to world with the full twist adjoint (with p̂ R coupling).
 *   - Commanded pose and world twist are composed in world.
 *
 * Vectors use [linear; angular] ordering throughout (matches KDL::Twist).
 */

#include <fmt/core.h>
#include <fmt/format.h>
#include <kdl_parser/kdl_parser.hpp>

// Pull in shared types (Vector6d, Result, KinematicsComponents, etc.)
#include "admittance_node.hpp"

namespace ur_admittance_controller {

// ---------- Small algebra helpers ----------

/** @brief 6×6 twist adjoint for R, p (expresses a twist from B into W given X_WB=(R,p)).
 *  [v_W; ω_W] = Ad_twist(R,p) [v_B; ω_B] = [[R, 0], [p^ R, R]] [v_B; ω_B]
 */
Matrix6d AdTwist(const Matrix3d& R, const Vector3d& p);

/** @brief 6×6 force adjoint (dual) = Ad_twist(R,p)^T. */
Matrix6d AdForce(const Matrix3d& R, const Vector3d& p);

/** @brief Build both adjoints from the desired pose X_WB_des=(R_des,p_des). */
struct Adjoints {
  Matrix6d Ad_pose;   // diag(R_des, R_des)
  Matrix6d Ad_twist;  // [[R_des, 0], [p̂_des R_des, R_des]]
};
Adjoints BuildAdjointsFromDesired(const Matrix3d& R_des, const Vector3d& p_des);

// ---------- Admittance ODE bits ----------

/** @brief Element-wise:  M^{-1} (F - D v - K x)  in [linear; angular] order. */
Vector6d ComputeAdmittanceAcceleration(
    const Vector6d& external_wrench,
    const Vector6d& velocity_state,
    const Vector6d& position_state,
    const Vector6d& mass_inverse,
    const Vector6d& damping,
    const Vector6d& stiffness);

/** @brief Magnitude limiting per translational & rotational 3-norms. */
Vector6d LimitAcceleration(
    const Vector6d& acceleration,
    double max_linear_acc,
    double max_angular_acc);

/** @brief Euler step for velocity state. */
Vector6d IntegrateVelocity(
    const Vector6d& current_velocity,
    const Vector6d& acceleration,
    double dt);

// ---------- Kinematics ----------
namespace kinematics {
/** @brief Parse URDF and build KDL structures; also computes wrist->tool offset. */
Result<KinematicsComponents> InitializeFromUrdf(
    const urdf::Model& urdf_model,
    const std::string& base_link,
    const std::string& tip_link);
} // namespace kinematics

/** @brief FK: base->TCP including tool offset, as Eigen::Isometry3d (world). */
Result<Eigen::Isometry3d> ComputeForwardKinematics(
    const std::vector<double>& q_joints,
    KDL::ChainFkSolverPos_recursive* fk_solver,
    const KDL::Frame& tool_offset);

/** @brief Resolved-rate IK (WDLS): tool twist at TCP -> joint rates, with shift to wrist. */
Result<std::vector<double>> ComputeInverseKinematicsVelocity(
    const Vector6d& V_tool_W,
    const Eigen::Isometry3d& X_tool_W,
    const KDL::Frame& X_wrist_W,
    const std::vector<double>& q_current,
    KDL::ChainIkSolverVel_wdls* ik_solver);

// ---------- Pose utilities ----------

/** @brief Small pose error e = (X_cmd ⊖ X_meas) in [pos; ori] (world). */
Vector6d ComputePoseError(
    const Eigen::Isometry3d& X_cmd_W,
    const Eigen::Isometry3d& X_meas_W);

/** @brief Norms of pos/orient parts (for logging). */
std::pair<double, double> GetPoseErrorNorms(const Vector6d& pose_error);

/** @brief Apply planar box workspace limits using current position (world). */
Vector6d ApplyWorkspaceLimits(
    const Vector6d& velocity_W,
    const Eigen::Vector3d& current_position_W,
    const Vector6d& workspace_limits);

/** @brief Norm clamp on linear/rotational speeds. */
Vector6d LimitVelocityMagnitude(
    const Vector6d& velocity,
    double max_linear_vel,
    double max_angular_vel);

// ---------- Wrench re-expression ----------

/**
 * @brief Express an incoming wrench F_W (in WORLD/base) into B_des axes.
 * @details Uses the force adjoint of X_BdesW = X_WB_des^{-1}. This corresponds to
 *          Step 0 in the pseudocode where the wrench is transformed into the ODE's basis.
 *          If your wrench arrives in some other frame, re-express it to world in your
 *          wrench node. This keeps this controller decoupled from the current pose.
 */
inline Vector6d ExpressWrenchWorldToBdes(const Vector6d& F_W,
                                         const Matrix3d& R_des,
                                         const Vector3d& p_des) {
  // X_BdesW = (R_des^T, -R_des^T p_des)
  const Matrix3d R = R_des.transpose();
  const Vector3d p = -R * p_des;
  return AdForce(R, p) * F_W;
}

}  // namespace ur_admittance_controller
