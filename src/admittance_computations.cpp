#include "admittance_computations.hpp"
#include <cmath>
#include <fmt/core.h>

namespace ur_admittance_controller {

// ------- Adjoint builders -------
Matrix6d AdTwist(const Matrix3d& R, const Vector3d& p) {
  Matrix6d Ad = Matrix6d::Zero();
  Ad.block<3,3>(0,0) = R;
  Ad.block<3,3>(0,3) = Skew(p) * R;   // v += w × p (expressed via p^ R)
  Ad.block<3,3>(3,3) = R;
  return Ad;
}

Adjoints BuildAdjointsFromDesired(const Matrix3d& R_des, const Vector3d& p_des) {
  Adjoints A;
  A.Ad_pose.setZero();
  A.Ad_pose.block<3,3>(0,0) = R_des;
  A.Ad_pose.block<3,3>(3,3) = R_des;
  A.Ad_twist = AdTwist(R_des, p_des);
  return A;
}

// ------- ODE helpers -------
Vector6d ComputeAdmittanceAcceleration(
    const Vector6d& F_ext_P,
    const Vector6d& xdot_P,
    const Vector6d& x_P,
    const Vector6d& M_inv_diag,
    const Vector6d& D_diag,
    const Vector6d& K_diag) {
  return M_inv_diag.array() * (F_ext_P.array() - D_diag.array()*xdot_P.array() - K_diag.array()*x_P.array());
}

Vector6d IntegrateVelocity(const Vector6d& v, const Vector6d& a, double dt) {
  return v + a * dt;
}

// ------- Kinematics -------
namespace kinematics {
Result<KinematicsComponents> InitializeFromUrdf(const urdf::Model& urdf_model,
                                                const std::string& base_link,
                                                const std::string& tip_link) {
  KinematicsComponents components;

  if (!kdl_parser::treeFromUrdfModel(urdf_model, components.tree)) {
    return tl::unexpected(MakeError(ErrorCode::kKinematicsInitFailed,
                                    "URDF→KDL tree conversion failed"));
  }

  if (!components.tree.getChain(base_link, "wrist_3_link", components.robot_chain)) {
    return tl::unexpected(MakeError(ErrorCode::kKinematicsInitFailed,
                                    "Chain base_link→wrist_3_link not found"));
  }

  KDL::Chain tool_chain;
  if (!components.tree.getChain("wrist_3_link", tip_link, tool_chain)) {
    components.tool_offset = KDL::Frame::Identity();
  } else {
    components.tool_offset = KDL::Frame::Identity();
    for (unsigned int i = 0; i < tool_chain.getNrOfSegments(); ++i) {
      components.tool_offset = components.tool_offset * tool_chain.getSegment(i).getFrameToTip();
    }
  }

  components.num_joints = components.robot_chain.getNrOfJoints();
  return components;
}
} // namespace kinematics

Result<Eigen::Isometry3d> ComputeForwardKinematics(const std::vector<double>& q_joints,
                                                   KDL::ChainFkSolverPos_recursive* fk_solver,
                                                   const KDL::Frame& tool_offset) {
  KDL::JntArray q_kdl(q_joints.size());
  for (size_t i = 0; i < q_joints.size(); ++i) {
    q_kdl(i) = std::atan2(std::sin(q_joints[i]), std::cos(q_joints[i])); // wrap to [-pi,pi]
  }

  KDL::Frame X_base_joint;
  int fk_result = fk_solver->JntToCart(q_kdl, X_base_joint);
  if (fk_result < 0) {
    return tl::unexpected(MakeError(ErrorCode::kKinematicsInitFailed, "KDL FK failed"));
  }

  KDL::Frame X_base_tool = X_base_joint * tool_offset;

  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(X_base_tool.p.x(),
                                         X_base_tool.p.y(),
                                         X_base_tool.p.z());
  double x, y, z, w;
  X_base_tool.M.GetQuaternion(x, y, z, w);
  result.linear() = Eigen::Quaterniond(w, x, y, z).toRotationMatrix();
  return result;
}

Result<std::vector<double>> ComputeInverseKinematicsVelocity(
    const Vector6d& V_tool_W,
    const Eigen::Isometry3d& X_tool_W,
    const KDL::Frame& X_wrist_W,
    const std::vector<double>& q_current,
    KDL::ChainIkSolverVel_wdls* ik_solver) {

  KDL::JntArray q_kdl(q_current.size());
  for (size_t i = 0; i < q_current.size(); ++i) q_kdl(i) = q_current[i];

  // TCP twist (WORLD) with [linear; angular] ordering
  KDL::Twist V_tool_kdl;
  V_tool_kdl.vel = KDL::Vector(V_tool_W(0), V_tool_W(1), V_tool_W(2));
  V_tool_kdl.rot = KDL::Vector(V_tool_W(3), V_tool_W(4), V_tool_W(5));

  // Shift to wrist point (WORLD): v_wrist = v_tool - omega × p
  KDL::Vector p_tool_wrist(
      X_tool_W.translation()(0) - X_wrist_W.p.x(),
      X_tool_W.translation()(1) - X_wrist_W.p.y(),
      X_tool_W.translation()(2) - X_wrist_W.p.z());

  KDL::Twist V_wrist_kdl;
  V_wrist_kdl.vel = V_tool_kdl.vel - V_tool_kdl.rot * p_tool_wrist;
  V_wrist_kdl.rot = V_tool_kdl.rot;

  KDL::JntArray v_kdl(q_current.size());
  if (ik_solver->CartToJnt(q_kdl, V_wrist_kdl, v_kdl) < 0) {
    return tl::unexpected(MakeError(ErrorCode::kIKSolverFailed, "KDL IKVel failed"));
  }

  std::vector<double> joint_velocities(q_current.size());
  for (size_t i = 0; i < q_current.size(); ++i) {
    if (std::isnan(v_kdl(i))) {
      return tl::unexpected(MakeError(ErrorCode::kIKSolverFailed, "IK returned NaN"));
    }
    joint_velocities[i] = v_kdl(i);
  }
  return joint_velocities;
}

// ------- Pose & limits -------
Vector6d ComputePoseError(const Eigen::Isometry3d& X_cmd_W,
                          const Eigen::Isometry3d& X_meas_W) {
  Vector6d e; // [pos; ori]
  e.head<3>() = X_cmd_W.translation() - X_meas_W.translation();

  Eigen::Quaterniond q_cmd(X_cmd_W.rotation());
  Eigen::Quaterniond q_meas(X_meas_W.rotation());
  if (q_cmd.dot(q_meas) < 0.0) q_meas.coeffs() *= -1.0;

  Eigen::Quaterniond q_err = (q_cmd * q_meas.inverse()).normalized();
  Eigen::AngleAxisd aa(q_err);
  e.tail<3>() = aa.axis() * aa.angle();
  return e;
}

std::pair<double,double> GetPoseErrorNorms(const Vector6d& e) {
  return {e.head<3>().norm(), e.tail<3>().norm()};
}

Vector6d ApplyWorkspaceLimits(const Vector6d& v,
                              const Eigen::Vector3d& pos,
                              const Vector6d& limits) {
  Vector6d out = v;
  for (size_t i = 0; i < 3; ++i) {
    const double min = limits[i*2], max = limits[i*2+1];
    if (pos[i] <= min) out[i] = std::max(0.0, out[i]);
    if (pos[i] >= max) out[i] = std::min(0.0, out[i]);
  }
  return out;
}

Vector6d LimitVelocityMagnitude(const Vector6d& v,
                                double max_lin, double max_ang) {
  Vector6d out = v;
  double ln = v.head<3>().norm();
  if (ln > max_lin && ln > 1e-12) out.head<3>() *= (max_lin / ln);
  double an = v.tail<3>().norm();
  if (an > max_ang && an > 1e-12) out.tail<3>() *= (max_ang / an);
  return out;
}

} // namespace ur_admittance_controller