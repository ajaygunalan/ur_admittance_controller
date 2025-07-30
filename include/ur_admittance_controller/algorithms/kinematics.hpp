#pragma once

#include <ur_admittance_controller/utilities/types.hpp>
#include <ur_admittance_controller/utilities/error.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <fmt/core.h>

namespace ur_admittance_controller {
namespace algorithms {

/**
 * @brief Compute forward kinematics from joint positions to Cartesian pose
 * 
 * @param q_joints Joint positions vector
 * @param fk_solver Forward kinematics solver
 * @param tool_offset Fixed transform from last joint to tool
 * @return Result containing tool pose or error
 */
inline Result<Eigen::Isometry3d> computeForwardKinematics(
    const std::vector<double>& q_joints,
    KDL::ChainFkSolverPos_recursive* fk_solver,
    const KDL::Frame& tool_offset) {
  
  // Convert to KDL format
  KDL::JntArray q_kdl(q_joints.size());
  for (size_t i = 0; i < q_joints.size(); ++i) {
    q_kdl(i) = q_joints[i];
  }
  
  // Compute FK to last joint
  KDL::Frame X_base_joint;
  int fk_result = fk_solver->JntToCart(q_kdl, X_base_joint);
  if (fk_result < 0) {
    // Debug info - fk_result: -1 = size mismatch, -2 = out of range
    return tl::unexpected(make_error(ErrorCode::kKinematicsInitFailed,
        fmt::format("FK failed: KDL error={}, input_joints={}, q_kdl_size={}",
                    fk_result, q_joints.size(), q_kdl.data.size())));
  }
  
  // Apply tool offset
  KDL::Frame X_base_tool = X_base_joint * tool_offset;
  
  // Convert to Eigen
  Eigen::Isometry3d result;
  result.translation() = Eigen::Vector3d(X_base_tool.p.x(), 
                                        X_base_tool.p.y(), 
                                        X_base_tool.p.z());
  
  double x, y, z, w;
  X_base_tool.M.GetQuaternion(x, y, z, w);
  result.linear() = Eigen::Quaterniond(w, x, y, z).toRotationMatrix();
  
  return result;
}

/**
 * @brief Compute inverse kinematics from Cartesian velocity to joint velocities
 * 
 * Handles the transformation from tool velocity to wrist velocity internally
 * 
 * @param V_tool Tool velocity twist (6D: linear, angular)
 * @param X_tool Current tool pose
 * @param X_wrist Current wrist pose (from cached FK)
 * @param q_current Current joint positions
 * @param ik_solver Inverse kinematics velocity solver
 * @return Result containing joint velocities or error
 */
inline Result<std::vector<double>> computeInverseKinematicsVelocity(
    const Vector6d& V_tool,
    const Eigen::Isometry3d& X_tool,
    const KDL::Frame& X_wrist,
    const std::vector<double>& q_current,
    KDL::ChainIkSolverVel_wdls* ik_solver) {
  
  // Convert current joints to KDL
  KDL::JntArray q_kdl(q_current.size());
  for (size_t i = 0; i < q_current.size(); ++i) {
    q_kdl(i) = q_current[i];
  }
  
  // Pack tool velocity
  KDL::Twist V_tool_kdl;
  V_tool_kdl.vel = KDL::Vector(V_tool(0), V_tool(1), V_tool(2));
  V_tool_kdl.rot = KDL::Vector(V_tool(3), V_tool(4), V_tool(5));
  
  // Compute position offset from wrist to tool
  KDL::Vector p_tool_wrist = KDL::Vector(
    X_tool.translation()(0) - X_wrist.p.x(),
    X_tool.translation()(1) - X_wrist.p.y(),
    X_tool.translation()(2) - X_wrist.p.z()
  );
  
  // Transform to wrist velocity
  KDL::Twist V_wrist_kdl;
  V_wrist_kdl.vel = V_tool_kdl.vel - V_tool_kdl.rot * p_tool_wrist;
  V_wrist_kdl.rot = V_tool_kdl.rot;
  
  // Solve IK
  KDL::JntArray v_kdl(q_current.size());
  if (ik_solver->CartToJnt(q_kdl, V_wrist_kdl, v_kdl) < 0) {
    return tl::unexpected(make_error(ErrorCode::kIKSolverFailed,
                                   "Inverse kinematics velocity computation failed"));
  }
  
  // Convert back to std::vector
  std::vector<double> joint_velocities(q_current.size());
  for (size_t i = 0; i < q_current.size(); ++i) {
    if (std::isnan(v_kdl(i))) {
      return tl::unexpected(make_error(ErrorCode::kIKSolverFailed,
                                     "IK returned NaN for joint " + std::to_string(i)));
    }
    joint_velocities[i] = v_kdl(i);
  }
  
  return joint_velocities;
}

} // namespace algorithms
} // namespace ur_admittance_controller