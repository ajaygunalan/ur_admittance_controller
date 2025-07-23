#pragma once

#include "types.hpp"
#include "error.hpp"
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <memory>
#include <string>

namespace ur_admittance_controller::kinematics {

/**
 * Container for KDL kinematics components
 */
struct KinematicsComponents {
    KDL::Tree tree;
    KDL::Chain robot_chain;         // Base → wrist_3_link (6 DOF)
    KDL::Frame tool_offset;         // Wrist_3 → tip (fixed transform)
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
    std::unique_ptr<KDL::ChainIkSolverVel_wdls> ik_vel_solver;
    size_t num_joints;
};

/**
 * Initialize KDL kinematics from URDF model
 * @param urdf_model URDF robot model
 * @param base_link Base link name (e.g., "base_link")
 * @param tip_link Tip link name (e.g., "tool0" or custom payload)
 * @param ik_precision Precision for IK solver (default: 1e-5)
 * @param ik_max_iterations Max iterations for IK (default: 150)
 * @param ik_damping Damping factor for IK (default: 0.15)
 * @return Result containing KinematicsComponents or error
 */
inline Result<KinematicsComponents> initializeFromUrdf(
    const urdf::Model& urdf_model,
    const std::string& base_link,
    const std::string& tip_link,
    double ik_precision = 1e-5,
    int ik_max_iterations = 150,
    double ik_damping = 0.15) {
    
    KinematicsComponents components;
    
    // Parse URDF to KDL tree
    if (!kdl_parser::treeFromUrdfModel(urdf_model, components.tree)) {
        return tl::unexpected(make_error(
            ErrorCode::kKinematicsInitFailed,
            "Failed to parse URDF to KDL tree"));
    }
    
    // Extract robot chain (base → wrist_3_link)
    if (!components.tree.getChain(base_link, "wrist_3_link", components.robot_chain)) {
        return tl::unexpected(make_error(
            ErrorCode::kKinematicsInitFailed,
            "Cannot extract chain from " + base_link + " to wrist_3_link"));
    }
    
    // Extract tool transform (wrist_3 → tip)
    KDL::Chain tool_chain;
    if (!components.tree.getChain("wrist_3_link", tip_link, tool_chain)) {
        return tl::unexpected(make_error(
            ErrorCode::kKinematicsInitFailed,
            "Cannot extract tool chain from wrist_3_link to " + tip_link));
    }
    
    // Compute fixed tool offset
    KDL::ChainFkSolverPos_recursive tool_fk(tool_chain);
    KDL::JntArray zero_joints(tool_chain.getNrOfJoints());
    tool_fk.JntToCart(zero_joints, components.tool_offset);
    
    // Create solvers
    components.fk_solver = std::make_unique<KDL::ChainFkSolverPos_recursive>(
        components.robot_chain);
    
    components.ik_vel_solver = std::make_unique<KDL::ChainIkSolverVel_wdls>(
        components.robot_chain, ik_precision, ik_max_iterations);
    components.ik_vel_solver->setLambda(ik_damping);
    
    components.num_joints = components.robot_chain.getNrOfJoints();
    
    return components;
}

/**
 * Initialize KDL kinematics from URDF string
 * @param urdf_string URDF XML string
 * @param base_link Base link name
 * @param tip_link Tip link name
 * @return Result containing KinematicsComponents or error
 */
inline Result<KinematicsComponents> initializeFromUrdfString(
    const std::string& urdf_string,
    const std::string& base_link,
    const std::string& tip_link) {
    
    urdf::Model urdf_model;
    if (!urdf_model.initString(urdf_string)) {
        return tl::unexpected(make_error(
            ErrorCode::kKinematicsInitFailed,
            "Failed to parse URDF string"));
    }
    
    return initializeFromUrdf(urdf_model, base_link, tip_link);
}

/**
 * Convert KDL::Frame to Eigen::Isometry3d
 * @param frame KDL frame
 * @return Eigen transform
 */
inline Transform kdlToEigen(const KDL::Frame& frame) {
    Transform result = Transform::Identity();
    
    // Rotation
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result.matrix()(i, j) = frame.M(i, j);
        }
    }
    
    // Translation
    result.translation() << frame.p.x(), frame.p.y(), frame.p.z();
    
    return result;
}

/**
 * Convert Eigen::Isometry3d to KDL::Frame
 * @param transform Eigen transform
 * @return KDL frame
 */
inline KDL::Frame eigenToKdl(const Transform& transform) {
    KDL::Frame frame;
    
    // Rotation
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            frame.M(i, j) = transform.matrix()(i, j);
        }
    }
    
    // Translation
    frame.p.x(transform.translation().x());
    frame.p.y(transform.translation().y());
    frame.p.z(transform.translation().z());
    
    return frame;
}

} // namespace ur_admittance_controller::kinematics