#pragma once

#include <string>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
#include <urdf/model.h>

#include <utilities/types.hpp>
#include <utilities/error.hpp>

namespace ur_admittance_controller {
namespace kinematics {

// Container for KDL kinematics components (following ROS2 pattern - no solvers)
struct KinematicsComponents {
    KDL::Tree tree;
    KDL::Chain robot_chain;         // Base → wrist_3_link (6 DOF)
    KDL::Frame tool_offset;         // Wrist_3 → tip (fixed transform)
    size_t num_joints;
};

// Initialize KDL kinematics from URDF model
Result<KinematicsComponents> InitializeFromUrdf(
    const urdf::Model& urdf_model,
    const std::string& base_link,
    const std::string& tip_link,
    double ik_precision = 1e-5,
    int ik_max_iterations = 150,
    double ik_damping = 0.15);

// Initialize KDL kinematics from URDF string
Result<KinematicsComponents> InitializeFromUrdfString(
    const std::string& urdf_string,
    const std::string& base_link,
    const std::string& tip_link);

// Convert KDL::Frame to Eigen::Isometry3d
Transform KdlToEigen(const KDL::Frame& frame);

} // namespace kinematics
} // namespace ur_admittance_controller