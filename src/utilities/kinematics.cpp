#include <utilities/kinematics.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/logging.hpp>
#include <fmt/core.h>

namespace ur_admittance_controller {
namespace kinematics {

Result<KinematicsComponents> InitializeFromUrdf(
    const urdf::Model& urdf_model,
    const std::string& base_link,
    const std::string& tip_link,
    double /*ik_precision*/,
    int /*ik_max_iterations*/,
    double /*ik_damping*/) {

    KinematicsComponents components;

    // Convert URDF to KDL tree
    if (!kdl_parser::treeFromUrdfModel(urdf_model, components.tree)) {
        return tl::unexpected(MakeError(ErrorCode::kKinematicsInitFailed,
                                       "Failed to convert URDF to KDL tree"));
    }

    // Extract chain from base to wrist (tool0 = wrist_3_link for UR robots)
    if (!components.tree.getChain(base_link, "wrist_3_link", components.robot_chain)) {
        return tl::unexpected(MakeError(ErrorCode::kKinematicsInitFailed,
            fmt::format("Failed to extract chain from {} to wrist_3_link", base_link)));
    }

    // Get transform from wrist to actual tip
    if (!components.tree.getChain("wrist_3_link", tip_link, components.robot_chain)) {
        // If direct chain fails, just use identity (wrist_3 = tool0)
        components.tool_offset = KDL::Frame::Identity();
    } else {
        // For UR robots with custom tools, this captures tool transform
        KDL::Chain tool_chain;
        components.tree.getChain("wrist_3_link", tip_link, tool_chain);
        if (tool_chain.getNrOfSegments() > 0) {
            components.tool_offset = tool_chain.getSegment(0).getFrameToTip();
        } else {
            components.tool_offset = KDL::Frame::Identity();
        }
    }

    components.num_joints = components.robot_chain.getNrOfJoints();

    RCLCPP_DEBUG(rclcpp::get_logger("kinematics"),
                 "Initialized KDL: %zu joints, %d segments",
                 components.num_joints, components.robot_chain.getNrOfSegments());

    return components;
}

Result<KinematicsComponents> InitializeFromUrdfString(
    const std::string& urdf_string,
    const std::string& base_link,
    const std::string& tip_link) {

    urdf::Model urdf_model;
    if (!urdf_model.initString(urdf_string)) {
        return tl::unexpected(MakeError(ErrorCode::kKinematicsInitFailed,
                                       "Failed to parse URDF string"));
    }

    return InitializeFromUrdf(urdf_model, base_link, tip_link);
}

Transform KdlToEigen(const KDL::Frame& frame) {
    Transform result;
    result.translation() << frame.p.x(), frame.p.y(), frame.p.z();

    double x, y, z, w;
    frame.M.GetQuaternion(x, y, z, w);
    result.linear() = Eigen::Quaterniond(w, x, y, z).toRotationMatrix();

    return result;
}

} // namespace kinematics
} // namespace ur_admittance_controller