#pragma once

#include <array>
#include <chrono>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <yaml-cpp/yaml.h>

namespace ur_admittance_controller {

// ============================================================================
// Pure Data Types (Immutable)
// ============================================================================

struct JointConfiguration {
  std::array<double, 6> values;
  
  constexpr double operator[](size_t i) const { return values[i]; }
  constexpr auto begin() const { return values.begin(); }
  constexpr auto end() const { return values.end(); }
  constexpr size_t size() const { return values.size(); }
};

struct CartesianPose {
  std::array<double, 3> position;
  std::array<double, 4> orientation;  // quaternion [w,x,y,z]
};

struct KinematicModel {
  KDL::Chain robot_chain;   // base_link -> wrist_3_link
  KDL::Frame probe_offset;  // wrist_3_link -> probe_link (TCP)
  size_t num_joints;
};

struct TrajectoryPoint {
  JointConfiguration joints;
  rclcpp::Duration time_from_start;
};

// ============================================================================
// Constants
// ============================================================================

namespace constants {
  constexpr JointConfiguration EQUILIBRIUM_JOINTS{{
    0.0,                    // shoulder_pan
    -1.5707963267948966,    // shoulder_lift (-π/2)
    1.5707963267948966,     // elbow (π/2)
    -1.5707963267948966,    // wrist_1 (-π/2)
    -1.5707963267948966,    // wrist_2 (-π/2)
    0.0                     // wrist_3
  }};
  
  inline const std::array<std::string, 6> JOINT_NAMES{{
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
  }};
  
  constexpr std::chrono::seconds TRAJECTORY_DURATION{12};
  constexpr std::chrono::seconds SERVICE_TIMEOUT{5};
}

// ============================================================================
// Pure Functions (No Side Effects) - Declarations
// ============================================================================

KinematicModel build_kinematic_model(const std::string& urdf_string);

CartesianPose compute_forward_kinematics(const KinematicModel& model,
                                         const JointConfiguration& joints);

std::vector<TrajectoryPoint> create_trajectory(const JointConfiguration& start,
                                               const JointConfiguration& target,
                                               rclcpp::Duration duration);

YAML::Node build_equilibrium_config(const CartesianPose& pose);

std::optional<JointConfiguration> parse_joint_state(const sensor_msgs::msg::JointState& msg);

std::filesystem::path get_config_path();

// ============================================================================
// IO Operations (Side Effects) - Declarations
// ============================================================================

std::string fetch_robot_description(rclcpp::Node::SharedPtr node);

JointConfiguration read_current_joints(rclcpp::Node::SharedPtr node);

void execute_trajectory(rclcpp::Node::SharedPtr node,
                        const std::vector<TrajectoryPoint>& trajectory);

void save_yaml_file(const std::filesystem::path& path, const YAML::Node& config);

} // namespace ur_admittance_controller
