#pragma once

#include <array>
#include <chrono>
#include <filesystem>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
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
};

struct CalibrationPose {
    std::array<double, 6> joints;
};

struct CalibrationSample {
    Eigen::Matrix<double, 6, 1> wrench_raw;
    Eigen::Isometry3d transform_TB;
    size_t pose_index;
};

struct CalibrationResult {
    Eigen::Matrix3d sensor_rotation;
    Eigen::Vector3d gravity_force;
    Eigen::Vector3d force_bias;
    Eigen::Vector3d torque_bias;
    Eigen::Vector3d center_of_mass;
    double tool_mass;
};

// ============================================================================
// Constants
// ============================================================================

namespace constants {
    constexpr int NUM_POSES = 32;
    constexpr int SAMPLES_PER_POSE = 10;
    constexpr double GRAVITY = 9.81;
    
    constexpr auto TRAJECTORY_DURATION = std::chrono::seconds(3);
    constexpr auto SAMPLE_DELAY = std::chrono::milliseconds(100);
    constexpr auto TIMEOUT = std::chrono::seconds(5);
    
    inline const std::array<std::string, 6> JOINT_NAMES{{
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    }};
}

// ============================================================================
// Pure Functions (No Side Effects)
// ============================================================================

std::vector<CalibrationPose> generate_calibration_poses(const JointConfiguration& current);

CalibrationResult compute_calibration(const std::vector<CalibrationSample>& samples);

YAML::Node build_yaml_config(const CalibrationResult& result);

std::filesystem::path get_calibration_config_path();

// ============================================================================
// IO Operations (Side Effects)
// ============================================================================

JointConfiguration read_current_joints(rclcpp::Node::SharedPtr node);

std::vector<CalibrationSample> collect_calibration_samples(
    rclcpp::Node::SharedPtr node,
    const std::vector<CalibrationPose>& poses);

void save_yaml_file(const std::filesystem::path& path, const YAML::Node& config);

} // namespace ur_admittance_controller