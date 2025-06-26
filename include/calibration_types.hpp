#pragma once

#include <vector>
#include <string>
#include <chrono>
#include <array>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ur_admittance_controller {

// Common type aliases
using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;
using Wrench = Eigen::Matrix<double, 6, 1>;
using Transform = Eigen::Isometry3d;
using JointAngles = std::vector<double>;
using JointNames = std::vector<std::string>;
using PoseSequence = std::vector<JointAngles>;
using Seconds = std::chrono::seconds;
using Milliseconds = std::chrono::milliseconds;

// Calibration constants
namespace CalibrationConstants {
    static constexpr size_t NUM_POSES = 32;
    static constexpr size_t SAMPLES_PER_POSE = 10;
    static constexpr size_t TOTAL_SAMPLES = NUM_POSES * SAMPLES_PER_POSE;
    static constexpr double SAMPLE_RATE_HZ = 10.0;
    static constexpr auto SAMPLE_PERIOD = Milliseconds(100);
    static constexpr double MOTION_DURATION_S = 3.0;
    
    // UR robot joint names
    inline static const JointNames UR_JOINT_NAMES = {
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    };
}

// Calibration data structures
struct CalibrationSample {
    Wrench F_P_P_raw;  // Raw wrench at Payload in Payload frame
    Transform X_PB;     // Transform from Payload to Base
    size_t pose_index;  // Which pose (0-31) this sample belongs to
};

struct GravityCompensationParams {
    double tool_mass_kg{0.0};
    Vector3d p_CoM_P{Vector3d::Zero()};  // Position of CoM in Payload frame
    Vector3d F_gravity_B{Vector3d::Zero()};  // Gravity force in Base frame
    Vector3d F_bias_P{Vector3d::Zero()};     // Force bias in Payload frame
    Vector3d T_bias_P{Vector3d::Zero()};     // Torque bias in Payload frame
    Matrix3d R_PP{Matrix3d::Identity()};     // Rotation from Payload to Payload (identity for UR)
    std::array<double, 4> quaternion_sensor_to_endeffector{{0, 0, 0, 1}};  // Keep for compatibility
    size_t num_poses_collected{0};
    double residual_force_error{0.0};
    double residual_torque_error{0.0};
};

struct CalibrationResult {
    GravityCompensationParams params;
    bool success{false};
    std::string error_message;
    double force_fit_rmse{0.0};
    double torque_fit_rmse{0.0};
};

} // namespace ur_admittance_controller