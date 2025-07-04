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
using Wrench6d = Eigen::Matrix<double, 6, 1>;  // 6D [fx,fy,fz,tx,ty,tz]
using Force3d = Eigen::Vector3d;                // 3D force only
using Torque3d = Eigen::Vector3d;               // 3D torque only
using Wrench = Wrench6d;  // Keep for backward compatibility
using Transform = Eigen::Isometry3d;
using JointAngles = std::vector<double>;
using JointNames = std::vector<std::string>;
using PoseSequence = std::vector<JointAngles>;
using Seconds = std::chrono::seconds;
using Milliseconds = std::chrono::milliseconds;

// Calibration constants
namespace CalibrationConstants {
    static constexpr int NUM_POSES = 32;
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
    Wrench F_S_S_raw;   // Force/Torque at Sensor Frame in Sensor Frame
    Transform X_EB;     // Transform from Base to End-effector
    size_t pose_index;  // Which pose (0-31) this sample belongs to
};

struct GravityCompensationParams {
    Vector3d p_SCoM_S{Vector3d::Zero()};  // Position of CoM from S, expressed in S
    Force3d f_gravity_B{Vector3d::Zero()};  // Gravity force in Base frame
    Force3d f_bias_S{Vector3d::Zero()};     // Force bias in Sensor frame
    Torque3d t_bias_S{Vector3d::Zero()};     // Torque bias in Sensor frame
    Matrix3d R_SE{Matrix3d::Identity()};     // Rotation from E to S
    std::array<double, 4> quaternion_sensor_to_endeffector{{0, 0, 0, 1}};  // Keep for compatibility
};

struct CalibrationResult {
    GravityCompensationParams params;
    bool success{false};
    std::string error_message;
    double force_fit_rmse{0.0};
    double torque_fit_rmse{0.0};
};

} // namespace ur_admittance_controller