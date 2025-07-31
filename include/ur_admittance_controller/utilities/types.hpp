#pragma once

#include <vector>
#include <string>
#include <chrono>
#include <array>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ur_admittance_controller {

// =============================================================================
// Core Type Definitions
// =============================================================================

// Degrees of freedom for 6D Cartesian space (3 translation + 3 rotation)
static constexpr size_t DOF = 6;

// Basic Eigen types
using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;
using Transform = Eigen::Isometry3d;

// 6D types for admittance control
using Matrix6d = Eigen::Matrix<double, 6, 6>;  // Mass, damping, stiffness matrices
using Vector6d = Eigen::Matrix<double, 6, 1>;  // Velocities, poses
using Wrench6d = Eigen::Matrix<double, 6, 1>;  // 6D [fx,fy,fz,tx,ty,tz]
using Wrench = Wrench6d;  // Backward compatibility alias

// 3D force/torque components
using Force3d = Eigen::Vector3d;
using Torque3d = Eigen::Vector3d;

// Joint space types
using JointVector = std::vector<double>;  // For joint positions/velocities
using JointAngles = std::vector<double>;  // Alias for clarity
using JointNames = std::vector<std::string>;
using PoseSequence = std::vector<JointAngles>;

// Time types
using Seconds = std::chrono::seconds;
using Milliseconds = std::chrono::milliseconds;

// =============================================================================
// Precision Management (Drake-inspired approach)
// =============================================================================

namespace precision {
    
// Relative accuracy (alpha in Drake's formula)
// Drake uses 1e-10 to 1e-15, we use 1e-10 for robotics applications
static constexpr double kDefaultRelativeTolerance = 1e-10;

// Characteristic scales for different physical quantities
// These represent typical magnitudes for each quantity type
struct Scales {
    static constexpr double kJointAngle = 1.0;      // radians (typical range: -π to π)
    static constexpr double kForce = 1.0;           // Newtons (typical F/T sensor range)
    static constexpr double kTorque = 0.1;          // Newton-meters (typical joint torques)
    static constexpr double kPosition = 0.001;      // meters (mm-scale precision)
    static constexpr double kVelocity = 0.01;       // m/s (typical joint velocities)
};

// Minimum absolute thresholds to prevent over-aggressive zeroing
// These are safety bounds below which we always consider values as zero
struct MinThresholds {
    static constexpr double kJointAngle = 1e-12;    // ~0.0000000001 degrees
    static constexpr double kForce = 1e-6;          // 1 micro-Newton
    static constexpr double kTorque = 1e-7;         // 0.1 micro-Newton-meter
    static constexpr double kPosition = 1e-9;       // 1 nanometer
    static constexpr double kVelocity = 1e-8;       // 10 nm/s
};

// Core comparison function following Drake's pattern
// Returns true if value should be considered zero
template<typename T>
inline bool isNearlyZero(T value, T scale, T min_threshold = 0) {
    // Compute adaptive tolerance: relative_accuracy * max(scale, |value|)
    T tolerance = kDefaultRelativeTolerance * std::max(scale, std::abs(value));
    
    // Ensure we don't go below the minimum threshold
    tolerance = std::max(tolerance, min_threshold);
    
    // Check if absolute value is within tolerance
    return std::abs(value) < tolerance;
}

// Sanitization function - returns 0 if value is nearly zero, otherwise returns value
template<typename T>
inline T sanitize(T value, T scale, T min_threshold = 0) {
    return isNearlyZero(value, scale, min_threshold) ? T(0) : value;
}

} // namespace precision

// =============================================================================
// Type-Safe Sanitization Functions
// =============================================================================

// Joint angle sanitization - handles individual angles
inline double sanitizeJointAngle(double angle) {
    // First apply Drake-style sanitization
    double sanitized = precision::sanitize(angle, 
                                         precision::Scales::kJointAngle,
                                         precision::MinThresholds::kJointAngle);
    
    // Round to 4 decimal places to avoid excessive precision
    // This gives us 0.1 millirad precision (0.0057 degrees) which is sufficient
    // for any practical robotics application
    return std::round(sanitized * 10000.0) / 10000.0;
}

// Joint vector sanitization - handles entire joint state
inline void sanitizeJointVector(std::vector<double>& joints) {
    for (auto& joint : joints) {
        joint = sanitizeJointAngle(joint);
    }
}

// Force vector sanitization (3D)
inline Vector3d sanitizeForce(const Vector3d& force) {
    Vector3d result;
    for (int i = 0; i < 3; ++i) {
        result[i] = precision::sanitize(force[i],
                                       precision::Scales::kForce,
                                       precision::MinThresholds::kForce);
    }
    return result;
}

// Torque vector sanitization (3D)
inline Vector3d sanitizeTorque(const Vector3d& torque) {
    Vector3d result;
    for (int i = 0; i < 3; ++i) {
        result[i] = precision::sanitize(torque[i],
                                       precision::Scales::kTorque,
                                       precision::MinThresholds::kTorque);
    }
    return result;
}

// Wrench sanitization (6D force/torque combined)
inline Wrench6d sanitizeWrench(const Wrench6d& wrench) {
    Wrench6d result;
    // First 3 components are forces
    result.head<3>() = sanitizeForce(wrench.head<3>());
    // Last 3 components are torques
    result.tail<3>() = sanitizeTorque(wrench.tail<3>());
    return result;
}

// Position sanitization (3D Cartesian)
inline Vector3d sanitizePosition(const Vector3d& position) {
    Vector3d result;
    for (int i = 0; i < 3; ++i) {
        result[i] = precision::sanitize(position[i],
                                       precision::Scales::kPosition,
                                       precision::MinThresholds::kPosition);
    }
    return result;
}

// Velocity sanitization (can be joint or Cartesian)
inline double sanitizeVelocity(double velocity) {
    return precision::sanitize(velocity,
                              precision::Scales::kVelocity,
                              precision::MinThresholds::kVelocity);
}

// =============================================================================
// Calibration Types
// =============================================================================

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
    Transform X_TB;     // Matches lookupTransform(tool, base) order
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

// =============================================================================
// Frame Definitions
// =============================================================================

namespace frames {
    static constexpr const char* ROBOT_BASE_FRAME = "base_link";
    static constexpr const char* ROBOT_TOOL_FRAME = "tool0";  
    static constexpr const char* SENSOR_FRAME = "netft_link1";
    static constexpr const char* PROBE_FRAME = "p42v_link1";
}

} // namespace ur_admittance_controller