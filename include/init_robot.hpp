#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tl/expected.hpp>
#include <urdf/model.h>
#include <yaml-cpp/yaml.h>

namespace ur_admittance_controller {

static constexpr size_t DOF = 6;

using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;
using Transform = Eigen::Isometry3d;

using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Wrench6d = Eigen::Matrix<double, 6, 1>;

using Force3d = Eigen::Vector3d;
using Torque3d = Eigen::Vector3d;

using JointVector = std::vector<double>;

enum class ErrorCode {
  kFileNotFound,
  kInvalidConfiguration,
  kKinematicsInitFailed,
  kCalibrationFailed,
  kIKSolverFailed,
  kTrajectoryExecutionFailed,
  kTimeout,
  kCommunicationTimeout
};

struct Error {
  ErrorCode code;
  std::string message;
};

template<typename T>
using Result = tl::expected<T, Error>;

using Status = Result<void>;

inline Error MakeError(ErrorCode code, const std::string& msg) {
  return {code, msg};
}

namespace constants {

constexpr double GRAVITY = 9.81;

constexpr auto DEFAULT_TIMEOUT = std::chrono::seconds(10);
constexpr auto SERVICE_TIMEOUT = std::chrono::seconds(5);
constexpr int CONTROL_LOOP_HZ = 100;
constexpr int DEFAULT_QUEUE_SIZE = 10;

constexpr int LOG_THROTTLE_MS = 1000;

constexpr double FORCE_THRESHOLD = 0.1;
constexpr double TORQUE_THRESHOLD = 0.1;

constexpr auto SAMPLE_DELAY = std::chrono::milliseconds(100);

constexpr double DEFAULT_MOVEMENT_DURATION = 12.0;

constexpr double CALIBRATION_ANGLE_LARGE = M_PI / 3.0;
constexpr double CALIBRATION_ANGLE_SMALL = M_PI / 6.0;

constexpr double CALIBRATION_INDEX_OFFSET = 1.0;
constexpr double CALIBRATION_MODULO_DIVISOR = 8.0;
constexpr double CALIBRATION_TRAJECTORY_DURATION = 3.0;

constexpr double WORKSPACE_X_MIN = -1.0;
constexpr double WORKSPACE_X_MAX = 1.0;
constexpr double WORKSPACE_Y_MIN = -1.0;
constexpr double WORKSPACE_Y_MAX = 1.0;
constexpr double WORKSPACE_Z_MIN = 0.0;
constexpr double WORKSPACE_Z_MAX = 1.0;

constexpr double ARM_MAX_ACCELERATION = 1.0;

}

namespace CalibrationConstants {
    static constexpr int NUM_POSES = 32;
    static constexpr size_t SAMPLES_PER_POSE = 10;
    static constexpr size_t TOTAL_SAMPLES = NUM_POSES * SAMPLES_PER_POSE;
    static constexpr double SAMPLE_RATE_HZ = 10.0;
    static constexpr double MOTION_DURATION_S = 3.0;

    inline static const std::vector<std::string> UR_JOINT_NAMES = {
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    };
}

namespace kinematics {

struct KinematicsComponents {
    KDL::Tree tree;
    KDL::Chain robot_chain;
    KDL::Frame tool_offset;
    size_t num_joints;
};

Result<KinematicsComponents> InitializeFromUrdf(
    const urdf::Model& urdf_model,
    const std::string& base_link,
    const std::string& tip_link);

Result<KinematicsComponents> InitializeFromUrdfString(
    const std::string& urdf_string,
    const std::string& base_link,
    const std::string& tip_link);

Transform KdlToEigen(const KDL::Frame& frame);

}

namespace logging {

template<typename Logger>
inline void LogVector3(const Logger& logger, const char* prefix, const Vector3d& vec) {
    RCLCPP_INFO(logger, "%s [%.3f, %.3f, %.3f]", prefix, vec.x(), vec.y(), vec.z());
}

template<typename Logger>
inline void LogPose(const Logger& logger, const char* prefix, const Vector3d& pos, const Eigen::Quaterniond& q) {
    RCLCPP_INFO(logger, "%s pos=[%.3f,%.3f,%.3f], quat=[%.3f,%.3f,%.3f,%.3f]",
                prefix, pos.x(), pos.y(), pos.z(), q.x(), q.y(), q.z(), q.w());
}

template<typename Logger>
inline void LogWrench(const Logger& logger, const char* prefix, const Wrench6d& wrench) {
    RCLCPP_INFO(logger, "%s F[%.2f,%.2f,%.2f]N, T[%.3f,%.3f,%.3f]Nm",
                prefix, wrench(0), wrench(1), wrench(2), wrench(3), wrench(4), wrench(5));
}

template<typename Logger>
inline void LogJoints(const Logger& logger, const char* prefix, const JointVector& joints) {
    RCLCPP_INFO(logger, "%s [%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]",
                prefix, joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);
}

}

namespace file_io {

inline Result<void> WriteYamlFile(const std::string& filepath, const YAML::Node& node) {
    try {
        std::filesystem::path path(filepath);
        std::filesystem::create_directories(path.parent_path());
        
        std::ofstream file(filepath);
        if (!file.is_open()) {
            return tl::make_unexpected(MakeError(ErrorCode::kFileNotFound, 
                "Failed to open file for writing: " + filepath));
        }
        
        file << node;
        return {};
    } catch (const std::exception& e) {
        return tl::make_unexpected(MakeError(ErrorCode::kInvalidConfiguration, 
            std::string("Failed to write YAML file: ") + e.what()));
    }
}

inline Result<YAML::Node> ReadYamlFile(const std::string& filepath) {
    try {
        if (!std::filesystem::exists(filepath)) {
            return tl::make_unexpected(MakeError(ErrorCode::kFileNotFound, 
                "File not found: " + filepath));
        }
        
        return YAML::LoadFile(filepath);
    } catch (const std::exception& e) {
        return tl::make_unexpected(MakeError(ErrorCode::kInvalidConfiguration, 
            std::string("Failed to read YAML file: ") + e.what()));
    }
}

}

}