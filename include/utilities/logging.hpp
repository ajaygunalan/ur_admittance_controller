#pragma once

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <utilities/types.hpp>

namespace ur_admittance_controller {
namespace logging {

// Log 3D vector with format: [x, y, z]
template<typename Logger>
inline void LogVector3(const Logger& logger, const char* prefix, const Vector3d& vec) {
    RCLCPP_INFO(logger, "%s [%.3f, %.3f, %.3f]", prefix, vec.x(), vec.y(), vec.z());
}

// Log pose with position and quaternion
template<typename Logger>
inline void LogPose(const Logger& logger, const char* prefix, const Vector3d& pos, const Eigen::Quaterniond& q) {
    RCLCPP_INFO(logger, "%s pos=[%.3f,%.3f,%.3f], quat=[%.3f,%.3f,%.3f,%.3f]",
                prefix, pos.x(), pos.y(), pos.z(), q.x(), q.y(), q.z(), q.w());
}

// Log wrench with forces and torques
template<typename Logger>
inline void LogWrench(const Logger& logger, const char* prefix, const Wrench6d& wrench) {
    RCLCPP_INFO(logger, "%s F[%.2f,%.2f,%.2f]N, T[%.3f,%.3f,%.3f]Nm",
                prefix, wrench(0), wrench(1), wrench(2), wrench(3), wrench(4), wrench(5));
}

// Log joint configuration
template<typename Logger>
inline void LogJoints(const Logger& logger, const char* prefix, const JointVector& joints) {
    RCLCPP_INFO(logger, "%s [%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]",
                prefix, joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);
}

} // namespace logging
} // namespace ur_admittance_controller