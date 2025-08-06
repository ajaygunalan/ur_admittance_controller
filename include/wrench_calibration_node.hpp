#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fmt/core.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rcpputils/scope_exit.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tl/expected.hpp>
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
using JointAngles = std::vector<double>;
using JointNames = std::vector<std::string>;
using PoseSequence = std::vector<JointAngles>;

struct GravityCompensationParams {
    Matrix3d R_SE;
    Vector3d f_grav_b;
    Vector3d f_bias_s;
    Vector3d t_bias_s;
    Vector3d p_CoM_s;
};

struct CalibrationSample {
    Wrench6d F_S_S_raw;
    Transform X_TB;
    size_t pose_index;
};

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

namespace frames {
    constexpr const char* ROBOT_BASE_FRAME = "base_link";
    constexpr const char* ROBOT_TOOL_FRAME = "tool0";
    constexpr const char* SENSOR_FRAME = "netft_link1";
    constexpr const char* PROBE_FRAME = "p42v_link1";
}

namespace CalibrationConstants {
    static constexpr int NUM_POSES = 32;
    static constexpr size_t SAMPLES_PER_POSE = 10;
    static constexpr size_t TOTAL_SAMPLES = NUM_POSES * SAMPLES_PER_POSE;
    static constexpr double SAMPLE_RATE_HZ = 10.0;
    static constexpr double MOTION_DURATION_S = 3.0;

    inline static const JointNames UR_JOINT_NAMES = {
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    };
}

namespace conversions {

inline Wrench6d FromMsg(const geometry_msgs::msg::WrenchStamped& msg) {
    return (Wrench6d() << msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
                          msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z).finished();
}

inline Wrench6d FromMsg(const geometry_msgs::msg::Wrench& wrench) {
    return (Wrench6d() << wrench.force.x, wrench.force.y, wrench.force.z,
                          wrench.torque.x, wrench.torque.y, wrench.torque.z).finished();
}

inline void FillMsg(const Wrench6d& wrench, geometry_msgs::msg::WrenchStamped& msg) {
    msg.wrench.force.x = wrench[0];
    msg.wrench.force.y = wrench[1];
    msg.wrench.force.z = wrench[2];
    msg.wrench.torque.x = wrench[3];
    msg.wrench.torque.y = wrench[4];
    msg.wrench.torque.z = wrench[5];
}

geometry_msgs::msg::WrenchStamped ToMsg(
    const Wrench6d& wrench,
    const std::string& frame_id,
    const rclcpp::Time& stamp);

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
    if (joints.size() >= 6) {
        RCLCPP_INFO(logger, "%s [%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]",
                    prefix, joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);
    }
}

}

class WrenchCalibrationNode : public rclcpp::Node {
public:
    using TrajectoryAction = control_msgs::action::FollowJointTrajectory;
    using TrajectoryClient = rclcpp_action::Client<TrajectoryAction>;
    using JointStateMsg = sensor_msgs::msg::JointState;
    using WrenchMsg = geometry_msgs::msg::WrenchStamped;


    WrenchCalibrationNode();
    Status Initialize();
    Status ExecuteCalibrationSequence();
    Status ComputeCalibrationParameters();

private:
    void UpdateJointPositions(const JointStateMsg::ConstSharedPtr& msg);
    void CollectSamplesAtCurrentPose(std::vector<CalibrationSample>& samples, size_t pose_idx);
    void GenerateCalibrationPoses();
    Status MoveToJointPosition(const JointAngles& target_joints);
    Status SaveCalibrationToYaml();

    TrajectoryClient::SharedPtr trajectory_client_;
    rclcpp::Subscription<JointStateMsg>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<WrenchMsg>::SharedPtr wrench_sub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;


    WrenchMsg latest_wrench_;
    JointAngles current_joint_positions_;
    PoseSequence calibration_poses_;
    std::vector<CalibrationSample> calibration_samples_;
    std::atomic<bool> has_wrench_{false};
    std::atomic<bool> has_joint_states_{false};

    GravityCompensationParams calibration_params_;
    bool calibration_computed_{false};


    inline static const JointNames joint_names_ = CalibrationConstants::UR_JOINT_NAMES;
    std::string robot_base_frame_;
    std::string robot_tool_frame_;
};

Result<Force3d> estimateGravitationalForceInBaseFrame(
    const std::vector<CalibrationSample>& samples);

Result<std::pair<Matrix3d, Force3d>> estimateSensorRotationAndForceBias(
    const std::vector<CalibrationSample>& samples,
    const Force3d& f_gravity_B);

Result<std::pair<Vector3d, Torque3d>> estimateCOMAndTorqueBias(
    const std::vector<CalibrationSample>& samples,
    const Force3d& f_bias_S);

Result<Matrix3d> estimateRobotInstallationBias(const Force3d& f_gravity_B);

Result<double> extractToolMass(const Force3d& f_gravity_B);

}
