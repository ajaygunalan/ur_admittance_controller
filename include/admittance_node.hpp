
#pragma once

/**
 * @file admittance_node.hpp
 * @brief ROS 2 node interface (I/O, params, timers). Math lives in *_computations.*
 *
 * External interface is unchanged (topics/params/services), but internals are
 * reorganized to follow the math pseudocode line-by-line for readability.      fileciteturn0file4
 */

#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fmt/core.h>
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tl/expected.hpp>
#include <urdf/model.h>
#include <yaml-cpp/yaml.h>

#include "ur_admittance_controller/ur_admittance_controller_parameters.hpp"

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

using JointVector = std::array<double, DOF>;

struct GravityCompensationParams {
    Matrix3d R_SE;
    Vector3d f_grav_b;
    Vector3d f_bias_s;
    Vector3d t_bias_s;
    Vector3d p_CoM_s;
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

}  // namespace constants

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

}  // namespace conversions

inline double SanitizeJointAngle(double angle) {
    return std::abs(angle) < 1e-12 ? 0.0 : std::round(angle * 10000.0) / 10000.0;
}

inline Vector3d SanitizeForce(const Vector3d& force) {
    return force.unaryExpr([](double v) {
        return std::abs(v) < 1e-6 ? 0.0 : v;
    });
}

inline Vector3d SanitizeTorque(const Vector3d& torque) {
    return torque.unaryExpr([](double v) {
        return std::abs(v) < 1e-7 ? 0.0 : v;
    });
}

inline Wrench6d SanitizeWrench(const Wrench6d& wrench) {
    Wrench6d result;
    result.head<3>() = SanitizeForce(wrench.head<3>());
    result.tail<3>() = SanitizeTorque(wrench.tail<3>());
    return result;
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

}  // namespace kinematics

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

}  // namespace logging

class AdmittanceNode : public rclcpp::Node {
public:
  explicit AdmittanceNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  void ControlCycle();
  void configure();

private:
  // ---- ROS I/O callbacks ----
  void WrenchCallback(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg);
  void JointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr msg);
  void DesiredPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);

  // ---- Core control pieces (implemented in *_computations.cpp) ----
  Status LoadKinematics();
  void ComputeAdmittance();     // Steps 0–5 (no FK needed)
  void GetXBPCurrent();         // FK for Step 6 (IK) & error logging
  void ComputePoseError();      // uses g_X_WB_cmd from ComputeAdmittance
  void ComputeAndPubJointVelocities();
  void LimitToWorkspace();
  void SetAdmittanceGains(const Params::Admittance& params);

  // ---- ROS I/O ----
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr desired_pose_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;
  std::shared_ptr<ur_admittance_controller::ParamListener> param_listener_;
  ur_admittance_controller::Params params_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_cb_handle_;
  std::vector<double> q_current_;
  std::vector<double> q_dot_cmd_;

  // ---- Controller state & params ----
  Vector6d F_P_B = Vector6d::Zero();          // Assumed WORLD/base wrench (see README).  fileciteturn0file5
  Vector6d V_P_B_commanded = Vector6d::Zero(); // World twist command
  Vector6d M_inverse_diag;
  Vector6d D_diag;
  Vector6d K_diag;

  Eigen::Isometry3d X_BP_current = Eigen::Isometry3d::Identity(); // world
  Eigen::Isometry3d X_BP_desired = Eigen::Isometry3d::Identity(); // world
  Vector6d X_BP_error = Vector6d::Zero();
  std_msgs::msg::Float64MultiArray velocity_msg_;

  Vector6d workspace_limits_;
  double arm_max_vel_;
  double arm_max_acc_;
  double admittance_ratio_;

  // ---- Kinematics ----
  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;
  KDL::Frame X_W3P;
  std::unique_ptr<KDL::ChainIkSolverVel_wdls> ik_vel_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
  size_t num_joints_ = 0;
  KDL::JntArray q_kdl_;
  KDL::JntArray v_kdl_;
  KDL::Frame X_BW3;

public:
  rclcpp::Duration control_period_{std::chrono::milliseconds(10)};
  bool joint_states_received_ = false;

private:
  std::unordered_map<std::string, size_t> joint_name_to_index_;
};

}  // namespace ur_admittance_controller
