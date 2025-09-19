#pragma once
// Critical insights: Interfaces unchanged; only frames W(world), P(probe), S(sensor).
// Wrench topic /netft/proc_probe is assumed compensated and expressed in P.

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

using JointVector = std::array<double, DOF>;

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
constexpr auto DEFAULT_TIMEOUT = std::chrono::seconds(10);
constexpr int CONTROL_LOOP_HZ = 100;
constexpr int DEFAULT_QUEUE_SIZE = 10;
constexpr int LOG_THROTTLE_MS = 1000;
constexpr double FORCE_THRESHOLD = 0.1;
constexpr double TORQUE_THRESHOLD = 0.1;
constexpr double WORKSPACE_X_MIN = -1.0;
constexpr double WORKSPACE_X_MAX =  1.0;
constexpr double WORKSPACE_Y_MIN = -1.0;
constexpr double WORKSPACE_Y_MAX =  1.0;
constexpr double WORKSPACE_Z_MIN =  0.0;
constexpr double WORKSPACE_Z_MAX =  1.0;
constexpr double ARM_MAX_ACCELERATION = 1.0; // translational accel gate (m/s^2)
}  // namespace constants

namespace conversions {
inline Wrench6d FromMsg(const geometry_msgs::msg::WrenchStamped& msg) {
  return (Wrench6d() << msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
                        msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z).finished();
}
}  // namespace conversions

 

namespace logging {
template<typename Logger>
inline void LogPose(const Logger& logger, const char* prefix, const Vector3d& pos, const Eigen::Quaterniond& q) {
  RCLCPP_INFO(logger, "%s pos=[%.3f,%.3f,%.3f], quat=[%.3f,%.3f,%.3f,%.3f]",
              prefix, pos.x(), pos.y(), pos.z(), q.x(), q.y(), q.z(), q.w());
}
} // namespace logging

class AdmittanceNode : public rclcpp::Node {
public:
  explicit AdmittanceNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  void configure();
  void ControlCycle();

private:
  // I/O
  void WrenchCallback(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg);
  void JointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr msg);
  void DesiredPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void OnControlTimer();

  // Core
  Status LoadKinematics();
  void LoadEquilibriumPose(const std::filesystem::path& path);
  bool UpdateWorldVelocityGain(const std::vector<double>& gains);
  void ComputeAdmittance();     // 0–5 (pure math: desired pose + P-wrench)
  void GetXWPCurrent();         // FK for pose error + IK shift
  void ComputePoseError();      // uses g_X_WP_cmd_
  void LimitWorldCommand();     // limiter in WORLD
  void ComputeAndPubJointVelocities();
  void SetAdmittanceGains(const Params::Admittance& params);

  // ROS
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr desired_pose_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;
  std::shared_ptr<ur_admittance_controller::ParamListener> param_listener_;
  ur_admittance_controller::Params params_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_cb_handle_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // State
  std::vector<double> q_current_;
  std::vector<double> q_dot_cmd_;
  std_msgs::msg::Float64MultiArray velocity_msg_;

  Vector6d F_ext_P_ = Vector6d::Zero();
  Vector6d V_cmd_W_ = Vector6d::Zero();
  Vector6d V_prev_W_ = Vector6d::Zero();
  Vector6d deltaX_P_ = Vector6d::Zero();
  Vector6d deltaXdot_P_ = Vector6d::Zero();
  Eigen::Isometry3d X_WP_cmd_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d X_WP_current_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d X_WP_desired_ = Eigen::Isometry3d::Identity();
  Vector6d X_WP_error_ = Vector6d::Zero();

  Vector6d M_inv_diag_;
  Vector6d D_diag_;
  Vector6d K_diag_;
  Vector6d Kp_W_ = (Vector6d() << 4.0, 4.0, 4.0, 2.0, 2.0, 2.0).finished();

  Vector6d workspace_limits_;
  double arm_max_vel_ = 1.5;
  double arm_max_acc_ = constants::ARM_MAX_ACCELERATION;
  double admittance_ratio_ = 1.0;

  // KDL
  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;
  KDL::Frame X_W3P_; // wrist_3_link -> tool
  std::unique_ptr<KDL::ChainIkSolverVel_wdls> ik_vel_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
  size_t num_joints_ = 0;
  KDL::JntArray q_kdl_;
  KDL::JntArray v_kdl_;
  KDL::Frame X_BW3_; // base->wrist (for shift)

public:
  rclcpp::Duration control_period_{std::chrono::milliseconds(10)};
  bool joint_states_received_ = false;

private:
  std::unordered_map<std::string, size_t> joint_name_to_index_;
};

} // namespace ur_admittance_controller
