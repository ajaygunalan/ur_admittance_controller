#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fmt/core.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_kdl/tf2_kdl.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <kdl/frames.hpp>
#include <tl/expected.hpp>
#include <yaml-cpp/yaml.h>

namespace ur_admittance_controller {

// ---------- Common typedefs ----------
using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;
using Transform = Eigen::Isometry3d;
using Wrench6d  = Eigen::Matrix<double, 6, 1>;
using Force3d   = Eigen::Vector3d;
using Torque3d  = Eigen::Vector3d;

// ---------- Parameters / error plumbing ----------
struct GravityCompensationParams {
  Matrix3d R_SE;     // rotation: {E}←{S}
  Vector3d f_grav_b; // gravity in {B}
  Vector3d f_bias_s; // force bias in {S}
  Vector3d t_bias_s; // torque bias in {S}
  Vector3d p_CoM_s;  // CoM vector in {S}
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
  ErrorCode code{};
  std::string message;
};

template<typename T>
using Result = tl::expected<T, Error>;
using Status = Result<void>;

inline Error MakeError(ErrorCode code, const std::string& msg) { return {code, msg}; }

// ---------- Constants / frames ----------
namespace constants {
  constexpr int LOG_THROTTLE_MS = 1000;
  constexpr double FORCE_THRESHOLD  = 0.1;  // for informative logging
  constexpr double TORQUE_THRESHOLD = 0.1;
}

namespace frames {
  constexpr const char* ROBOT_BASE_FRAME = "base_link";
  constexpr const char* ROBOT_TOOL_FRAME = "tool0";
  constexpr const char* SENSOR_FRAME     = "netft_link1";
  constexpr const char* PROBE_FRAME      = "p42v_link1";
}

// ---------- Conversions ----------
namespace conversions {
inline Wrench6d FromMsg(const geometry_msgs::msg::WrenchStamped& msg) {
  return (Wrench6d() << msg.wrench.force.x,  msg.wrench.force.y,  msg.wrench.force.z,
                         msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z).finished();
}

inline Wrench6d FromMsg(const geometry_msgs::msg::Wrench& w) {
  return (Wrench6d() << w.force.x,  w.force.y,  w.force.z,
                         w.torque.x, w.torque.y, w.torque.z).finished();
}

inline void FillMsg(const Wrench6d& w, geometry_msgs::msg::WrenchStamped& msg) {
  msg.wrench.force.x  = w[0];
  msg.wrench.force.y  = w[1];
  msg.wrench.force.z  = w[2];
  msg.wrench.torque.x = w[3];
  msg.wrench.torque.y = w[4];
  msg.wrench.torque.z = w[5];
}

// Implemented in the .cpp (kept as a small helper there)
geometry_msgs::msg::WrenchStamped ToMsg(const Wrench6d& wrench,
                                        const std::string& frame_id,
                                        const rclcpp::Time& stamp);
} // namespace conversions

// --------- small utility ----------
inline Wrench6d SanitizeWrench(const Wrench6d& w) {
  Wrench6d out;
  out.head<3>() = w.head<3>().unaryExpr([](double v){ return std::abs(v) < 1e-6 ? 0.0 : v; });
  out.tail<3>() = w.tail<3>().unaryExpr([](double v){ return std::abs(v) < 1e-7 ? 0.0 : v; });
  return out;
}

// ---------- Node ----------
class WrenchNode : public rclcpp::Node {
public:
  using WrenchMsg = geometry_msgs::msg::WrenchStamped;
  WrenchNode();

private:
  void   WrenchCallback(const WrenchMsg::ConstSharedPtr msg);
  Status LoadCalibrationParams();
  void   SetupROSInterfaces();
  void   InitStaticTransformOnce();

  // --- state (SENSOR frame unless stated otherwise) ---
  Wrench6d  f_raw_s_   = Wrench6d::Zero();
  Transform X_TB_      = Transform::Identity();  // TOOL←BASE (for gravity mapping)
  Wrench6d  ft_proc_s_ = Wrench6d::Zero();

  // Cached static SENSOR→PROBE transform for tf2_kdl::doTransform
  geometry_msgs::msg::TransformStamped T_SP_;
  bool sp_ready_ = false;
  rclcpp::TimerBase::SharedPtr init_timer_;

  // Calibration
  Matrix3d R_SE_   = Matrix3d::Identity();
  Vector3d f_grav_b_ = Vector3d::Zero();
  Vector3d f_bias_s_ = Vector3d::Zero();
  Vector3d t_bias_s_ = Vector3d::Zero();
  Vector3d p_CoM_s_  = Vector3d::Zero();
  GravityCompensationParams calibration_params_;

  // TF
  std::unique_ptr<tf2_ros::Buffer>           tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ROS I/O
  rclcpp::Subscription<WrenchMsg>::SharedPtr wrench_sub_;
  rclcpp::Publisher<WrenchMsg>::SharedPtr    wrench_proc_sensor_pub_;
  rclcpp::Publisher<WrenchMsg>::SharedPtr    wrench_proc_probe_pub_;

  // Filtering
  Wrench6d lpf_state_ = Wrench6d::Zero();
  bool     first_sample_ = true;
  static constexpr double ALPHA            = 0.715; // fc≈200 Hz, ΔT≈0.002 s
  static constexpr double DEADBAND_FORCE   = 0.5;   // N
  static constexpr double DEADBAND_TORQUE  = 0.01;  // N·m
};

} 
