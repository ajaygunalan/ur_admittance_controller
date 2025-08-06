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

}

namespace frames {
    constexpr const char* ROBOT_BASE_FRAME = "base_link";
    constexpr const char* ROBOT_TOOL_FRAME = "tool0";
    constexpr const char* SENSOR_FRAME = "netft_link1";
    constexpr const char* PROBE_FRAME = "p42v_link1";
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

inline Wrench6d SanitizeWrench(const Wrench6d& wrench) {
    Wrench6d result;
    result.head<3>() = wrench.head<3>().unaryExpr([](double v) {
        return std::abs(v) < 1e-6 ? 0.0 : v;
    });
    result.tail<3>() = wrench.tail<3>().unaryExpr([](double v) {
        return std::abs(v) < 1e-7 ? 0.0 : v;
    });
    return result;
}

class WrenchNode : public rclcpp::Node {
public:
    using WrenchMsg = geometry_msgs::msg::WrenchStamped;

    WrenchNode();

private:
    void WrenchCallback(const WrenchMsg::ConstSharedPtr msg);
    Status LoadCalibrationParams();
    void SetupROSInterfaces();
    void ComputeSensorToProbeAdjoint();


    Wrench6d f_raw_s_ = Wrench6d::Zero();
    Transform X_TB_ = Transform::Identity();
    Vector3d f_grav_s_ = Vector3d::Zero();
    Wrench6d ft_proc_s_ = Wrench6d::Zero();
    Wrench6d wrench_probe_ = Wrench6d::Zero();
    Wrench6d ft_proc_b_ = Wrench6d::Zero();

    Eigen::Matrix<double, 6, 6> adjoint_probe_sensor_ = Eigen::Matrix<double, 6, 6>::Identity();

    Matrix3d R_SE_ = Matrix3d::Identity();
    Vector3d f_grav_b_ = Vector3d::Zero();
    Vector3d f_bias_s_ = Vector3d::Zero();
    Vector3d t_bias_s_ = Vector3d::Zero();
    Vector3d p_CoM_s_ = Vector3d::Zero();

    GravityCompensationParams calibration_params_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<WrenchMsg>::SharedPtr wrench_sub_;
    rclcpp::Publisher<WrenchMsg>::SharedPtr wrench_proc_sensor_pub_;
    rclcpp::Publisher<WrenchMsg>::SharedPtr wrench_proc_probe_pub_;
    rclcpp::Publisher<WrenchMsg>::SharedPtr wrench_proc_probe_base_pub_;
};

}
