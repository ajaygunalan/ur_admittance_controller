#pragma once

#include <array>
#include <chrono>
#include <filesystem>
#include <string>
#include <vector>
#include <fstream>
#include <algorithm>
#include <numeric>
#include <optional>
#include <functional>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <yaml-cpp/yaml.h>
#include <tl/expected.hpp>

namespace ur_admittance_controller {

// ============================================================================
// Type-Safe Units for Compile-Time Safety
// ============================================================================

// Force in Newtons
struct Force {
    Eigen::Vector3d N;
    
    explicit Force(const Eigen::Vector3d& newtons) : N(newtons) {}
    Force() : N(Eigen::Vector3d::Zero()) {}
    
    Force operator+(const Force& other) const { return Force(N + other.N); }
    Force operator-(const Force& other) const { return Force(N - other.N); }
    Force operator/(double scalar) const { return Force(N / scalar); }
    Force& operator+=(const Force& other) { N += other.N; return *this; }
    
    double norm() const { return N.norm(); }
    double x() const { return N.x(); }
    double y() const { return N.y(); }
    double z() const { return N.z(); }
};

// Torque in Newton-meters
struct Torque {
    Eigen::Vector3d Nm;
    
    explicit Torque(const Eigen::Vector3d& newton_meters) : Nm(newton_meters) {}
    Torque() : Nm(Eigen::Vector3d::Zero()) {}
    
    Torque operator+(const Torque& other) const { return Torque(Nm + other.Nm); }
    Torque operator-(const Torque& other) const { return Torque(Nm - other.Nm); }
    Torque operator/(double scalar) const { return Torque(Nm / scalar); }
    Torque& operator+=(const Torque& other) { Nm += other.Nm; return *this; }
    
    double x() const { return Nm.x(); }
    double y() const { return Nm.y(); }
    double z() const { return Nm.z(); }
};

// Mass in kilograms (non-negative)
struct Mass {
    double kg;
    
    explicit Mass(double kilograms) : kg(std::max(0.0, kilograms)) {}
    Mass() : kg(0.0) {}
    
    double value() const { return kg; }
};

// Combined wrench with type-safe forces and torques
struct TypedWrench {
    Force force;
    Torque torque;
    
    TypedWrench(const Force& f, const Torque& t) : force(f), torque(t) {}
    TypedWrench() = default;
    
    // Convert from raw Eigen vector (for legacy compatibility)
    static TypedWrench fromEigen(const Eigen::Matrix<double, 6, 1>& wrench) {
        return TypedWrench(
            Force(wrench.head<3>()),
            Torque(wrench.tail<3>())
        );
    }
    
    // Convert to raw Eigen vector (for legacy compatibility)
    Eigen::Matrix<double, 6, 1> toEigen() const {
        Eigen::Matrix<double, 6, 1> result;
        result << force.N, torque.Nm;
        return result;
    }
};

// ============================================================================
// Monadic Error Handling Utilities
// ============================================================================

// Monadic bind for tl::expected - chain operations that can fail
template<typename T, typename E, typename F>
auto mbind(const tl::expected<T, E>& exp, F&& f) 
    -> decltype(f(std::declval<T>())) {
    if (!exp) {
        return tl::make_unexpected(exp.error());
    }
    return f(exp.value());
}

// Monadic bind for std::optional
template<typename T, typename F>
auto mbind(const std::optional<T>& opt, F&& f) 
    -> decltype(f(std::declval<T>())) {
    if (!opt) {
        using ReturnType = decltype(f(std::declval<T>()));
        return ReturnType{};
    }
    return f(*opt);
}

// Try wrapper - convert exception-throwing code to expected
template<typename F>
auto mtry(F&& f) -> tl::expected<decltype(f()), std::string> {
    try {
        return f();
    } catch (const std::exception& e) {
        return tl::make_unexpected(std::string(e.what()));
    } catch (...) {
        return tl::make_unexpected(std::string("Unknown error"));
    }
}

struct JointConfiguration {
    std::array<double, 6> values;
    
    constexpr double operator[](size_t i) const { return values[i]; }
    constexpr auto begin() const { return values.begin(); }
    constexpr auto end() const { return values.end(); }
};

struct CalibrationPose {
    std::array<double, 6> joints;
};

struct CalibrationSample {
    Eigen::Matrix<double, 6, 1> wrench_raw;
    Eigen::Isometry3d transform_TB;
    size_t pose_index;
};

struct CalibrationResult {
    Eigen::Matrix3d sensor_rotation;
    Force gravity_force;      // Type-safe force in Newtons
    Force force_bias;         // Type-safe force in Newtons
    Torque torque_bias;       // Type-safe torque in Newton-meters
    Eigen::Vector3d center_of_mass;  // Position in meters
    Mass tool_mass;           // Type-safe mass in kilograms
    double installation_roll;   // Robot base roll angle relative to gravity (radians)
    double installation_pitch;  // Robot base pitch angle relative to gravity (radians)
};

constexpr int NUM_POSES = 32;
constexpr int SAMPLES_PER_POSE = 10;
constexpr double GRAVITY = 9.81;

constexpr auto TRAJECTORY_DURATION = std::chrono::seconds(3);
constexpr auto SAMPLE_DELAY = std::chrono::milliseconds(100);
constexpr auto TIMEOUT = std::chrono::seconds(5);

inline const std::array<std::string, 6> JOINT_NAMES{{
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
}};

class WrenchCalibrationNode : public rclcpp::Node {
public:
    WrenchCalibrationNode();
    
    // I/O operations (instance methods - need ROS state)
    JointConfiguration read_current_joints();
    std::vector<CalibrationSample> collect_calibration_samples(const std::vector<CalibrationPose>& poses);
    
    // Pure algorithms (static - no ROS dependency) - now with type-safe units
    static CalibrationPose computeCalibrationPose(const JointConfiguration& current, int index);
    static std::vector<CalibrationPose> generate_calibration_poses(const JointConfiguration& current);
    static Force estimateGravitationalForceInBaseFrame(const std::vector<CalibrationSample>& samples);
    static std::pair<Eigen::Matrix3d, Force> estimateSensorRotationAndForceBias(
        const std::vector<CalibrationSample>& samples,
        const Force& gravity_in_base);
    static std::pair<Eigen::Vector3d, Torque> estimateCOMAndTorqueBias(
        const std::vector<CalibrationSample>& samples,
        const Force& force_bias);
    static std::tuple<Mass, double, double> decomposeGravityVector(const Force& gravity_in_base);
    static void save_calibration_result(const CalibrationResult& result);
    
    // Pure helper functions
    static inline Eigen::Matrix3d makeSkewSymmetric(const Eigen::Vector3d& v) {
        Eigen::Matrix3d skew;
        skew <<     0, -v(2),  v(1),
                 v(2),     0, -v(0),
                -v(1),  v(0),     0;
        return skew;
    }
    
    static inline Eigen::Matrix<double, 6, 1> wrenchMsgToEigen(const geometry_msgs::msg::Wrench& w) {
        return (Eigen::Matrix<double, 6, 1>() << 
            w.force.x, w.force.y, w.force.z,
            w.torque.x, w.torque.y, w.torque.z
        ).finished();
    }
    
    static inline std::filesystem::path getConfigPath(const char* filename) {
        const char* workspace = std::getenv("ROS_WORKSPACE");
        auto base = workspace ? std::filesystem::path(workspace) 
                              : std::filesystem::path(std::getenv("HOME")) / "ros2_ws";
        return base / "src" / "ur_admittance_controller" / "config" / filename;
    }
    
public:
    // Combined logging and saving
    void log_and_save_result(const CalibrationResult& result);
    
private:
    // Helper methods
    void executeTrajectory(const CalibrationPose& target_pose);
    
    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr trajectory_client_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Data streams
    std::optional<geometry_msgs::msg::WrenchStamped> latest_wrench_;
};

} // namespace ur_admittance_controller