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
    Eigen::Vector3d gravity_force;
    Eigen::Vector3d force_bias;
    Eigen::Vector3d torque_bias;
    Eigen::Vector3d center_of_mass;
    double tool_mass;
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
    
    // Pure algorithms (static - no ROS dependency)
    static CalibrationPose computeCalibrationPose(const JointConfiguration& current, int index);
    static std::vector<CalibrationPose> generate_calibration_poses(const JointConfiguration& current);
    static Eigen::Vector3d estimateGravitationalForceInBaseFrame(const std::vector<CalibrationSample>& samples);
    static std::pair<Eigen::Matrix3d, Eigen::Vector3d> estimateSensorRotationAndForceBias(
        const std::vector<CalibrationSample>& samples,
        const Eigen::Vector3d& gravity_in_base);
    static std::pair<Eigen::Vector3d, Eigen::Vector3d> estimateCOMAndTorqueBias(
        const std::vector<CalibrationSample>& samples,
        const Eigen::Vector3d& force_bias);
    static void save_calibration_result(const CalibrationResult& result);
    
    // Pure helper functions
    static inline Eigen::Matrix3d makeSkewSymmetric(const Eigen::Vector3d& v) {
        Eigen::Matrix3d skew;
        skew <<     0, -v(2),  v(1),
                 v(2),     0, -v(0),
                -v(1),  v(0),     0;
        return skew;
    }
    
    // Combined logging and saving
    void log_and_save_result(const CalibrationResult& result);
    
private:
    // Helper methods
    void executeTrajectory(const CalibrationPose& target_pose);
    tl::expected<CalibrationSample, std::string> collectSingleSample(size_t pose_index);
    
    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr trajectory_client_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Data streams
    std::optional<geometry_msgs::msg::WrenchStamped> latest_wrench_;
};

} // namespace ur_admittance_controller