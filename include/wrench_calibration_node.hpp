#pragma once

#include <array>
#include <chrono>
#include <filesystem>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>
#include <yaml-cpp/yaml.h>

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

CalibrationPose computeCalibrationPose(const JointConfiguration& current, int index);

std::vector<CalibrationPose> generate_calibration_poses(const JointConfiguration& current);

Eigen::Vector3d estimateGravitationalForceInBaseFrame(
    const std::vector<CalibrationSample>& samples);

std::pair<Eigen::Matrix3d, Eigen::Vector3d> estimateSensorRotationAndForceBias(
    const std::vector<CalibrationSample>& samples,
    const Eigen::Vector3d& gravity_in_base);

std::pair<Eigen::Vector3d, Eigen::Vector3d> estimateCOMAndTorqueBias(
    const std::vector<CalibrationSample>& samples,
    const Eigen::Vector3d& force_bias);

JointConfiguration read_current_joints(rclcpp::Node::SharedPtr node);

void executeTrajectory(rclcpp::Node::SharedPtr node,
                       const CalibrationPose& target_pose,
                       rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr client);

CalibrationSample collectSingleSample(
    const geometry_msgs::msg::WrenchStamped& wrench,
    tf2_ros::Buffer& tf_buffer,
    size_t pose_index);

std::vector<CalibrationSample> collect_calibration_samples(
    rclcpp::Node::SharedPtr node,
    const std::vector<CalibrationPose>& poses);

void save_calibration_result(const CalibrationResult& result);

} // namespace ur_admittance_controller