#include "wrench_calibration_node.hpp"

#include <rclcpp/wait_for_message.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <fstream>
#include <algorithm>
#include <numeric>

using namespace ur_admittance_controller;
using namespace ur_admittance_controller::constants;

namespace ur_admittance_controller {

CalibrationPose computeCalibrationPose(const JointConfiguration& current, int index) {
    constexpr double ANGLE_LARGE = M_PI / 3.0;
    constexpr double ANGLE_SMALL = M_PI / 6.0;
    constexpr double MODULO_DIV = 8.0;
    
    CalibrationPose pose{{current.values[0], current.values[1], current.values[2],
                         current.values[3], current.values[4], current.values[5]}};
    
    if (index > 0) {
        const double idx = static_cast<double>(index - 1);
        const double normalized_idx = idx / NUM_POSES;
        const double modulated_idx = std::fmod(idx, MODULO_DIV) / MODULO_DIV;
        
        pose.joints[3] += normalized_idx * ANGLE_LARGE - ANGLE_SMALL;
        pose.joints[4] += modulated_idx * ANGLE_LARGE - ANGLE_SMALL;
        pose.joints[5] += idx * M_PI / NUM_POSES - M_PI / 2.0;
    }
    
    return pose;
}

std::vector<CalibrationPose> generate_calibration_poses(const JointConfiguration& current) {
    std::vector<CalibrationPose> poses(NUM_POSES);
    std::generate_n(poses.begin(), NUM_POSES,
                    [&current, n = 0]() mutable {
                        return computeCalibrationPose(current, n++);
                    });
    return poses;
}

void save_calibration_result(const CalibrationResult& result) {
    YAML::Node config;
    
    YAML::Node rotation;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            rotation.push_back(result.sensor_rotation(i, j));
    config["sensor_rotation"] = rotation;
    
    YAML::Node force_bias;
    for (int i = 0; i < 3; ++i)
        force_bias.push_back(result.force_bias(i));
    config["force_bias"] = force_bias;
    
    YAML::Node torque_bias;
    for (int i = 0; i < 3; ++i)
        torque_bias.push_back(result.torque_bias(i));
    config["torque_bias"] = torque_bias;
    
    config["tool_mass"] = result.tool_mass;
    
    YAML::Node com;
    for (int i = 0; i < 3; ++i)
        com.push_back(result.center_of_mass(i));
    config["center_of_mass"] = com;
    
    YAML::Node gravity;
    for (int i = 0; i < 3; ++i)
        gravity.push_back(result.gravity_force(i));
    config["gravity_force"] = gravity;
    
    const char* workspace = std::getenv("ROS_WORKSPACE");
    std::filesystem::path base = workspace ? std::filesystem::path(workspace) : 
                                            std::filesystem::path(std::getenv("HOME")) / "ros2_ws";
    std::filesystem::path path = base / "src" / "ur_admittance_controller" / "config" / "wrench_calibration.yaml";
    
    std::filesystem::create_directories(path.parent_path());
    std::ofstream file(path);
    file << config;
}

JointConfiguration read_current_joints(rclcpp::Node::SharedPtr node) {
    sensor_msgs::msg::JointState joint_msg;
    rclcpp::wait_for_message(joint_msg, node, "/joint_states", TIMEOUT);
    
    JointConfiguration config;
    std::transform(JOINT_NAMES.begin(), 
                   JOINT_NAMES.end(),
                   config.values.begin(),
                   [&joint_msg](const std::string& name) {
                       std::vector<std::string>::iterator it = std::find(joint_msg.name.begin(), joint_msg.name.end(), name);
                       return joint_msg.position[std::distance(joint_msg.name.begin(), it)];
                   });
    
    return config;
}

void executeTrajectory(rclcpp::Node::SharedPtr node, const CalibrationPose& target_pose, rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr client) {
    using TrajectoryAction = control_msgs::action::FollowJointTrajectory;
    
    TrajectoryAction::Goal goal;
    goal.trajectory.joint_names = {JOINT_NAMES.begin(), JOINT_NAMES.end()};
    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions = {target_pose.joints.begin(), target_pose.joints.end()};
    goal.trajectory.points[0].time_from_start = rclcpp::Duration(TRAJECTORY_DURATION);
    
    // Send goal and wait for it to be accepted
    auto goal_handle_future = client->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(node, goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to send goal");
        return;
    }
    
    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(node->get_logger(), "Goal was rejected");
        return;
    }
    
    // Wait for the result
    auto result_future = client->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to get result");
        return;
    }
    
    // Let robot settle
    rclcpp::sleep_for(std::chrono::seconds(1));
}

CalibrationSample collectSingleSample(const geometry_msgs::msg::WrenchStamped& wrench, tf2_ros::Buffer& tf_buffer, size_t pose_index) {
    
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer.lookupTransform("base_link", "tool0", tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(rclcpp::get_logger("wrench_calibration"), "Transform error: %s", ex.what());
        throw;
    }
    
    CalibrationSample sample;
    sample.wrench_raw = (Eigen::Matrix<double, 6, 1>() << 
        wrench.wrench.force.x,  wrench.wrench.force.y,  wrench.wrench.force.z,
        wrench.wrench.torque.x, wrench.wrench.torque.y, wrench.wrench.torque.z).finished();
    sample.transform_TB = tf2::transformToEigen(transform);
    sample.pose_index = pose_index;
    
    return sample;
}

std::vector<CalibrationSample> collect_calibration_samples(rclcpp::Node::SharedPtr node, const std::vector<CalibrationPose>& poses) {
    
    using TrajectoryAction = control_msgs::action::FollowJointTrajectory;
    
    rclcpp_action::Client<TrajectoryAction>::SharedPtr trajectory_client = rclcpp_action::create_client<TrajectoryAction>(
        node, "/scaled_joint_trajectory_controller/follow_joint_trajectory");
    trajectory_client->wait_for_action_server(TIMEOUT);
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    
    // Wait for TF to be available
    rclcpp::sleep_for(std::chrono::seconds(1));
    
    geometry_msgs::msg::WrenchStamped latest_wrench;
    bool has_wrench = false;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub = node->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/netft/raw_sensor", 10,
        [&](const geometry_msgs::msg::WrenchStamped::ConstSharedPtr& msg) {
            latest_wrench = *msg;
            has_wrench = true;
        });
    
    // Wait for first wrench message
    while (!has_wrench && rclcpp::ok()) {
        rclcpp::spin_some(node);
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    
    std::vector<CalibrationSample> samples;
    samples.reserve(poses.size() * SAMPLES_PER_POSE);
    
    for (size_t pose_idx = 0; pose_idx < poses.size(); ++pose_idx) {
        RCLCPP_INFO(node->get_logger(), "Pose %zu/%zu", pose_idx + 1, poses.size());
        
        if (pose_idx > 0) {
            executeTrajectory(node, poses[pose_idx], trajectory_client);
        }
        
        for (int i = 0; i < SAMPLES_PER_POSE; ++i) {
            rclcpp::spin_some(node);
            
            if (!has_wrench) {
                RCLCPP_WARN(node->get_logger(), "No wrench data available");
                continue;
            }
            
            try {
                samples.push_back(collectSingleSample(latest_wrench, *tf_buffer, pose_idx));
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN(node->get_logger(), "Failed to collect sample %d at pose %zu: %s", i+1, pose_idx+1, ex.what());
                continue;
            }
            rclcpp::sleep_for(SAMPLE_DELAY);
        }
        
        // Return to home after each pose (except pose 0 which is already home)
        if (pose_idx > 0) {
            executeTrajectory(node, poses[0], trajectory_client);
        }
    }
    
    RCLCPP_INFO(node->get_logger(), "Collected %zu samples", samples.size());
    return samples;
}

}

int main(int argc, char** argv) {
    using namespace ur_admittance_controller;
    using namespace ur_admittance_controller::constants;
    
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("wrench_calibration_node");
    
    RCLCPP_INFO(node->get_logger(), "Initializing calibration system...");
    
    JointConfiguration current_joints = read_current_joints(node);
    
    std::vector<CalibrationPose> poses = generate_calibration_poses(current_joints);
    RCLCPP_INFO(node->get_logger(), "Generated %zu calibration poses", poses.size());
    
    RCLCPP_INFO(node->get_logger(), "Data collection at %zu poses", poses.size());
    std::vector<CalibrationSample> samples = collect_calibration_samples(node, poses);
    
    RCLCPP_INFO(node->get_logger(), "Processing %zu samples", samples.size());
    
    Eigen::Vector3d gravity = estimateGravitationalForceInBaseFrame(samples);
    
    std::pair<Eigen::Matrix3d, Eigen::Vector3d> rotation_and_bias = estimateSensorRotationAndForceBias(samples, gravity);
    Eigen::Matrix3d R_SE = rotation_and_bias.first;
    Eigen::Vector3d force_bias = rotation_and_bias.second;
    
    std::pair<Eigen::Vector3d, Eigen::Vector3d> com_and_torque = estimateCOMAndTorqueBias(samples, force_bias);
    Eigen::Vector3d com = com_and_torque.first;
    Eigen::Vector3d torque_bias = com_and_torque.second;
    
    double tool_mass = gravity.norm() / GRAVITY;
    
    CalibrationResult result{R_SE, gravity, force_bias, torque_bias, com, tool_mass};
    
    RCLCPP_INFO(node->get_logger(), "Calibration complete:");
    RCLCPP_INFO(node->get_logger(), "  Tool mass: %.3f kg", result.tool_mass);
    RCLCPP_INFO(node->get_logger(), "  COM: [%.3f, %.3f, %.3f] m", 
        result.center_of_mass.x(), result.center_of_mass.y(), result.center_of_mass.z());
    RCLCPP_INFO(node->get_logger(), "  Force bias: [%.2f, %.2f, %.2f] N", 
        result.force_bias.x(), result.force_bias.y(), result.force_bias.z());
    RCLCPP_INFO(node->get_logger(), "  Torque bias: [%.3f, %.3f, %.3f] Nm", 
        result.torque_bias.x(), result.torque_bias.y(), result.torque_bias.z());
    
    save_calibration_result(result);
    
    const char* workspace = std::getenv("ROS_WORKSPACE");
    std::filesystem::path base = workspace ? std::filesystem::path(workspace) : 
                                            std::filesystem::path(std::getenv("HOME")) / "ros2_ws";
    std::filesystem::path config_path = base / "src" / "ur_admittance_controller" / "config" / "wrench_calibration.yaml";
    
    RCLCPP_INFO(node->get_logger(), "Saved to %s", config_path.c_str());
    RCLCPP_INFO(node->get_logger(), "Calibration completed successfully");
    
    rclcpp::shutdown();
    return 0;
}