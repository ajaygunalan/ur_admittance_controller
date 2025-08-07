// F/T sensor calibration node - collects data from 32 robot poses for LROM calibration
#include "wrench_calibration_node.hpp"

#include <rclcpp/wait_for_message.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <fstream>

namespace ur_admittance_controller {

// ============================================================================
// Pure Functions Implementation
// ============================================================================

std::vector<CalibrationPose> generate_calibration_poses(const JointConfiguration& current) {
    std::vector<CalibrationPose> poses;
    poses.reserve(constants::NUM_POSES);
    
    // Constants for calibration motion
    constexpr double CALIBRATION_ANGLE_LARGE = M_PI / 3.0;  // 60 degrees
    constexpr double CALIBRATION_ANGLE_SMALL = M_PI / 6.0;  // 30 degrees
    constexpr double CALIBRATION_MODULO_DIVISOR = 8.0;
    
    // Generate 32 poses by varying wrist joints only (safer)
    for (int i = 0; i < constants::NUM_POSES; ++i) {
        CalibrationPose pose;
        std::copy(current.begin(), current.end(), pose.joints.begin());
        
        if (i > 0) {  // First pose stays at current position
            const double idx = static_cast<double>(i - 1);
            
            // Wrist 1 (joint 3): Gradual sweep
            pose.joints[3] = current.values[3] + 
                (idx / constants::NUM_POSES) * CALIBRATION_ANGLE_LARGE - CALIBRATION_ANGLE_SMALL;
            
            // Wrist 2 (joint 4): Modulated sweep  
            pose.joints[4] = current.values[4] +
                (std::fmod(idx, CALIBRATION_MODULO_DIVISOR) / CALIBRATION_MODULO_DIVISOR) * 
                CALIBRATION_ANGLE_LARGE - CALIBRATION_ANGLE_SMALL;
            
            // Wrist 3 (joint 5): Full rotation sweep
            pose.joints[5] = current.values[5] + 
                idx * M_PI / constants::NUM_POSES - M_PI / 2.0;
        }
        
        poses.push_back(pose);
    }
    
    return poses;
}

// Note: compute_calibration implementation is in wrench_calibration_algorithm.cpp

YAML::Node build_yaml_config(const CalibrationResult& result) {
    YAML::Node config;
    
    // Sensor rotation matrix
    YAML::Node rotation;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            rotation.push_back(result.sensor_rotation(i, j));
        }
    }
    config["sensor_rotation"] = rotation;
    
    // Force bias
    YAML::Node force_bias;
    for (int i = 0; i < 3; ++i) {
        force_bias.push_back(result.force_bias(i));
    }
    config["force_bias"] = force_bias;
    
    // Torque bias
    YAML::Node torque_bias;
    for (int i = 0; i < 3; ++i) {
        torque_bias.push_back(result.torque_bias(i));
    }
    config["torque_bias"] = torque_bias;
    
    // Tool parameters
    config["tool_mass"] = result.tool_mass;
    
    YAML::Node com;
    for (int i = 0; i < 3; ++i) {
        com.push_back(result.center_of_mass(i));
    }
    config["center_of_mass"] = com;
    
    // Gravity
    YAML::Node gravity;
    for (int i = 0; i < 3; ++i) {
        gravity.push_back(result.gravity_force(i));
    }
    config["gravity_force"] = gravity;
    
    return config;
}

std::filesystem::path get_calibration_config_path() {
    // Get workspace path from environment or use default
    std::string workspace = std::getenv("ROS_WORKSPACE") ?
                           std::getenv("ROS_WORKSPACE") :
                           std::string(std::getenv("HOME")) + "/ros2_ws";
    
    return std::filesystem::path(workspace) / "src" / "ur_admittance_controller" / 
           "config" / "wrench_calibration.yaml";
}

// ============================================================================
// IO Operations Implementation
// ============================================================================

JointConfiguration read_current_joints(rclcpp::Node::SharedPtr node) {
    RCLCPP_INFO(node->get_logger(), "Reading current joint positions...");
    
    sensor_msgs::msg::JointState joint_msg;
    if (!rclcpp::wait_for_message(joint_msg, node, "/joint_states", constants::TIMEOUT)) {
        throw std::runtime_error("Failed to receive joint states");
    }
    
    JointConfiguration config;
    for (size_t i = 0; i < constants::JOINT_NAMES.size(); ++i) {
        auto it = std::find(joint_msg.name.begin(), joint_msg.name.end(), 
                           constants::JOINT_NAMES[i]);
        if (it != joint_msg.name.end()) {
            size_t idx = std::distance(joint_msg.name.begin(), it);
            config.values[i] = joint_msg.position[idx];
        }
    }
    
    return config;
}

std::vector<CalibrationSample> collect_calibration_samples(
    rclcpp::Node::SharedPtr node,
    const std::vector<CalibrationPose>& poses) {
    
    RCLCPP_INFO(node->get_logger(), "Starting calibration data collection...");
    
    // Create action client
    using TrajectoryAction = control_msgs::action::FollowJointTrajectory;
    auto trajectory_client = rclcpp_action::create_client<TrajectoryAction>(
        node, "/scaled_joint_trajectory_controller/follow_joint_trajectory");
    
    if (!trajectory_client->wait_for_action_server(constants::TIMEOUT)) {
        throw std::runtime_error("Action server not available");
    }
    
    // Setup TF listener
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    
    // Setup wrench subscriber
    geometry_msgs::msg::WrenchStamped latest_wrench;
    bool has_wrench = false;
    auto wrench_sub = node->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/netft/raw_sensor", 10,
        [&](const geometry_msgs::msg::WrenchStamped::ConstSharedPtr& msg) {
            latest_wrench = *msg;
            has_wrench = true;
        });
    
    std::vector<CalibrationSample> samples;
    samples.reserve(poses.size() * constants::SAMPLES_PER_POSE);
    
    // Helper lambda to move to a specific pose
    auto move_to_pose = [&](const CalibrationPose& target_pose) -> bool {
        auto goal_msg = TrajectoryAction::Goal();
        goal_msg.trajectory.joint_names = std::vector<std::string>(
            constants::JOINT_NAMES.begin(), constants::JOINT_NAMES.end());
        
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = std::vector<double>(target_pose.joints.begin(), target_pose.joints.end());
        point.time_from_start = rclcpp::Duration(constants::TRAJECTORY_DURATION);
        goal_msg.trajectory.points.push_back(point);
        
        auto goal_handle_future = trajectory_client->async_send_goal(goal_msg);
        if (rclcpp::spin_until_future_complete(node, goal_handle_future) != 
            rclcpp::FutureReturnCode::SUCCESS) {
            return false;
        }
        
        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            return false;
        }
        
        auto result_future = trajectory_client->async_get_result(goal_handle);
        if (rclcpp::spin_until_future_complete(node, result_future) != 
            rclcpp::FutureReturnCode::SUCCESS) {
            return false;
        }
        
        // Let robot settle
        rclcpp::sleep_for(std::chrono::seconds(1));
        return true;
    };
    
    // Process each pose
    for (size_t pose_idx = 0; pose_idx < poses.size(); ++pose_idx) {
        const auto& pose = poses[pose_idx];
        
        RCLCPP_INFO(node->get_logger(), "Moving to pose %zu/%zu", 
                   pose_idx + 1, poses.size());
        
        // Move to calibration pose
        if (!move_to_pose(pose)) {
            RCLCPP_ERROR(node->get_logger(), "Failed to move to pose %zu", pose_idx + 1);
            continue;
        }
        
        // Collect samples at this pose
        for (int sample = 0; sample < constants::SAMPLES_PER_POSE; ++sample) {
            rclcpp::spin_some(node);
            
            if (!has_wrench) {
                RCLCPP_WARN(node->get_logger(), "No wrench data available");
                continue;
            }
            
            // Get transform
            geometry_msgs::msg::TransformStamped transform_msg;
            try {
                transform_msg = tf_buffer->lookupTransform(
                    "base_link", "tool0", tf2::TimePointZero);
            } catch (const tf2::TransformException& ex) {
                RCLCPP_ERROR(node->get_logger(), "Transform error: %s", ex.what());
                continue;
            }
            
            // Create sample
            CalibrationSample sample_data;
            sample_data.wrench_raw << 
                latest_wrench.wrench.force.x,
                latest_wrench.wrench.force.y,
                latest_wrench.wrench.force.z,
                latest_wrench.wrench.torque.x,
                latest_wrench.wrench.torque.y,
                latest_wrench.wrench.torque.z;
            
            sample_data.transform_TB = tf2::transformToEigen(transform_msg);
            sample_data.pose_index = pose_idx;
            
            samples.push_back(sample_data);
            
            rclcpp::sleep_for(constants::SAMPLE_DELAY);
        }
        
        RCLCPP_INFO(node->get_logger(), "Collected %d samples at pose %zu",
                   constants::SAMPLES_PER_POSE, pose_idx + 1);
        
        // Return to home position (pose 0) after each calibration pose (except after the first)
        if (pose_idx > 0) {
            RCLCPP_INFO(node->get_logger(), "Returning to home position");
            if (!move_to_pose(poses[0])) {
                RCLCPP_WARN(node->get_logger(), "Failed to return to home position");
            }
        }
    }
    
    RCLCPP_INFO(node->get_logger(), "Collection complete: %zu total samples", 
               samples.size());
    
    return samples;
}

void save_yaml_file(const std::filesystem::path& path, const YAML::Node& config) {
    // Create directory if it doesn't exist
    std::filesystem::create_directories(path.parent_path());
    
    std::ofstream file(path.string());
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + path.string());
    }
    
    file << config;
    file.close();
}

} // namespace ur_admittance_controller

// ============================================================================
// Main Function
// ============================================================================

int main(int argc, char** argv) {
    using namespace ur_admittance_controller;
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("wrench_calibration_node");
    
    try {
        RCLCPP_INFO(node->get_logger(), "Starting wrench calibration...");
        
        // Read current position
        auto current_joints = read_current_joints(node);
        
        // Generate calibration poses
        auto poses = generate_calibration_poses(current_joints);
        RCLCPP_INFO(node->get_logger(), "Generated %zu calibration poses", poses.size());
        
        // Collect samples
        auto samples = collect_calibration_samples(node, poses);
        
        // Compute calibration
        auto result = compute_calibration(samples);
        RCLCPP_INFO(node->get_logger(), "Calibration computed - Tool mass: %.3f kg", 
                   result.tool_mass);
        
        // Build and save configuration
        auto config = build_yaml_config(result);
        auto config_path = get_calibration_config_path();
        save_yaml_file(config_path, config);
        
        RCLCPP_INFO(node->get_logger(), "Calibration saved to: %s", 
                   config_path.c_str());
        RCLCPP_INFO(node->get_logger(), "Calibration completed successfully");
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Calibration failed: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}