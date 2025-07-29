// F/T sensor calibration node - collects data from 32 robot poses for LROM calibration

// System headers (C++ standard library)
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <thread>

// Third-party headers (ROS2, external libraries)
#include <rclcpp/wait_for_message.hpp>
#include <rcpputils/scope_exit.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <yaml-cpp/yaml.h>

// Local headers
#include "wrench_calibration_node.hpp"
#include "wrench_calibration_algorithm.hpp"
#include <ur_admittance_controller/utilities/conversions.hpp>
#include <fmt/core.h>

namespace ur_admittance_controller {

// =============================================================================
// CONSTRUCTOR AND INITIALIZATION
// =============================================================================

WrenchCalibrationNode::WrenchCalibrationNode() : Node("wrench_calibration_node"),
    tf_buffer_(get_clock()), tf_listener_(tf_buffer_),
    robot_base_frame(declare_parameter("robot_base_frame", frames::ROBOT_BASE_FRAME)),
    robot_tool_frame(declare_parameter("robot_tool_frame", frames::ROBOT_TOOL_FRAME)) {

    // Creates robot controller client and subscribes to sensor data streams
    trajectory_client_ = rclcpp_action::create_client<TrajectoryAction>(
        this, "/scaled_joint_trajectory_controller/follow_joint_trajectory");
    joint_state_sub_ = create_subscription<JointStateMsg>("/joint_states", 10,
        [this](const JointStateMsg::ConstSharedPtr& msg) { updateJointPositions(msg); });
    wrench_sub_ = create_subscription<WrenchMsg>("/netft/raw_sensor", 10,
        [this](const WrenchMsg::ConstSharedPtr& msg) { latest_wrench_ = *msg; has_wrench_ = true; });

    // Pre-allocate memory and initialize data structures
    current_joint_positions_.resize(joint_names_.size());
    calibration_samples_.reserve(CalibrationConstants::TOTAL_SAMPLES);
    
    RCLCPP_INFO(get_logger(), "Calibration node constructed. Call initialize() to prepare for data collection.");
}

// Initialize system - wait for dependencies and generate poses
Status WrenchCalibrationNode::initialize() {
    RCLCPP_INFO(get_logger(), "Initializing calibration system...");
    
    // Wait for robot controller with timeout
    if (!trajectory_client_->wait_for_action_server(std::chrono::seconds(10))) {
        return tl::unexpected(make_error(ErrorCode::kCommunicationTimeout,
                                       "Trajectory action server not available within 10 seconds"));
    }
    RCLCPP_INFO(get_logger(), "Trajectory action server connected");
    
    // Wait for joint state data with timeout and process it
    sensor_msgs::msg::JointState joint_msg;
    if (!rclcpp::wait_for_message(joint_msg, shared_from_this(), "/joint_states", std::chrono::seconds(10))) {
        return tl::unexpected(make_error(ErrorCode::kCommunicationTimeout,
                                       "Joint states not available within 10 seconds"));
    }
    // Process the received message to populate current_joint_positions_
    auto joint_msg_ptr = std::make_shared<sensor_msgs::msg::JointState>(joint_msg);
    updateJointPositions(joint_msg_ptr);
    RCLCPP_INFO(get_logger(), "Joint states received and processed");
    
    // Wait for F/T sensor data with timeout 
    geometry_msgs::msg::WrenchStamped wrench_msg;
    if (!rclcpp::wait_for_message(wrench_msg, shared_from_this(), "/netft/raw_sensor", std::chrono::seconds(10))) {
        return tl::unexpected(make_error(ErrorCode::kCommunicationTimeout,
                                       "F/T sensor data not available within 10 seconds"));
    }
    RCLCPP_INFO(get_logger(), "F/T sensor data received");
    
    // Generate calibration poses using current joint positions
    generateCalibrationPoses();
    
    RCLCPP_INFO(get_logger(), "Calibration system ready for data collection with %zu poses generated", calibration_poses_.size());
    return {};  // Success
}

// Generate 32 calibration poses 
void WrenchCalibrationNode::generateCalibrationPoses() {
    calibration_poses_.assign(CalibrationConstants::NUM_POSES, current_joint_positions_);
    for (int i = 1; i < CalibrationConstants::NUM_POSES; ++i) {
        const double idx = i - 1.0;
        calibration_poses_[i][3] = current_joint_positions_[3] + (idx/CalibrationConstants::NUM_POSES) * M_PI/3.0 - M_PI/6.0;
        calibration_poses_[i][4] = current_joint_positions_[4] + (std::fmod(idx, 8.0)/8.0) * M_PI/3.0 - M_PI/6.0;
        calibration_poses_[i][5] = current_joint_positions_[5] + idx * M_PI/CalibrationConstants::NUM_POSES - M_PI/2.0;
    }
}

// Extract UR joint positions by name from joint_states topic
void WrenchCalibrationNode::updateJointPositions(const JointStateMsg::ConstSharedPtr& msg) {
    // Map joint names to positions - joint_states may have different ordering than expected
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
        if (it == msg->name.end()) {
            // Missing joint - mark as invalid
            has_joint_states_ = false;
            return;
        }
        // Extract position using iterator distance to get correct index
        current_joint_positions_[i] = msg->position[std::distance(msg->name.begin(), it)];
    }
    // All joints found - mark as valid
    has_joint_states_ = true;
}

// =============================================================================
// STEP 1: DATA COLLECTION FUNCTIONS
// =============================================================================

// Execute robot movement sequence and collect F/T sensor data
Status WrenchCalibrationNode::executeCalibrationSequence() {
    // Clear any existing samples
    calibration_samples_.clear();
    
    RCLCPP_INFO(get_logger(), "Starting %zu-pose calibration data collection", calibration_poses_.size());
    
    // Execute calibration sequence across all generated poses
    for (size_t i = 0; i < calibration_poses_.size(); ++i) {
        const auto& pose = calibration_poses_[i];
        RCLCPP_INFO(get_logger(), "Pose %zu/%zu", i + 1, calibration_poses_.size());
        
        // Move to calibration pose - propagate error if movement fails
        auto move_status = moveToJointPosition(pose);
        if (!move_status) {
            RCLCPP_ERROR(get_logger(), "Failed to move to pose %zu: %s", 
                        i + 1, move_status.error().message.c_str());
            return tl::unexpected(make_error(ErrorCode::kTrajectoryExecutionFailed,
                                           "Failed at pose " + std::to_string(i + 1) + ": " + 
                                           move_status.error().message));
        }
        RCLCPP_INFO(get_logger(), "✓ Reached pose %zu", i + 1);
        
        collectSamplesAtCurrentPose(calibration_samples_, i);  // Collect 10 samples at 10Hz
        
        // Return to home position (best effort - don't fail if this fails)
        auto home_status = moveToJointPosition(calibration_poses_[0]);
        if (!home_status) {
            RCLCPP_WARN(get_logger(), "Failed to return to home position: %s",
                       home_status.error().message.c_str());
        }
    }
    
    RCLCPP_INFO(get_logger(), "Data collection completed with %zu samples", calibration_samples_.size());
    return {};  // Success
}


// Execute robot motion to target joint angles
Status WrenchCalibrationNode::moveToJointPosition(const JointAngles& target_joints) {
    // Build trajectory action goal
    control_msgs::action::FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = joint_names_;
    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions = target_joints;
    goal.trajectory.points[0].time_from_start = rclcpp::Duration::from_seconds(3.0);
    
    auto goal_future = trajectory_client_->async_send_goal(goal);
    
    // Wait for goal acceptance
    auto status = rclcpp::spin_until_future_complete(
        get_node_base_interface(), goal_future);
    if (status != rclcpp::FutureReturnCode::SUCCESS) {
        return tl::unexpected(make_error(ErrorCode::kTimeout,
                                       "Timeout waiting for trajectory goal acceptance"));
    }
    
    auto goal_handle = goal_future.get();
    if (!goal_handle) {
        return tl::unexpected(make_error(ErrorCode::kTrajectoryExecutionFailed,
                                       "Trajectory goal was rejected by controller"));
    }
    
    // Wait for motion completion
    auto result_future = trajectory_client_->async_get_result(goal_handle);
    auto result_status = rclcpp::spin_until_future_complete(
        get_node_base_interface(), result_future);
    if (result_status != rclcpp::FutureReturnCode::SUCCESS) {
        return tl::unexpected(make_error(ErrorCode::kTimeout,
                                       "Timeout waiting for trajectory execution"));
    }
    
    // Check actual result
    auto result = result_future.get();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        return tl::unexpected(make_error(ErrorCode::kTrajectoryExecutionFailed,
                                       "Trajectory execution failed with action result code: " + 
                                       std::to_string(static_cast<int>(result.code))));
    }
    
    if (result.result->error_code != control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL) {
        return tl::unexpected(make_error(ErrorCode::kTrajectoryExecutionFailed,
                                       "Trajectory execution failed with error code: " + 
                                       std::to_string(result.result->error_code)));
    }
    
    return {};  // Success
}

// Collect 10 samples at 10Hz (1 second of data)
void WrenchCalibrationNode::collectSamplesAtCurrentPose(std::vector<CalibrationSample>& samples, size_t pose_idx) {
    // Wait for mechanical settling after robot motion
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Get current pose transform (X_EB: Base pose in End-effector frame) 
    const auto X_EB = tf2::transformToEigen(
        tf_buffer_.lookupTransform(robot_tool_frame, robot_base_frame, tf2::TimePointZero));
    
    // Log transform details
    const auto& t = X_EB.translation();
    const auto q = Eigen::Quaterniond(X_EB.rotation());
    RCLCPP_INFO(get_logger(), "Pose of %s w.r.t. %s: pos[%.3f,%.3f,%.3f] quat[%.3f,%.3f,%.3f,%.3f]",
        robot_base_frame.c_str(), robot_tool_frame.c_str(),
        t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w());
    
    // Collect wrench data at 10Hz for statistical averaging
    Wrench6d raw_sensor_avg = Wrench6d::Zero();
    for (size_t i = 0; i < CalibrationConstants::SAMPLES_PER_POSE; ++i) {
        // Convert ROS WrenchStamped message to Eigen 6D vector for calibration math
        Wrench6d wrench;
        wrench = conversions::fromMsg(latest_wrench_);
        raw_sensor_avg += wrench;  // Accumulate for averaging
        samples.push_back(CalibrationSample{wrench, X_EB, pose_idx});  // Store individual sample
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 10Hz sampling rate
        rclcpp::spin_some(shared_from_this());  // Process callbacks to get fresh data
    }
    raw_sensor_avg /= CalibrationConstants::SAMPLES_PER_POSE;  // Calculate mean
    
    // Log averaged F/T readings for this pose in sensor coordinate frame
    // Drake notation: use .head<3>() for force, .tail<3>() for torque
    Force3d force_avg = raw_sensor_avg.head<3>();
    Torque3d torque_avg = raw_sensor_avg.tail<3>();
    RCLCPP_INFO(get_logger(), "Raw F/T (sensor frame): F[%.2f,%.2f,%.2f]N, T[%.3f,%.3f,%.3f]Nm",
        force_avg.x(), force_avg.y(), force_avg.z(),   // Forces X,Y,Z
        torque_avg.x(), torque_avg.y(), torque_avg.z()); // Torques X,Y,Z
}

// =============================================================================
// STEP 2: COMPUTATION FUNCTIONS
// =============================================================================

// Process collected data and compute calibration parameters with YAML saving
Status WrenchCalibrationNode::computeCalibrationParameters() {
    RCLCPP_INFO(get_logger(), "Processing %zu calibration samples", calibration_samples_.size());
    
    // LROM algorithm steps matching paper structure
    // Section 1: Estimate gravitational force in base frame
    RCLCPP_INFO(get_logger(), "Step 1/4: Estimating gravity force in base frame");
    RCLCPP_INFO(get_logger(), "  Input: %zu force/torque samples from different robot poses", calibration_samples_.size());
    auto f_gravity_B = ur_admittance_controller::estimateGravitationalForceInBaseFrame(calibration_samples_).value();
    RCLCPP_INFO(get_logger(), "  Output: Gravity force F_b = [%.3f, %.3f, %.3f] N (magnitude: %.3f N)", 
                f_gravity_B.x(), f_gravity_B.y(), f_gravity_B.z(), f_gravity_B.norm());
    
    // Section 2: Estimate sensor rotation and force bias using Procrustes
    RCLCPP_INFO(get_logger(), "Step 2/4: Computing sensor-to-endeffector rotation and force bias");
    RCLCPP_INFO(get_logger(), "  Input: Force samples + gravity force F_b");
    auto [R_SE, f_bias_S] = ur_admittance_controller::estimateSensorRotationAndForceBias(calibration_samples_, f_gravity_B).value();
    // Convert rotation to Euler angles for display
    Eigen::Vector3d angles = R_SE.eulerAngles(2, 1, 0); // ZYX convention
    RCLCPP_INFO(get_logger(), "  Output: Force bias = [%.3f, %.3f, %.3f] N", 
                f_bias_S.x(), f_bias_S.y(), f_bias_S.z());
    RCLCPP_INFO(get_logger(), "  Output: Rotation R_SE (euler ZYX) = [%.1f°, %.1f°, %.1f°]",
                angles.z() * 180/M_PI, angles.y() * 180/M_PI, angles.x() * 180/M_PI);
    
    // Section 3: Estimate COM and torque bias
    RCLCPP_INFO(get_logger(), "Step 3/4: Computing tool center of mass and torque bias");
    RCLCPP_INFO(get_logger(), "  Input: Torque samples + force bias from Step 2");
    auto [p_SCoM_S, t_bias_S] = ur_admittance_controller::estimateCOMAndTorqueBias(calibration_samples_, f_bias_S).value();
    RCLCPP_INFO(get_logger(), "  Output: COM position = [%.3f, %.3f, %.3f] m", 
                p_SCoM_S.x(), p_SCoM_S.y(), p_SCoM_S.z());
    RCLCPP_INFO(get_logger(), "  Output: Torque bias = [%.3f, %.3f, %.3f] Nm",
                t_bias_S.x(), t_bias_S.y(), t_bias_S.z());
    
    // Section 4: Robot installation bias estimation
    RCLCPP_INFO(get_logger(), "Step 4/4: Estimating robot installation bias");
    RCLCPP_INFO(get_logger(), "  Input: Gravity force F_b from Step 1");
    [[maybe_unused]] auto rot_b_g = ur_admittance_controller::estimateRobotInstallationBias(f_gravity_B).value();
    // Extract installation angles
    double fbx = f_gravity_B.x();
    double fby = f_gravity_B.y();
    double fbz = f_gravity_B.z();
    double beta = std::atan2(fbx, fbz);
    double alpha = std::atan2(-fby * std::cos(beta), fbz);
    RCLCPP_INFO(get_logger(), "  Output: Installation angles α=%.1f°, β=%.1f° (robot base tilt)",
                alpha * 180/M_PI, beta * 180/M_PI);
    
    // Fill calibration parameters using member variable
    calibration_params_.f_gravity_B = f_gravity_B;
    calibration_params_.R_SE = R_SE;
    calibration_params_.f_bias_S = f_bias_S;
    calibration_params_.p_SCoM_S = p_SCoM_S;
    calibration_params_.t_bias_S = t_bias_S;
    
    // Convert rotation matrix to quaternion [x,y,z,w] format
    Eigen::Quaterniond q(R_SE);
    calibration_params_.quaternion_sensor_to_endeffector = {{q.x(), q.y(), q.z(), q.w()}};
    
    // Final summary
    RCLCPP_INFO(get_logger(), "=== Calibration Complete ===");
    RCLCPP_INFO(get_logger(), "Tool mass: %.3f kg", f_gravity_B.norm() / 9.81);
    RCLCPP_INFO(get_logger(), "COM offset from sensor: [%.3f, %.3f, %.3f] m",
                calibration_params_.p_SCoM_S.x(), calibration_params_.p_SCoM_S.y(), calibration_params_.p_SCoM_S.z());
    
    // Mark calibration as computed
    calibration_computed_ = true;
    
    // Save calibration - no parameters needed!
    return saveCalibrationToYaml();
}


// Save calibration parameters to YAML file
Status WrenchCalibrationNode::saveCalibrationToYaml() {
    if (!calibration_computed_) {
        return tl::unexpected(make_error(ErrorCode::kInvalidConfiguration,
                                       "No calibration data to save - run calibration first"));
    }
    
    // Get workspace from environment or use default
    std::string workspace = std::getenv("ROS_WORKSPACE") ? 
                           std::getenv("ROS_WORKSPACE") : 
                           std::string(std::getenv("HOME")) + "/ros2_ws";
    
    const auto config_file = std::filesystem::path(workspace) / "src" / "ur_admittance_controller" / "config" / "wrench_calibration.yaml";
    
    // Ensure config directory exists
    std::filesystem::create_directories(config_file.parent_path());
    
    // Use member calibration_params_ directly
    YAML::Emitter out;
    out << YAML::BeginMap;
    
    // Write all vectors directly
    out << YAML::Key << "tool_center_of_mass" << YAML::Value << YAML::Flow 
        << YAML::BeginSeq << calibration_params_.p_SCoM_S.x() << calibration_params_.p_SCoM_S.y() 
        << calibration_params_.p_SCoM_S.z() << YAML::EndSeq;
    out << YAML::Key << "gravity_in_base_frame" << YAML::Value << YAML::Flow 
        << YAML::BeginSeq << calibration_params_.f_gravity_B.x() << calibration_params_.f_gravity_B.y() 
        << calibration_params_.f_gravity_B.z() << YAML::EndSeq;
    out << YAML::Key << "force_bias" << YAML::Value << YAML::Flow 
        << YAML::BeginSeq << calibration_params_.f_bias_S.x() << calibration_params_.f_bias_S.y() 
        << calibration_params_.f_bias_S.z() << YAML::EndSeq;
    out << YAML::Key << "torque_bias" << YAML::Value << YAML::Flow 
        << YAML::BeginSeq << calibration_params_.t_bias_S.x() << calibration_params_.t_bias_S.y() 
        << calibration_params_.t_bias_S.z() << YAML::EndSeq;
    
    // Rotation matrix as nested sequences
    out << YAML::Key << "rotation_sensor_to_endeffector" << YAML::Value << YAML::BeginSeq;
    for (int i = 0; i < 3; ++i) {
        out << YAML::Flow << std::vector<double>{calibration_params_.R_SE(i,0), calibration_params_.R_SE(i,1), calibration_params_.R_SE(i,2)};
    }
    out << YAML::EndSeq;
    
    // Quaternion and metadata
    out << YAML::Key << "quaternion_sensor_to_endeffector" 
        << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (const auto& q : calibration_params_.quaternion_sensor_to_endeffector) {
        out << q;
    }
    out << YAML::EndSeq
        << YAML::EndMap;
    
    std::ofstream file(config_file.string());
    if (!file || !(file << out.c_str())) {
        return tl::unexpected(make_error(ErrorCode::kFileNotFound,
                                       fmt::format("Failed to write calibration to {}", config_file.string())));
    }
    
    RCLCPP_INFO(get_logger(), "Calibration saved to %s", config_file.c_str());
    return {};  // Success
}

} 

// =============================================================================
// MAIN FUNCTION
// =============================================================================

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto cleanup = rcpputils::make_scope_exit([]{ rclcpp::shutdown(); });  // RAII cleanup for ROS
    
    // Create calibration node with lightweight constructor
    auto node = std::make_shared<ur_admittance_controller::WrenchCalibrationNode>();
    
    // Initialize system - wait for dependencies and prepare for calibration
    if (auto status = node->initialize(); !status) {
        RCLCPP_ERROR(node->get_logger(), "System initialization failed: %s", status.error().message.c_str());
        return 1;
    }
    
    // Step 1: Execute robot movement sequence and collect sensor data
    auto calibration_status = node->executeCalibrationSequence();
    if (!calibration_status) {
        RCLCPP_ERROR(node->get_logger(), "Data collection failed: %s", 
                    calibration_status.error().message.c_str());
        return 1;
    }
    
    // Step 2: Process collected data and compute calibration parameters
    if (auto status = node->computeCalibrationParameters(); !status) {
        RCLCPP_ERROR(node->get_logger(), "Calibration computation failed: %s", status.error().message.c_str());
        return 1;
    }
    
    RCLCPP_INFO(node->get_logger(), "Calibration completed successfully");
    return 0;  // Success
}