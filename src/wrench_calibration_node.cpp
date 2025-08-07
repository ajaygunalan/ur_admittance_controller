#include "wrench_calibration_node.hpp"

namespace ur_admittance_controller {

using TrajectoryAction = control_msgs::action::FollowJointTrajectory;

// ============================================================================
// Constructor - All I/O Setup
// ============================================================================

WrenchCalibrationNode::WrenchCalibrationNode() : Node("wrench_calibration_node") {
    // Setup wrench subscription
    wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/netft/raw_sensor", 10,
        [this](const geometry_msgs::msg::WrenchStamped::ConstSharedPtr& msg) {
            latest_wrench_ = *msg;
            has_wrench_ = true;
        });
    
    // Setup TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Setup action client
    trajectory_client_ = rclcpp_action::create_client<TrajectoryAction>(
        this, "/scaled_joint_trajectory_controller/follow_joint_trajectory");
    trajectory_client_->wait_for_action_server(TIMEOUT);
    
    // Wait for TF to be available
    rclcpp::sleep_for(std::chrono::seconds(1));
}

// ============================================================================
// I/O Operations (Instance Methods)
// ============================================================================

JointConfiguration WrenchCalibrationNode::read_current_joints() {
    sensor_msgs::msg::JointState joint_msg;
    rclcpp::wait_for_message(joint_msg, shared_from_this(), "/joint_states", TIMEOUT);
    
    JointConfiguration config;
    std::transform(JOINT_NAMES.begin(), 
                   JOINT_NAMES.end(),
                   config.values.begin(),
                   [&joint_msg](const std::string& name) {
                       auto it = std::find(joint_msg.name.begin(), joint_msg.name.end(), name);
                       return joint_msg.position[std::distance(joint_msg.name.begin(), it)];
                   });
    
    return config;
}

std::vector<CalibrationSample> WrenchCalibrationNode::collect_calibration_samples(
    const std::vector<CalibrationPose>& poses) {
    
    // Wait for first wrench message
    while (!has_wrench_ && rclcpp::ok()) {
        rclcpp::spin_some(shared_from_this());
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    
    std::vector<CalibrationSample> samples;
    samples.reserve(poses.size() * SAMPLES_PER_POSE);
    
    for (size_t pose_idx = 0; pose_idx < poses.size(); ++pose_idx) {
        RCLCPP_INFO(get_logger(), "Pose %zu/%zu", pose_idx + 1, poses.size());
        
        if (pose_idx > 0) executeTrajectory(poses[pose_idx]);
        
        for (int sample = 0; sample < SAMPLES_PER_POSE; ++sample) {
            rclcpp::spin_some(shared_from_this());
            
            if (has_wrench_) {
                try {
                    samples.push_back(collectSingleSample(pose_idx));
                } catch (const tf2::TransformException& ex) {
                    RCLCPP_WARN(get_logger(), "Sample %d at pose %zu failed: %s", 
                                sample+1, pose_idx+1, ex.what());
                }
            } else {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
                                      "No wrench data available");
            }
            rclcpp::sleep_for(SAMPLE_DELAY);
        }
        
        if (pose_idx > 0) executeTrajectory(poses[0]);  // Return home
    }
    
    RCLCPP_INFO(get_logger(), "Collected %zu samples", samples.size());
    return samples;
}

void WrenchCalibrationNode::executeTrajectory(const CalibrationPose& target_pose) {
    TrajectoryAction::Goal goal;
    goal.trajectory.joint_names = {JOINT_NAMES.begin(), JOINT_NAMES.end()};
    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions = {target_pose.joints.begin(), target_pose.joints.end()};
    goal.trajectory.points[0].time_from_start = rclcpp::Duration(TRAJECTORY_DURATION);
    
    // Send goal and check acceptance
    auto goal_handle_future = trajectory_client_->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(shared_from_this(), goal_handle_future) != 
        rclcpp::FutureReturnCode::SUCCESS || !goal_handle_future.get()) {
        RCLCPP_ERROR(get_logger(), "Failed to send trajectory goal");
        return;
    }
    
    // Wait for result
    auto result_future = trajectory_client_->async_get_result(goal_handle_future.get());
    if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) != 
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Failed to execute trajectory");
    }
    
    rclcpp::sleep_for(std::chrono::seconds(1));  // Let robot settle
}

CalibrationSample WrenchCalibrationNode::collectSingleSample(size_t pose_index) {
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_->lookupTransform("base_link", "tool0", tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(get_logger(), "Transform error: %s", ex.what());
        throw;
    }
    
    CalibrationSample sample;
    sample.wrench_raw = (Eigen::Matrix<double, 6, 1>() << 
        latest_wrench_.wrench.force.x,  latest_wrench_.wrench.force.y,  latest_wrench_.wrench.force.z,
        latest_wrench_.wrench.torque.x, latest_wrench_.wrench.torque.y, latest_wrench_.wrench.torque.z).finished();
    sample.transform_TB = tf2::transformToEigen(transform);
    sample.pose_index = pose_index;
    
    return sample;
}

void WrenchCalibrationNode::log_and_save_result(const CalibrationResult& result) {
    RCLCPP_INFO(get_logger(), "Calibration complete:");
    RCLCPP_INFO(get_logger(), "  Tool mass: %.3f kg", result.tool_mass);
    RCLCPP_INFO(get_logger(), "  COM: [%.3f, %.3f, %.3f] m", 
        result.center_of_mass.x(), result.center_of_mass.y(), result.center_of_mass.z());
    RCLCPP_INFO(get_logger(), "  Force bias: [%.2f, %.2f, %.2f] N", 
        result.force_bias.x(), result.force_bias.y(), result.force_bias.z());
    RCLCPP_INFO(get_logger(), "  Torque bias: [%.3f, %.3f, %.3f] Nm", 
        result.torque_bias.x(), result.torque_bias.y(), result.torque_bias.z());
    
    save_calibration_result(result);
    
    const char* workspace = std::getenv("ROS_WORKSPACE");
    std::filesystem::path config_path = (workspace ? std::filesystem::path(workspace) : 
                                         std::filesystem::path(std::getenv("HOME")) / "ros2_ws")
                                         / "src" / "ur_admittance_controller" / "config" / "wrench_calibration.yaml";
    
    RCLCPP_INFO(get_logger(), "Saved to %s - Calibration completed successfully", config_path.c_str());
}

// ============================================================================
// Pure Algorithms (Static Methods)
// ============================================================================

CalibrationPose WrenchCalibrationNode::computeCalibrationPose(const JointConfiguration& current, int index) {
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

std::vector<CalibrationPose> WrenchCalibrationNode::generate_calibration_poses(const JointConfiguration& current) {
    std::vector<CalibrationPose> poses(NUM_POSES);
    std::generate_n(poses.begin(), NUM_POSES,
                    [&current, n = 0]() mutable {
                        return computeCalibrationPose(current, n++);
                    });
    return poses;
}

void WrenchCalibrationNode::save_calibration_result(const CalibrationResult& result) {
    YAML::Node config;
    
    // Convert rotation matrix row-by-row for correct ordering
    std::vector<double> rotation_vec;
    rotation_vec.reserve(9);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            rotation_vec.push_back(result.sensor_rotation(i, j));
    
    config["sensor_rotation"] = rotation_vec;
    config["force_bias"] = std::vector<double>{result.force_bias.x(), result.force_bias.y(), result.force_bias.z()};
    config["torque_bias"] = std::vector<double>{result.torque_bias.x(), result.torque_bias.y(), result.torque_bias.z()};
    config["tool_mass"] = result.tool_mass;
    config["center_of_mass"] = std::vector<double>{result.center_of_mass.x(), result.center_of_mass.y(), result.center_of_mass.z()};
    config["gravity_force"] = std::vector<double>{result.gravity_force.x(), result.gravity_force.y(), result.gravity_force.z()};
    
    const char* workspace = std::getenv("ROS_WORKSPACE");
    std::filesystem::path path = (workspace ? std::filesystem::path(workspace) : 
                                  std::filesystem::path(std::getenv("HOME")) / "ros2_ws")
                                  / "src" / "ur_admittance_controller" / "config" / "wrench_calibration.yaml";
    
    std::filesystem::create_directories(path.parent_path());
    std::ofstream file(path);
    file << config;
}



}

int main(int argc, char** argv) {
    using namespace ur_admittance_controller;
    
    rclcpp::init(argc, argv);
    
    // All I/O setup happens in constructor
    auto node = std::make_shared<WrenchCalibrationNode>();
    RCLCPP_INFO(node->get_logger(), "Initializing calibration system...");
    
    // === Main orchestrates the pipeline clearly ===
    
    // Step 1: Read current state
    JointConfiguration current_joints = node->read_current_joints();
    
    // Step 2: Generate calibration poses (pure)
    std::vector<CalibrationPose> poses = WrenchCalibrationNode::generate_calibration_poses(current_joints);
    RCLCPP_INFO(node->get_logger(), "Generated %zu calibration poses", poses.size());
    
    // Step 3: Collect data (I/O intensive)
    RCLCPP_INFO(node->get_logger(), "Started data collection at %zu poses", poses.size());
    std::vector<CalibrationSample> samples = node->collect_calibration_samples(poses);
    
    // Step 4: Process data (pure algorithms)
    RCLCPP_INFO(node->get_logger(), "Processing %zu samples", samples.size());
    
    Eigen::Vector3d gravity = WrenchCalibrationNode::estimateGravitationalForceInBaseFrame(samples);
    
    auto [R_SE, force_bias] = WrenchCalibrationNode::estimateSensorRotationAndForceBias(samples, gravity);
    
    auto [com, torque_bias] = WrenchCalibrationNode::estimateCOMAndTorqueBias(samples, force_bias);
    
    double tool_mass = gravity.norm() / GRAVITY;
    
    // Step 5: Build result
    CalibrationResult result{R_SE, gravity, force_bias, torque_bias, com, tool_mass};
    
    // Step 6: Log results and save
    node->log_and_save_result(result);
    
    rclcpp::shutdown();
    return 0;
}