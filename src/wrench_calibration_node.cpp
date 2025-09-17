#include "wrench_calibration_node.hpp"

namespace ur_admittance_controller {

using TrajectoryAction = control_msgs::action::FollowJointTrajectory;

// Constructor - All I/O Setup


WrenchCalibrationNode::WrenchCalibrationNode() : Node("wrench_calibration_node") {
    // Setup wrench subscription
    wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/netft/raw_sensor", 10,
        std::bind(&WrenchCalibrationNode::OnWrench, this, std::placeholders::_1));
    
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
    std::transform(JOINT_NAMES.begin(), JOINT_NAMES.end(), config.values.begin(),
                   [&joint_msg](const std::string& name) {
                       auto it = std::find(joint_msg.name.begin(), joint_msg.name.end(), name);
                       return it != joint_msg.name.end() 
                           ? joint_msg.position[std::distance(joint_msg.name.begin(), it)]
                           : 0.0;
                   });
    
    return config;
}

std::vector<CalibrationSample> WrenchCalibrationNode::collect_calibration_samples(
    const std::vector<CalibrationPose>& poses) {
    
    // Wait for first wrench message
    while (!latest_wrench_.has_value() && rclcpp::ok()) {
        rclcpp::spin_some(shared_from_this());
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    
    std::vector<CalibrationSample> samples;
    samples.reserve(poses.size() * SAMPLES_PER_POSE);
    
    for (size_t pose_idx = 0; pose_idx < poses.size(); ++pose_idx) {
        RCLCPP_INFO(get_logger(), "Pose %zu/%zu", pose_idx + 1, poses.size());
        
        if (pose_idx > 0) executeTrajectory(poses[pose_idx]);
        
        // Get transform ONCE per pose after robot moves
        Eigen::Isometry3d pose_transform;
        try {
            auto transform = tf_buffer_->lookupTransform("tool0", "base_link", tf2::TimePointZero);
            pose_transform = tf2::transformToEigen(transform);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_ERROR(get_logger(), "Transform error at pose %zu: %s", pose_idx, ex.what());
            continue;
        }
        
        for (int sample = 0; sample < SAMPLES_PER_POSE; ++sample) {
            rclcpp::spin_some(shared_from_this());
            
            if (!latest_wrench_.has_value()) {
                RCLCPP_WARN(get_logger(), "No wrench data available for sample %d at pose %zu", sample+1, pose_idx+1);
                continue;
            }
            
            CalibrationSample cal_sample;
            cal_sample.wrench_raw = wrenchMsgToEigen(latest_wrench_.value().wrench);
            cal_sample.transform_TB = pose_transform;  // Use same transform for all samples of this pose
            cal_sample.pose_index = pose_idx;
            samples.push_back(cal_sample);
            
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
    
    // Send goal
    auto goal_handle_future = trajectory_client_->async_send_goal(goal);
    auto goal_result = rclcpp::spin_until_future_complete(shared_from_this(), goal_handle_future);
    
    if (goal_result != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Trajectory goal timed out");
        return;
    }
    
    if (!goal_handle_future.get()) {
        RCLCPP_ERROR(get_logger(), "Trajectory goal was rejected");
        return;
    }
    
    // Wait for execution
    auto result_future = trajectory_client_->async_get_result(goal_handle_future.get());
    auto exec_result = rclcpp::spin_until_future_complete(shared_from_this(), result_future);
    
    if (exec_result != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Trajectory execution timed out");
    }
    
    rclcpp::sleep_for(std::chrono::seconds(1));  // Let robot settle
}


void WrenchCalibrationNode::log_and_save_result(const CalibrationResult& result) {
    RCLCPP_INFO(get_logger(), "Calibration complete:");
    RCLCPP_INFO(get_logger(), "  Tool mass: %.3f kg", result.tool_mass.value());
    RCLCPP_INFO(get_logger(), "  COM: [%.3f, %.3f, %.3f] m", 
        result.center_of_mass.x(), result.center_of_mass.y(), result.center_of_mass.z());
    RCLCPP_INFO(get_logger(), "  Force bias: [%.2f, %.2f, %.2f] N", 
        result.force_bias.x(), result.force_bias.y(), result.force_bias.z());
    RCLCPP_INFO(get_logger(), "  Torque bias: [%.3f, %.3f, %.3f] Nm", 
        result.torque_bias.x(), result.torque_bias.y(), result.torque_bias.z());
    // Report robot installation angles
    RCLCPP_INFO(get_logger(), "  Robot installation: roll=%.1f° pitch=%.1f°", 
        result.installation_roll * 180.0 / M_PI, result.installation_pitch * 180.0 / M_PI);
    RCLCPP_INFO(get_logger(), "  (Typical floor-mounted: roll≈0°, pitch≈0° or ±180°)");
    
    save_calibration_result(result);
    
    auto config_path = getConfigPath("wrench_calibration.yaml");
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
    config["tool_mass"] = result.tool_mass.value();
    config["center_of_mass"] = std::vector<double>{result.center_of_mass.x(), result.center_of_mass.y(), result.center_of_mass.z()};
    config["gravity_force"] = std::vector<double>{result.gravity_force.x(), result.gravity_force.y(), result.gravity_force.z()};
    config["installation_roll_rad"] = result.installation_roll;
    config["installation_pitch_rad"] = result.installation_pitch;
    config["installation_roll_deg"] = result.installation_roll * 180.0 / M_PI;
    config["installation_pitch_deg"] = result.installation_pitch * 180.0 / M_PI;
    
    auto path = getConfigPath("wrench_calibration.yaml");
    
    std::filesystem::create_directories(path.parent_path());
    std::ofstream file(path);
    file << config;
}

void WrenchCalibrationNode::OnWrench(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr& msg) {
    latest_wrench_ = *msg;
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
    
    // Step 4: Process data (pure algorithms with type-safe units)
    RCLCPP_INFO(node->get_logger(), "Processing %zu samples", samples.size());
    
    // Section 1: LROM - Estimate gravitational force
    Force gravity = WrenchCalibrationNode::estimateGravitationalForceInBaseFrame(samples);
    
    // Section 2: Procrustes - Estimate rotation and force bias
    auto [R_SE, force_bias] = WrenchCalibrationNode::estimateSensorRotationAndForceBias(samples, gravity);
    
    // Section 3: Estimate torque bias and center of mass
    auto [com, torque_bias] = WrenchCalibrationNode::estimateCOMAndTorqueBias(samples, force_bias);
    
    // Section 4: Decompose gravity vector into mass and installation angles
    auto [tool_mass, roll, pitch] = WrenchCalibrationNode::decomposeGravityVector(gravity);
    
    // Step 5: Build result with type-safe units
    CalibrationResult result{R_SE, gravity, force_bias, torque_bias, com, tool_mass, roll, pitch};
    
    // Step 6: Log results and save
    node->log_and_save_result(result);
    
    rclcpp::shutdown();
    return 0;
}
