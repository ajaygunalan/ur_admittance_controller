// F/T sensor calibration node - collects data from 32 robot poses for LROM calibration
#include "wrench_calibration_node.hpp"
#include "calibration_types.hpp"
#include <tf2_eigen/tf2_eigen.hpp>
#include <filesystem>
#include <chrono>
#include <thread>
#include <ranges>
#include <cmath>
#include <rcpputils/scope_exit.hpp>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <functional>

namespace ur_admittance_controller {

// ============================================================================
// CONSOLIDATED UTILITY FUNCTIONS (previously in separate files)
// ============================================================================

// Generic wait function for conditions
void waitFor(rclcpp::Node::SharedPtr node, std::function<bool()> condition) {
    while (!condition() && rclcpp::ok()) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// Template helper functions for action handling
template<typename FutureT>
bool waitForGoalAcceptance(rclcpp::Node::SharedPtr node, FutureT& future) {
    auto status = rclcpp::spin_until_future_complete(
        node->get_node_base_interface(), future);
    return status == rclcpp::FutureReturnCode::SUCCESS;
}

template<typename GoalHandleT, typename ActionClientT>
bool waitForMotionComplete(rclcpp::Node::SharedPtr node, 
                          GoalHandleT goal_handle,
                          ActionClientT& client) {
    auto result_future = client->async_get_result(goal_handle);
    auto status = rclcpp::spin_until_future_complete(
        node->get_node_base_interface(), result_future);
    return status == rclcpp::FutureReturnCode::SUCCESS;
}

// Build trajectory action goal
control_msgs::action::FollowJointTrajectory::Goal createTrajectoryGoal(
    const JointAngles& target, const JointNames& joint_names) {
    control_msgs::action::FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = joint_names;
    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions = target;
    goal.trajectory.points[0].time_from_start = rclcpp::Duration::from_seconds(3.0);
    return goal;
}

// Execute robot motion to target joint angles
bool moveToJointPosition(
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr& client,
    rclcpp::Node::SharedPtr node,
    const JointAngles& target_joints,
    const JointNames& joint_names) {
    
    auto goal = createTrajectoryGoal(target_joints, joint_names);
    auto goal_future = client->async_send_goal(goal);
    
    if (!waitForGoalAcceptance(node, goal_future)) return false;
    
    auto goal_handle = goal_future.get();
    return goal_handle ? waitForMotionComplete(node, goal_handle, client) : false;
}

// Logging helpers
void logCalibrationSuccess(rclcpp::Logger logger, const GravityCompensationParams& params) {
    RCLCPP_INFO(logger, "Success! Mass: %.3fkg, COM: [%.3f,%.3f,%.3f]m",
        params.tool_mass_kg, params.p_CoM_P.x(), params.p_CoM_P.y(), params.p_CoM_P.z());
}

void logTransform(rclcpp::Logger logger, const Transform& tf, 
                  const std::string& ee_frame, const std::string& base_frame) {
    const auto& t = tf.translation();
    const auto q = Eigen::Quaterniond(tf.rotation());
    RCLCPP_INFO(logger, "Pose of %s w.r.t. %s: pos[%.3f,%.3f,%.3f] quat[%.3f,%.3f,%.3f,%.3f]",
        ee_frame.c_str(), base_frame.c_str(),
        t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w());
}

// YAML helper - write 3D vector
void writeVec3Yaml(YAML::Emitter& out, const char* key, const Vector3d& v) {
    out << YAML::Key << key << YAML::Value << YAML::Flow << YAML::BeginSeq << v.x() << v.y() << v.z() << YAML::EndSeq;
}

// Save calibration parameters to YAML file
void saveCalibrationToYaml(const GravityCompensationParams& p, const std::string& filename) {
    YAML::Emitter out;
    out << YAML::BeginMap
        << YAML::Key << "tool_mass_kg" << YAML::Value << p.tool_mass_kg;
    
    // Write all vectors directly
    writeVec3Yaml(out, "tool_center_of_mass", p.p_CoM_P);
    writeVec3Yaml(out, "gravity_in_base_frame", p.F_gravity_B);
    writeVec3Yaml(out, "force_bias", p.F_bias_P);
    writeVec3Yaml(out, "torque_bias", p.T_bias_P);
    
    // Rotation matrix as nested sequences
    out << YAML::Key << "rotation_sensor_to_endeffector" << YAML::Value << YAML::BeginSeq;
    for (int i = 0; i < 3; ++i) {
        out << YAML::Flow << std::vector<double>{p.R_PP(i,0), p.R_PP(i,1), p.R_PP(i,2)};
    }
    out << YAML::EndSeq;
    
    // Quaternion and metadata
    out << YAML::Key << "quaternion_sensor_to_endeffector" 
        << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (const auto& q : p.quaternion_sensor_to_endeffector) {
        out << q;
    }
    out << YAML::EndSeq
        << YAML::Key << "calibration_date" 
        << YAML::Value << std::chrono::system_clock::to_time_t(std::chrono::system_clock::now())
        << YAML::Key << "num_poses" << YAML::Value << p.num_poses_collected 
        << YAML::EndMap;
    
    std::ofstream(filename) << out.c_str();
}

// ============================================================================
// WRENCH CALIBRATION NODE CLASS IMPLEMENTATION
// ============================================================================

// Constructor - sets up action client and subscribers
WrenchCalibrationNode::WrenchCalibrationNode() : Node("wrench_calibration_node"),
    tf_buffer_(get_clock()), tf_listener_(tf_buffer_),  // Initialize TF system for transform lookups
    base_frame_(declare_parameter("base_frame", "base_link")),      // Robot base reference frame
    ee_frame_(declare_parameter("ee_frame", "tool_payload")) {      // End-effector frame with payload
    
    // Create action client to control robot trajectory execution
    trajectory_client_ = rclcpp_action::create_client<TrajectoryAction>(
        this, "/scaled_joint_trajectory_controller/follow_joint_trajectory");
    
    // Subscribe to joint states to track current robot configuration
    joint_state_sub_ = create_subscription<JointStateMsg>("/joint_states", 10, 
        std::bind(&WrenchCalibrationNode::updateJointPositions, this, std::placeholders::_1));
    
    // Subscribe to raw F/T sensor data - lambda captures latest reading atomically
    wrench_sub_ = create_subscription<WrenchMsg>("/netft/raw_sensor", 10,
        [this](const WrenchMsg::ConstSharedPtr& msg) { latest_wrench_ = *msg; has_wrench_ = true; });
    
    // Ensure trajectory controller is available before proceeding with calibration
    if (!trajectory_client_->wait_for_action_server(Seconds(10)))
        throw std::runtime_error("Trajectory action server not available");
    
    RCLCPP_INFO(get_logger(), "Calibration ready");
}

// Execute 32-pose calibration sequence
CalibrationResult WrenchCalibrationNode::runCalibration() {
    std::vector<CalibrationSample> samples;
    samples.reserve(CalibrationConstants::TOTAL_SAMPLES);  // Pre-allocate for 320 samples (32 poses × 10 samples each)
    
    // Block until both F/T sensor and joint state data are available
    waitFor(shared_from_this(), [this] { return has_wrench_ && has_joint_states_; });
    
    const auto poses = generateCalibrationPoses();  // Generate 32 poses varying wrist joints only
    RCLCPP_INFO(get_logger(), "Starting %zu-pose calibration", poses.size());
    
    // Execute calibration sequence across all generated poses
    for (size_t i = 0; const auto& pose : poses) {
        RCLCPP_INFO(get_logger(), "Pose %zu/%zu", ++i, poses.size());
        
        // Move to calibration pose - skip if movement fails
        if (!moveToJointPosition(trajectory_client_, shared_from_this(), pose, joint_names_)) continue;
        
        collectSamplesAtCurrentPose(samples, i - 1);  // Collect 10 samples at 10Hz (i-1 because i starts at 1)
        moveToJointPosition(trajectory_client_, shared_from_this(), poses[0], joint_names_);  // Return to home pose to prevent cable tangling
    }
    
    // Execute LROM (Limited Robot Orientation Method) calibration algorithm
    auto result = LROMCalibrator().calibrate(samples);
    if (result.success) logCalibrationSuccess(get_logger(), result.params);
    
    return result;
}

// Extract UR joint positions by name from joint_states topic
void WrenchCalibrationNode::updateJointPositions(const JointStateMsg::ConstSharedPtr& msg) {
    current_joint_positions_.resize(joint_names_.size());
    
    // Map joint names to positions - joint_states may have different ordering than expected
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
        if (it == msg->name.end()) {
            has_joint_states_ = false;  // Missing joint - mark as invalid
            return;
        }
        // Extract position using iterator distance to get correct index
        current_joint_positions_[i] = msg->position[std::distance(msg->name.begin(), it)];
    }
    has_joint_states_ = true;  // All joints found - mark as valid
}


// Collect 10 samples at 10Hz (1 second of data)
void WrenchCalibrationNode::collectSamplesAtCurrentPose(std::vector<CalibrationSample>& samples, size_t pose_idx) {
    // Get current pose transform (P→B: Payload to Base) - critical for LROM algorithm
    const auto X_PB = tf2::transformToEigen(
        tf_buffer_.lookupTransform(ee_frame_, base_frame_, tf2::TimePointZero));
    
    logTransform(get_logger(), X_PB, ee_frame_, base_frame_);
    
    // Collect wrench data at 10Hz for statistical averaging
    Wrench raw_sensor_avg = Wrench::Zero();
    for (size_t i = 0; i < CalibrationConstants::SAMPLES_PER_POSE; ++i) {
        // Convert ROS WrenchStamped message to Eigen 6D vector for calibration math
        Wrench wrench;
        wrench << latest_wrench_.wrench.force.x, latest_wrench_.wrench.force.y, latest_wrench_.wrench.force.z,    // Forces [N]
                  latest_wrench_.wrench.torque.x, latest_wrench_.wrench.torque.y, latest_wrench_.wrench.torque.z;  // Torques [Nm]
        raw_sensor_avg += wrench;  // Accumulate for averaging
        samples.push_back(CalibrationSample{wrench, X_PB, pose_idx});  // Store individual sample
        
        std::this_thread::sleep_for(Milliseconds(100));  // 10Hz sampling rate
        rclcpp::spin_some(shared_from_this());  // Process callbacks to get fresh data
    }
    raw_sensor_avg /= CalibrationConstants::SAMPLES_PER_POSE;  // Calculate mean
    
    // Log averaged F/T readings for this pose in sensor coordinate frame
    RCLCPP_INFO(get_logger(), "Raw F/T (sensor frame): F[%.2f,%.2f,%.2f]N, T[%.3f,%.3f,%.3f]Nm",
        raw_sensor_avg(0), raw_sensor_avg(1), raw_sensor_avg(2),  // Forces X,Y,Z
        raw_sensor_avg(3), raw_sensor_avg(4), raw_sensor_avg(5)); // Torques X,Y,Z
}





// Generate 32 poses varying only wrist joints (LROM method)
PoseSequence WrenchCalibrationNode::generateCalibrationPoses() {
    waitFor(shared_from_this(), [this] { return has_joint_states_.load(); });
    
    constexpr auto NUM_POSES = CalibrationConstants::NUM_POSES;
    PoseSequence poses(NUM_POSES, current_joint_positions_);  // Initialize all poses with current position
    
    // Generate wrist variations while keeping arm position fixed (LROM constraint)
    for (int i = 1; i < static_cast<int>(NUM_POSES); ++i) {
        const double idx = i - 1.0;
        // Joint 4 (wrist 1): Linear variation across full range ±30°
        poses[i][3] = current_joint_positions_[3] + (idx/NUM_POSES) * M_PI/3.0 - M_PI/6.0;
        // Joint 5 (wrist 2): Cyclic pattern every 8 poses ±30°
        poses[i][4] = current_joint_positions_[4] + (std::fmod(idx, 8.0)/8.0) * M_PI/3.0 - M_PI/6.0;
        // Joint 6 (wrist 3): Full rotation distributed across poses ±90°
        poses[i][5] = current_joint_positions_[5] + idx * M_PI/NUM_POSES - M_PI/2.0;
    }
    return poses;
}



} 

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto cleanup = rcpputils::make_scope_exit([]{ rclcpp::shutdown(); });  // RAII cleanup for ROS
    
    // Create calibration node and execute full calibration sequence
    auto node = std::make_shared<ur_admittance_controller::WrenchCalibrationNode>();
    auto result = node->runCalibration();  // Blocks until all 32 poses completed
    
    if (!result.success) {
        RCLCPP_ERROR(rclcpp::get_logger("calibration"), "Calibration failed");
        return 1;  // Exit with error code
    }
    
    // Save calibration parameters to YAML config file for controller use
    const auto config_file = std::filesystem::path(PACKAGE_SOURCE_DIR) / "config" / "wrench_calibration.yaml";
    saveCalibrationToYaml(result.params, config_file.string());
    RCLCPP_INFO(node->get_logger(), "Calibration saved to %s", config_file.c_str());
    return 0;  // Success
}