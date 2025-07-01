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


// Helper functions removed - inlined into moveToJointPosition for better performance

// Execute robot motion to target joint angles
bool moveToJointPosition(
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr& client,
    rclcpp::Node::SharedPtr node,
    const JointAngles& target_joints,
    const JointNames& joint_names) {
    
    // Build trajectory action goal (inlined from createTrajectoryGoal)
    control_msgs::action::FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = joint_names;
    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions = target_joints;
    goal.trajectory.points[0].time_from_start = rclcpp::Duration::from_seconds(3.0);
    
    auto goal_future = client->async_send_goal(goal);
    
    // Wait for goal acceptance
    auto status = rclcpp::spin_until_future_complete(
        node->get_node_base_interface(), goal_future);
    if (status != rclcpp::FutureReturnCode::SUCCESS) return false;
    
    auto goal_handle = goal_future.get();
    if (!goal_handle) return false;
    
    // Wait for motion completion (inlined from waitForMotionComplete)
    auto result_future = client->async_get_result(goal_handle);
    auto result_status = rclcpp::spin_until_future_complete(
        node->get_node_base_interface(), result_future);
    return result_status == rclcpp::FutureReturnCode::SUCCESS;
}

// Logging and YAML helper functions removed - inlined at call sites

// Save calibration parameters to YAML file
void saveCalibrationToYaml(const GravityCompensationParams& p, const std::string& filename) {
    YAML::Emitter out;
    out << YAML::BeginMap
        << YAML::Key << "tool_mass_kg" << YAML::Value << p.tool_mass_kg;
    
    // Write all vectors directly
    out << YAML::Key << "tool_center_of_mass" << YAML::Value << YAML::Flow << YAML::BeginSeq << p.p_CoM_P.x() << p.p_CoM_P.y() << p.p_CoM_P.z() << YAML::EndSeq;
    out << YAML::Key << "gravity_in_base_frame" << YAML::Value << YAML::Flow << YAML::BeginSeq << p.F_gravity_B.x() << p.F_gravity_B.y() << p.F_gravity_B.z() << YAML::EndSeq;
    out << YAML::Key << "force_bias" << YAML::Value << YAML::Flow << YAML::BeginSeq << p.F_bias_P.x() << p.F_bias_P.y() << p.F_bias_P.z() << YAML::EndSeq;
    out << YAML::Key << "torque_bias" << YAML::Value << YAML::Flow << YAML::BeginSeq << p.T_bias_P.x() << p.T_bias_P.y() << p.T_bias_P.z() << YAML::EndSeq;
    
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
        [this](const JointStateMsg::ConstSharedPtr& msg) { updateJointPositions(msg); });
    
    // Subscribe to raw F/T sensor data - lambda captures latest reading atomically
    wrench_sub_ = create_subscription<WrenchMsg>("/netft/raw_sensor", 10,
        [this](const WrenchMsg::ConstSharedPtr& msg) { latest_wrench_ = *msg; has_wrench_ = true; });
    
    // Ensure trajectory controller is available before proceeding with calibration
    if (!trajectory_client_->wait_for_action_server(Seconds(10)))
        throw std::runtime_error("Trajectory action server not available");
    
    // Pre-allocate joint positions vector to avoid resize() in high-frequency callback
    current_joint_positions_.resize(joint_names_.size());
    
    RCLCPP_INFO(get_logger(), "Calibration ready");
}

// Execute 32-pose calibration sequence
CalibrationResult WrenchCalibrationNode::runCalibration() {
    std::vector<CalibrationSample> samples;
    samples.reserve(CalibrationConstants::TOTAL_SAMPLES);  // Pre-allocate for 320 samples (32 poses × 10 samples each)
    
    // Block until both F/T sensor and joint state data are available
    while (!(has_wrench_ && has_joint_states_) && rclcpp::ok()) {
        rclcpp::spin_some(shared_from_this());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
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
    CalibrationResult result;
    
    // Validate input - expecting all samples (10 per pose)
    if (samples.size() < CalibrationConstants::NUM_POSES * CalibrationConstants::SAMPLES_PER_POSE) {
        result.error_message = "Insufficient calibration samples";
        return result;
    }
    
    // LROM algorithm steps (Yu et al. paper)
    auto [gravity_in_base, rotation_s_to_e] = estimateGravityAndRotation(samples);
    auto force_bias = estimateForceBias(samples, gravity_in_base, rotation_s_to_e);
    auto [com_in_sensor, torque_bias] = estimateCOMAndTorqueBias(samples, force_bias);
    
    // Fill result
    result.params.F_gravity_B = gravity_in_base;
    result.params.R_PP = rotation_s_to_e;
    result.params.F_bias_P = force_bias;
    result.params.p_CoM_P = com_in_sensor;
    result.params.T_bias_P = torque_bias;
    result.params.tool_mass_kg = gravity_in_base.norm() / 9.81;
    result.params.num_poses_collected = samples.size();
    
    // Convert rotation matrix to quaternion [x,y,z,w] format
    Eigen::Quaterniond q(rotation_s_to_e);
    result.params.quaternion_sensor_to_endeffector = {{q.x(), q.y(), q.z(), q.w()}};
    
    // Validate results
    auto [force_rmse, torque_rmse] = computeResiduals(samples, result.params);
    result.force_fit_rmse = force_rmse;
    result.torque_fit_rmse = torque_rmse;
    
    result.success = true;
    
    if (result.success) {
        RCLCPP_INFO(get_logger(), "Success! Mass: %.3fkg, COM: [%.3f,%.3f,%.3f]m",
            result.params.tool_mass_kg, result.params.p_CoM_P.x(), result.params.p_CoM_P.y(), result.params.p_CoM_P.z());
    }
    
    return result;
}

// Extract UR joint positions by name from joint_states topic
void WrenchCalibrationNode::updateJointPositions(const JointStateMsg::ConstSharedPtr& msg) {
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
    
    // Log transform details
    const auto& t = X_PB.translation();
    const auto q = Eigen::Quaterniond(X_PB.rotation());
    RCLCPP_INFO(get_logger(), "Pose of %s w.r.t. %s: pos[%.3f,%.3f,%.3f] quat[%.3f,%.3f,%.3f,%.3f]",
        ee_frame_.c_str(), base_frame_.c_str(),
        t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w());
    
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
    // Wait for joint states to be available
    while (!has_joint_states_.load() && rclcpp::ok()) {
        rclcpp::spin_some(shared_from_this());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
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

// ============================================================================
// CALIBRATION MATH METHODS - LROM Algorithm Implementation
// ============================================================================

// Convert vector to skew-symmetric matrix for cross product operations
ur_admittance_controller::Matrix3d ur_admittance_controller::WrenchCalibrationNode::skew_symmetric(const ur_admittance_controller::Vector3d& v) {
    return (ur_admittance_controller::Matrix3d() << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0).finished();
}

// Step 1: Estimate gravity vector and sensor-to-endeffector rotation
std::pair<ur_admittance_controller::Vector3d, ur_admittance_controller::Matrix3d> ur_admittance_controller::WrenchCalibrationNode::estimateGravityAndRotation(
    const std::vector<ur_admittance_controller::CalibrationSample>& samples) const
{
    const size_t n = samples.size();
    
    // Build constraint matrices
    Eigen::MatrixXd A6 = Eigen::MatrixXd::Zero(3 * n, 6);
    Eigen::MatrixXd A9 = Eigen::MatrixXd::Zero(3 * n, 9);
    
    for (size_t i = 0; i < n; ++i) {
        const auto row = 3 * i;
        const auto& force = samples[i].F_P_P_raw.head<3>();
        
        A6.block<3, 3>(row, 0) = -samples[i].X_PB.rotation();
        A6.block<3, 3>(row, 3) = -ur_admittance_controller::Matrix3d::Identity();
        
        // Build A9 following Yu et al.
        for (int j = 0; j < 3; ++j) {
            A9.block<1, 3>(row + j, 3 * j) = force.transpose();
        }
    }
    
    // Constrained least squares with SO(3) constraint
    const auto I9 = std::sqrt(3.0) * Eigen::MatrixXd::Identity(9, 9);
    const auto A6_inv = (A6.transpose() * A6).inverse();
    const auto H = A9 * I9 - A6 * A6_inv * A6.transpose() * A9 * I9;
    
    // Find minimum eigenvalue eigenvector
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H.transpose() * H, Eigen::ComputeThinU);
    const auto& sigma = svd.singularValues();
    const int min_idx = std::distance(sigma.data(), 
        std::min_element(sigma.data(), sigma.data() + sigma.size()));
    
    // Recover solution
    const auto y_opt = svd.matrixU().col(min_idx);
    const auto x6 = -(A6_inv * A6.transpose() * A9 * I9) * y_opt;
    const auto x9 = I9 * y_opt;
    
    // Extract gravity (ensure downward)
    const ur_admittance_controller::Vector3d gravity = x6(2) < 0 ? x6.head<3>() : ur_admittance_controller::Vector3d(-x6.head<3>());
    
    // Reconstruct and project rotation to SO(3)
    ur_admittance_controller::Matrix3d R;
    R << x9.segment<3>(0).transpose(),
         x9.segment<3>(3).transpose(), 
         x9.segment<3>(6).transpose();
    
    Eigen::JacobiSVD<ur_admittance_controller::Matrix3d> rot_svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    
    return {gravity, rot_svd.matrixU() * rot_svd.matrixV().transpose()};
}

// Step 2: Estimate constant force bias in sensor frame
ur_admittance_controller::Vector3d ur_admittance_controller::WrenchCalibrationNode::estimateForceBias(
    const std::vector<ur_admittance_controller::CalibrationSample>& samples,
    const ur_admittance_controller::Vector3d& gravity_in_base,
    const ur_admittance_controller::Matrix3d& rotation_s_to_e) const
{
    ur_admittance_controller::Vector3d force_sum = ur_admittance_controller::Vector3d::Zero();
    ur_admittance_controller::Vector3d gravity_sum = ur_admittance_controller::Vector3d::Zero();
    
    for (const auto& sample : samples) {
        const ur_admittance_controller::Vector3d force = sample.F_P_P_raw.head<3>();
        const ur_admittance_controller::Matrix3d R_PB = sample.X_PB.rotation();
        // Transform gravity to sensor frame for each sample's orientation
        const ur_admittance_controller::Vector3d gravity_in_sensor = rotation_s_to_e * R_PB * gravity_in_base;
        
        force_sum += force;
        gravity_sum += gravity_in_sensor;
    }
    
    const size_t n = samples.size();
    return (force_sum - gravity_sum) / static_cast<double>(n);
}

// Step 3: Estimate tool center of mass and torque bias
std::pair<ur_admittance_controller::Vector3d, ur_admittance_controller::Vector3d> ur_admittance_controller::WrenchCalibrationNode::estimateCOMAndTorqueBias(
    const std::vector<ur_admittance_controller::CalibrationSample>& samples,
    const ur_admittance_controller::Vector3d& force_bias) const
{
    const size_t n = samples.size();
    
    // Linear system: torque = COM × force + bias
    // Matrix form: [F×  I₃] × [p_CoM; T_bias] = T_measured
    Eigen::MatrixXd C(3 * n, 6);
    Eigen::VectorXd b(3 * n);
    
    for (size_t i = 0; i < n; ++i) {
        const auto& sample = samples[i];
        const ur_admittance_controller::Vector3d torque = sample.F_P_P_raw.tail<3>();
        const ur_admittance_controller::Vector3d force  = sample.F_P_P_raw.head<3>();
        
        // Critical: Negate to get reaction force at sensor (Newton's 3rd law)
        const ur_admittance_controller::Vector3d compensated_force = -(force - force_bias);
        
        const size_t row_idx = 3 * i;
        
        C.block<3, 3>(row_idx, 0) = skew_symmetric(compensated_force);  // p_CoM cross product matrix
        C.block<3, 3>(row_idx, 3) = ur_admittance_controller::Matrix3d::Identity();               // torque bias terms
        b.segment<3>(row_idx)      = torque;                            // measured torques
    }
    
    // Solve normal equations: x = (C'C)⁻¹C'b
    const Eigen::VectorXd solution = (C.transpose() * C).inverse() * C.transpose() * b;
    
    const ur_admittance_controller::Vector3d com_in_sensor = solution.head<3>();  // First 3: center of mass
    const ur_admittance_controller::Vector3d torque_bias   = solution.tail<3>();  // Last 3: torque bias
    
    return {com_in_sensor, torque_bias};
}

// Validate calibration by computing residual errors
std::pair<double, double> ur_admittance_controller::WrenchCalibrationNode::computeResiduals(
    const std::vector<ur_admittance_controller::CalibrationSample>& samples,
    const ur_admittance_controller::GravityCompensationParams& params) const
{
    double force_error_sum = 0.0;
    double torque_error_sum = 0.0;
    
    for (const auto& sample : samples) {
        // Direct compensation math (same as GravityCompensator::compensate but without class overhead)
        const ur_admittance_controller::Matrix3d R_EB = sample.X_PB.rotation();
        const ur_admittance_controller::Vector3d f_grav_s = params.R_PP * R_EB * params.F_gravity_B;
        const ur_admittance_controller::Vector3d gravity_torque = params.p_CoM_P.cross(f_grav_s);
        
        ur_admittance_controller::Wrench compensated;
        compensated.head<3>() = sample.F_P_P_raw.head<3>() - f_grav_s - params.F_bias_P;
        compensated.tail<3>() = sample.F_P_P_raw.tail<3>() - gravity_torque - params.T_bias_P;
        
        force_error_sum += compensated.head<3>().squaredNorm();
        torque_error_sum += compensated.tail<3>().squaredNorm();
    }
    
    const size_t n = samples.size();
    const double force_rmse = std::sqrt(force_error_sum / n);
    const double torque_rmse = std::sqrt(torque_error_sum / n);
    
    return {force_rmse, torque_rmse};
}