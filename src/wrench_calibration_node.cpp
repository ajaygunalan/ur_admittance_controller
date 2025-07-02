// F/T sensor calibration node - collects data from 32 robot poses for LROM calibration

// System headers (C++ standard library)
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <thread>

// Third-party headers (ROS2, external libraries)
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <rcpputils/scope_exit.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <yaml-cpp/yaml.h>

// Local headers
#include "calibration_types.hpp"
#include "wrench_calibration_node.hpp"

namespace ur_admittance_controller {

// =============================================================================
// CONSTRUCTOR AND INITIALIZATION
// =============================================================================

WrenchCalibrationNode::WrenchCalibrationNode() : Node("wrench_calibration_node"),
    tf_buffer_(get_clock()), tf_listener_(tf_buffer_),
    base_frame_(declare_parameter("base_frame", "base_link")),
    ee_frame_(declare_parameter("ee_frame", "tool0")) {

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
bool WrenchCalibrationNode::initialize() {
    RCLCPP_INFO(get_logger(), "Initializing calibration system...");
    
    // Wait for robot controller with timeout
    if (!trajectory_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(get_logger(), "Trajectory action server not available within 10 seconds");
        return false;
    }
    RCLCPP_INFO(get_logger(), "Trajectory action server connected");
    
    // Wait for joint state data with timeout 
    sensor_msgs::msg::JointState joint_msg;
    if (!rclcpp::wait_for_message(joint_msg, shared_from_this(), "/joint_states", std::chrono::seconds(10))) {
        RCLCPP_ERROR(get_logger(), "Joint states not available within 10 seconds");
        return false;
    }
    RCLCPP_INFO(get_logger(), "Joint states received");
    
    // Wait for F/T sensor data with timeout 
    geometry_msgs::msg::WrenchStamped wrench_msg;
    if (!rclcpp::wait_for_message(wrench_msg, shared_from_this(), "/netft/raw_sensor", std::chrono::seconds(10))) {
        RCLCPP_ERROR(get_logger(), "F/T sensor data not available within 10 seconds");
        return false;
    }
    RCLCPP_INFO(get_logger(), "F/T sensor data received");
    
    // Generate calibration poses using current joint positions
    generateCalibrationPoses();
    
    RCLCPP_INFO(get_logger(), "Calibration system ready for data collection with %zu poses generated", calibration_poses_.size());
    return true;
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
            has_joint_states_ = false;  // Missing joint - mark as invalid
            return;
        }
        // Extract position using iterator distance to get correct index
        current_joint_positions_[i] = msg->position[std::distance(msg->name.begin(), it)];
    }
    has_joint_states_ = true;  // All joints found - mark as valid
}

// =============================================================================
// STEP 1: DATA COLLECTION FUNCTIONS
// =============================================================================

// Execute robot movement sequence and collect F/T sensor data
bool WrenchCalibrationNode::executeCalibrationSequence() {
    // Clear any existing samples
    calibration_samples_.clear();
    
    RCLCPP_INFO(get_logger(), "Starting %zu-pose calibration data collection", calibration_poses_.size());
    
    // Execute calibration sequence across all generated poses
    for (size_t i = 0; i < calibration_poses_.size(); ++i) {
        const auto& pose = calibration_poses_[i];
        RCLCPP_INFO(get_logger(), "Pose %zu/%zu", i + 1, calibration_poses_.size());
        
        // Move to calibration pose - return false if movement fails
        if (!moveToJointPosition(pose)) {
            RCLCPP_ERROR(get_logger(), "Failed to move to pose %zu", i + 1);
            return false;
        }
        
        collectSamplesAtCurrentPose(calibration_samples_, i);  // Collect 10 samples at 10Hz
        moveToJointPosition(calibration_poses_[0]);  // Return to home pose to prevent cable tangling
    }
    
    RCLCPP_INFO(get_logger(), "Data collection completed with %zu samples", calibration_samples_.size());
    return true;
}

// Execute robot motion to target joint angles
bool WrenchCalibrationNode::moveToJointPosition(const JointAngles& target_joints) {
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
    if (status != rclcpp::FutureReturnCode::SUCCESS) return false;
    
    auto goal_handle = goal_future.get();
    if (!goal_handle) return false;
    
    // Wait for motion completion
    auto result_future = trajectory_client_->async_get_result(goal_handle);
    auto result_status = rclcpp::spin_until_future_complete(
        get_node_base_interface(), result_future);
    return result_status == rclcpp::FutureReturnCode::SUCCESS;
}

// Collect 10 samples at 10Hz (1 second of data)
void WrenchCalibrationNode::collectSamplesAtCurrentPose(std::vector<CalibrationSample>& samples, size_t pose_idx) {
    // Get current pose transform (P→B: Payload to Base) - critical for LROM algorithm
    const auto X_PB = tf2::transformToEigen(
        tf_buffer_.lookupTransform(base_frame_, ee_frame_, tf2::TimePointZero));
    
    // Log transform details
    const auto& t = X_PB.translation();
    const auto q = Eigen::Quaterniond(X_PB.rotation());
    RCLCPP_INFO(get_logger(), "Pose of %s w.r.t. %s: pos[%.3f,%.3f,%.3f] quat[%.3f,%.3f,%.3f,%.3f]",
        ee_frame_.c_str(), base_frame_.c_str(),
        t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w());
    
    // Collect wrench data at 10Hz for statistical averaging using timer-based approach
    samples_collected_ = 0;
    collection_complete_ = false;
    Wrench raw_sensor_avg = Wrench::Zero();
    
    // Create timer for precise 10Hz sampling
    sample_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        [this, &samples, &raw_sensor_avg, X_PB, pose_idx]() {
            if (samples_collected_ < CalibrationConstants::SAMPLES_PER_POSE) {
                // Convert ROS WrenchStamped message to Eigen 6D vector for calibration math
                Wrench wrench;
                wrench << latest_wrench_.wrench.force.x, latest_wrench_.wrench.force.y, latest_wrench_.wrench.force.z,    // Forces [N]
                          latest_wrench_.wrench.torque.x, latest_wrench_.wrench.torque.y, latest_wrench_.wrench.torque.z;  // Torques [Nm]
                raw_sensor_avg += wrench;  // Accumulate for averaging
                samples.push_back(CalibrationSample{wrench, X_PB, pose_idx});  // Store individual sample
                samples_collected_++;
            } else {
                sample_timer_->cancel();  // Stop timer
                collection_complete_ = true;
            }
        }
    );
    
    // Wait for collection to complete
    while (!collection_complete_ && rclcpp::ok()) {
        rclcpp::spin_some(shared_from_this());
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    raw_sensor_avg /= CalibrationConstants::SAMPLES_PER_POSE;  // Calculate mean
    
    // Log averaged F/T readings for this pose in sensor coordinate frame
    RCLCPP_INFO(get_logger(), "Raw F/T (sensor frame): F[%.2f,%.2f,%.2f]N, T[%.3f,%.3f,%.3f]Nm",
        raw_sensor_avg(0), raw_sensor_avg(1), raw_sensor_avg(2),  // Forces X,Y,Z
        raw_sensor_avg(3), raw_sensor_avg(4), raw_sensor_avg(5)); // Torques X,Y,Z
}

// =============================================================================
// STEP 2: COMPUTATION FUNCTIONS
// =============================================================================

// Process collected data and compute calibration parameters with YAML saving
bool WrenchCalibrationNode::computeCalibrationParameters() {
    // Validate input - expecting all samples (10 per pose)
    if (calibration_samples_.size() < CalibrationConstants::NUM_POSES * CalibrationConstants::SAMPLES_PER_POSE) {
        RCLCPP_ERROR(get_logger(), "Insufficient calibration samples: %zu (expected %zu)", 
                     calibration_samples_.size(), CalibrationConstants::NUM_POSES * CalibrationConstants::SAMPLES_PER_POSE);
        return false;
    }
    
    RCLCPP_INFO(get_logger(), "Processing %zu calibration samples", calibration_samples_.size());
    
    // LROM algorithm steps (Yu et al. paper)
    auto [gravity_in_base, rotation_s_to_e] = estimateGravityAndRotation(calibration_samples_);
    auto force_bias = estimateForceBias(calibration_samples_, gravity_in_base, rotation_s_to_e);
    auto [com_in_sensor, torque_bias] = estimateCOMAndTorqueBias(calibration_samples_, force_bias);
    
    // Robot installation bias estimation (matches old system sequence)
    [[maybe_unused]] auto rot_b_g = estimateRobotInstallationBias(gravity_in_base);
    
    // Fill calibration parameters using member variable
    calibration_params_.F_gravity_B = gravity_in_base;
    calibration_params_.R_PP = rotation_s_to_e;
    calibration_params_.F_bias_P = force_bias;
    calibration_params_.p_CoM_P = com_in_sensor;
    calibration_params_.T_bias_P = torque_bias;
    
    // Convert rotation matrix to quaternion [x,y,z,w] format
    Eigen::Quaterniond q(rotation_s_to_e);
    calibration_params_.quaternion_sensor_to_endeffector = {{q.x(), q.y(), q.z(), q.w()}};
    
    RCLCPP_INFO(get_logger(), "Calibration successful! COM: [%.3f,%.3f,%.3f]m",
                calibration_params_.p_CoM_P.x(), calibration_params_.p_CoM_P.y(), calibration_params_.p_CoM_P.z());
    
    // Mark calibration as computed
    calibration_computed_ = true;
    
    // Save calibration - no parameters needed!
    return saveCalibrationToYaml();
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
        
        // Build A9 matrix - match old system indexing exactly
        // Columns of matrix A_9 are the transpose of the force reading repeated and shifted based on the row index
        for (int i1 = 0; i1 < 3; ++i1) {
            for (int i2 = 0; i2 < 3; ++i2) {
                A9(row + i1, (3 * i1) + i2) = force[i2];
            }
        }
    }
    
    // Constrained least squares with SO(3) constraint
    const auto I9 = std::sqrt(3.0) * Eigen::MatrixXd::Identity(9, 9);
    const auto A6_inv = (A6.transpose() * A6).inverse();
    const auto H = A9 * I9 - A6 * A6_inv * A6.transpose() * A9 * I9;
    
    // Find minimum eigenvalue eigenvector - match old system's manual search
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H.transpose() * H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const auto& sigma = svd.singularValues();
    
    // Manual minimum finding loop (matches old system exactly)
    int min_eigen_val_id = 0;
    for (int i0 = 0; i0 < sigma.rows(); i0++) {
        if (sigma[i0] < sigma[min_eigen_val_id]) {
            min_eigen_val_id = i0;
        }
    }
    
    // Recover solution
    const auto y_opt = svd.matrixU().col(min_eigen_val_id);
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
    // Match old system's approach: average forces and rotations separately, then apply transformation
    ur_admittance_controller::Vector3d force_readings_avg = ur_admittance_controller::Vector3d::Zero();
    ur_admittance_controller::Matrix3d rotation_avg = ur_admittance_controller::Matrix3d::Zero();
    
    // Average of force readings F in sensor frame (Eq. 37 in Yu et al.)
    for (const auto& sample : samples) {
        const ur_admittance_controller::Vector3d force = sample.F_P_P_raw.head<3>();
        force_readings_avg += force;
    }
    force_readings_avg /= static_cast<double>(samples.size());
    
    // Average of base-to-end effector rotation matrices R in Eq. (37) and related text
    for (const auto& sample : samples) {
        const ur_admittance_controller::Matrix3d R_PB = sample.X_PB.rotation();
        rotation_avg += R_PB;
    }
    rotation_avg /= static_cast<double>(samples.size());
    
    // Force bias calculation as in Eq. (39): f_bias_s = force_readings_avg_s - (rot_s_e * eig_e_b_avg_mat * f_grav_b)
    return force_readings_avg - (rotation_s_to_e * rotation_avg * gravity_in_base);
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

// Robot installation bias estimation - matches old system exactly (see Eq. 48 in Yu et al.)
ur_admittance_controller::Matrix3d ur_admittance_controller::WrenchCalibrationNode::estimateRobotInstallationBias(
    const ur_admittance_controller::Vector3d& gravity_in_base) const
{
    // Compute the magnitude of the gravitational force (see Eq. (48))
    [[maybe_unused]] const double f_grav_norm = std::sqrt(
        gravity_in_base(0) * gravity_in_base(0) + 
        gravity_in_base(1) * gravity_in_base(1) + 
        gravity_in_base(2) * gravity_in_base(2));
    
    // Compute the Tait-Bryan angles (see Eq. (48)) from the gravitational
    // frame G to the robot base frame B
    const double tait_bryan_beta = std::atan2(gravity_in_base(0), gravity_in_base(2));
    
    // Potential singularity here if robot base is effectively horizontal (i.e.,
    // no gravitational force along z-axis of robot base frame B). This will
    // likely never occur in practice.
    const double tait_bryan_alpha = std::atan2(-gravity_in_base(1) * std::cos(tait_bryan_beta), gravity_in_base(2));
    
    // Compute the rotation matrix one term at a time (see Eq. (46))
    ur_admittance_controller::Matrix3d rot_b_g = ur_admittance_controller::Matrix3d::Identity();
    
    rot_b_g(0, 0) = std::cos(tait_bryan_beta);
    rot_b_g(0, 1) = std::sin(tait_bryan_alpha) * std::sin(tait_bryan_beta);
    rot_b_g(0, 2) = std::cos(tait_bryan_alpha) * std::sin(tait_bryan_beta);
    rot_b_g(1, 0) = 0.0;
    rot_b_g(1, 1) = std::cos(tait_bryan_alpha);
    rot_b_g(1, 2) = -std::sin(tait_bryan_alpha);
    rot_b_g(2, 0) = -std::sin(tait_bryan_beta);
    rot_b_g(2, 1) = std::sin(tait_bryan_alpha) * std::cos(tait_bryan_beta);
    rot_b_g(2, 2) = std::cos(tait_bryan_alpha) * std::cos(tait_bryan_beta);
    
    return rot_b_g;
}

// Convert vector to skew-symmetric matrix for cross product operations
ur_admittance_controller::Matrix3d ur_admittance_controller::WrenchCalibrationNode::skew_symmetric(const ur_admittance_controller::Vector3d& v) {
    return (ur_admittance_controller::Matrix3d() << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0).finished();
}

// Save calibration parameters to YAML file
bool WrenchCalibrationNode::saveCalibrationToYaml() {
    if (!calibration_computed_) {
        RCLCPP_ERROR(get_logger(), "No calibration data to save");
        return false;
    }
    
    // Generate filename internally
    std::string package_share_dir;
    try {
        package_share_dir = ament_index_cpp::get_package_share_directory("ur_admittance_controller");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Package not found: %s", e.what());
        return false;
    }
    
    const auto config_file = std::filesystem::path(package_share_dir) / "config" / "wrench_calibration.yaml";
    
    // Ensure config directory exists
    std::filesystem::create_directories(config_file.parent_path());
    
    // Use member calibration_params_ directly
    YAML::Emitter out;
    out << YAML::BeginMap;
    
    // Write all vectors directly
    out << YAML::Key << "tool_center_of_mass" << YAML::Value << YAML::Flow 
        << YAML::BeginSeq << calibration_params_.p_CoM_P.x() << calibration_params_.p_CoM_P.y() 
        << calibration_params_.p_CoM_P.z() << YAML::EndSeq;
    out << YAML::Key << "gravity_in_base_frame" << YAML::Value << YAML::Flow 
        << YAML::BeginSeq << calibration_params_.F_gravity_B.x() << calibration_params_.F_gravity_B.y() 
        << calibration_params_.F_gravity_B.z() << YAML::EndSeq;
    out << YAML::Key << "force_bias" << YAML::Value << YAML::Flow 
        << YAML::BeginSeq << calibration_params_.F_bias_P.x() << calibration_params_.F_bias_P.y() 
        << calibration_params_.F_bias_P.z() << YAML::EndSeq;
    out << YAML::Key << "torque_bias" << YAML::Value << YAML::Flow 
        << YAML::BeginSeq << calibration_params_.T_bias_P.x() << calibration_params_.T_bias_P.y() 
        << calibration_params_.T_bias_P.z() << YAML::EndSeq;
    
    // Rotation matrix as nested sequences
    out << YAML::Key << "rotation_sensor_to_endeffector" << YAML::Value << YAML::BeginSeq;
    for (int i = 0; i < 3; ++i) {
        out << YAML::Flow << std::vector<double>{calibration_params_.R_PP(i,0), calibration_params_.R_PP(i,1), calibration_params_.R_PP(i,2)};
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
    
    std::ofstream(config_file.string()) << out.c_str();
    RCLCPP_INFO(get_logger(), "Calibration saved to %s", config_file.c_str());
    return true;
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
    if (!node->initialize()) {
        RCLCPP_ERROR(node->get_logger(), "System initialization failed");
        return 1;
    }
    
    // Step 1: Execute robot movement sequence and collect sensor data
    if (!node->executeCalibrationSequence()) {
        RCLCPP_ERROR(node->get_logger(), "Data collection failed");
        return 1;
    }
    
    // Step 2: Process collected data and compute calibration parameters
    if (!node->computeCalibrationParameters()) {
        RCLCPP_ERROR(node->get_logger(), "Calibration computation failed");
        return 1;
    }
    
    RCLCPP_INFO(node->get_logger(), "Calibration completed successfully");
    return 0;  // Success
}