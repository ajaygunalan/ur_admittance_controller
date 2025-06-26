// F/T sensor calibration node - collects data from 32 robot poses for LROM calibration
#include "wrench_calibration_node.hpp"
#include "calibration_types.hpp"
#include <tf2_eigen/tf2_eigen.hpp>
#include <filesystem>
#include <fstream>
#include <chrono>
#include <thread>
#include <ranges>
#include <cmath>
#include <functional>
#include <rcpputils/scope_exit.hpp>

namespace ur_admittance_controller {

// Constructor - sets up action client and subscribers
WrenchCalibrationNode::WrenchCalibrationNode() : Node("wrench_calibration_node"),
    tf_buffer_(get_clock()), tf_listener_(tf_buffer_),
    base_frame_(declare_parameter("base_frame", "base_link")),
    ee_frame_(declare_parameter("ee_frame", "tool_payload")) {
    
    trajectory_client_ = rclcpp_action::create_client<TrajectoryAction>(
        this, "/scaled_joint_trajectory_controller/follow_joint_trajectory");
    
    joint_state_sub_ = create_subscription<JointStateMsg>("/joint_states", 10, 
        std::bind(&WrenchCalibrationNode::updateJointPositions, this, std::placeholders::_1));
    
    wrench_sub_ = create_subscription<WrenchMsg>("/F_P_P_raw", 10,
        [this](const WrenchMsg::ConstSharedPtr& msg) { latest_wrench_ = *msg; has_wrench_ = true; });
    
    if (!trajectory_client_->wait_for_action_server(Seconds(10)))
        throw std::runtime_error("Trajectory action server not available");
    
    RCLCPP_INFO(get_logger(), "Calibration ready");
}

// Execute 32-pose calibration sequence
CalibrationResult WrenchCalibrationNode::runCalibration() {
    std::vector<CalibrationSample> samples;
    samples.reserve(CalibrationConstants::TOTAL_SAMPLES);
    
    waitFor([this] { return has_wrench_ && has_joint_states_; });
    
    const auto poses = generateCalibrationPoses();
    RCLCPP_INFO(get_logger(), "Starting %zu-pose calibration", poses.size());
    
    for (size_t i = 0; const auto& pose : poses) {
        RCLCPP_INFO(get_logger(), "Pose %zu/%zu", ++i, poses.size());
        
        if (!moveToJointPosition(pose)) continue;
        
        collectSamplesAtCurrentPose(samples, i - 1);  // i-1 because i starts at 1
        moveToJointPosition(poses[0]);  // Always untangle after data collection
    }
    
    auto result = LROMCalibrator().calibrate(samples);
    if (result.success) logCalibrationSuccess(result.params);
    
    return result;
}

// Extract UR joint positions by name from joint_states topic
void WrenchCalibrationNode::updateJointPositions(const JointStateMsg::ConstSharedPtr& msg) {
    current_joint_positions_.resize(joint_names_.size());
    
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
        if (it == msg->name.end()) {
            has_joint_states_ = false;
            return;
        }
        current_joint_positions_[i] = msg->position[std::distance(msg->name.begin(), it)];
    }
    has_joint_states_ = true;
}

// Generic wait function  
void WrenchCalibrationNode::waitFor(std::function<bool()> condition) {
    while (!condition() && rclcpp::ok()) {
        rclcpp::spin_some(shared_from_this());
        std::this_thread::sleep_for(Milliseconds(100));
    }
}

// Collect 10 samples at 10Hz (1 second of data)
void WrenchCalibrationNode::collectSamplesAtCurrentPose(std::vector<CalibrationSample>& samples, size_t pose_idx) {
    // Get current pose transform (P→B: Payload to Base)
    const auto X_PB = tf2::transformToEigen(
        tf_buffer_.lookupTransform(ee_frame_, base_frame_, tf2::TimePointZero));
    
    logTransform(X_PB);
    
    // Collect wrench data at 10Hz
    Wrench F_P_P_raw_avg = Wrench::Zero();
    for (size_t i = 0; i < CalibrationConstants::SAMPLES_PER_POSE; ++i) {
        auto wrench = extractWrench(latest_wrench_);
        F_P_P_raw_avg += wrench;
        samples.push_back(CalibrationSample{wrench, X_PB, pose_idx});
        
        std::this_thread::sleep_for(Milliseconds(100));
        rclcpp::spin_some(shared_from_this());
    }
    F_P_P_raw_avg /= CalibrationConstants::SAMPLES_PER_POSE;
    
    RCLCPP_INFO(get_logger(), "Raw F/T (payload frame): F[%.2f,%.2f,%.2f]N, T[%.3f,%.3f,%.3f]Nm",
        F_P_P_raw_avg(0), F_P_P_raw_avg(1), F_P_P_raw_avg(2),  // Forces
        F_P_P_raw_avg(3), F_P_P_raw_avg(4), F_P_P_raw_avg(5)); // Torques
}


// Logging helpers
void WrenchCalibrationNode::logCalibrationSuccess(const GravityCompensationParams& params) {
    RCLCPP_INFO(get_logger(), "Success! Mass: %.3fkg, COM: [%.3f,%.3f,%.3f]m",
        params.tool_mass_kg, params.p_CoM_P.x(), params.p_CoM_P.y(), params.p_CoM_P.z());
}

void WrenchCalibrationNode::logTransform(const Transform& tf) {
    const auto& t = tf.translation();
    const auto q = Eigen::Quaterniond(tf.rotation());
    RCLCPP_INFO(get_logger(), "Pose of %s w.r.t. %s: pos[%.3f,%.3f,%.3f] quat[%.3f,%.3f,%.3f,%.3f]",
        ee_frame_.c_str(), base_frame_.c_str(),
        t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w());
}

// Execute robot motion to target joint angles
bool WrenchCalibrationNode::moveToJointPosition(const JointAngles& target_joints) {
    auto goal_future = trajectory_client_->async_send_goal(createTrajectoryGoal(target_joints));
    if (!waitForGoalAcceptance(goal_future)) return false;
    
    auto goal_handle = goal_future.get();
    return goal_handle ? waitForMotionComplete(goal_handle) : false;
}

// Build trajectory action goal
WrenchCalibrationNode::TrajectoryAction::Goal WrenchCalibrationNode::createTrajectoryGoal(const JointAngles& target) {
    TrajectoryAction::Goal goal;
    goal.trajectory.joint_names = joint_names_;
    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions = target;
    goal.trajectory.points[0].time_from_start = rclcpp::Duration::from_seconds(3.0);
    return goal;
}


// Generate 32 poses varying only wrist joints (LROM method)
PoseSequence WrenchCalibrationNode::generateCalibrationPoses() {
    waitFor([this] { return has_joint_states_.load(); });
    
    constexpr auto NUM_POSES = CalibrationConstants::NUM_POSES;
    PoseSequence poses(NUM_POSES, current_joint_positions_);
    
    // Generate wrist variations
    for (int i = 1; i < static_cast<int>(NUM_POSES); ++i) {
        const double idx = i - 1.0;
        poses[i][3] = current_joint_positions_[3] + (idx/NUM_POSES) * M_PI/3.0 - M_PI/6.0;
        poses[i][4] = current_joint_positions_[4] + (std::fmod(idx, 8.0)/8.0) * M_PI/3.0 - M_PI/6.0;
        poses[i][5] = current_joint_positions_[5] + idx * M_PI/NUM_POSES - M_PI/2.0;
    }
    return poses;
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

} // namespace ur_admittance_controller

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto cleanup = rcpputils::make_scope_exit([]{ rclcpp::shutdown(); });
    
    auto node = std::make_shared<ur_admittance_controller::WrenchCalibrationNode>();
    auto result = node->runCalibration();
    
    if (!result.success) {
        RCLCPP_ERROR(rclcpp::get_logger("calibration"), "Calibration failed");
        return 1;
    }
    
    const auto config_file = std::filesystem::path(PACKAGE_SOURCE_DIR) / "config" / "wrench_calibration.yaml";
    ur_admittance_controller::saveCalibrationToYaml(result.params, config_file.string());
    RCLCPP_INFO(node->get_logger(), "Calibration saved to %s", config_file.c_str());
    return 0;
}