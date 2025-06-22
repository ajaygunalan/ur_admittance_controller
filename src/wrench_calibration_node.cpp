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

namespace ur_admittance_controller {

// Constructor - sets up action client and subscribers
WrenchCalibrationNode::WrenchCalibrationNode() : Node("wrench_calibration_node"),
    tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {
    
    base_frame_ = declare_parameter("base_frame", "base_link");
    ee_frame_ = declare_parameter("ee_frame", "tool_payload");
    
    trajectory_client_ = rclcpp_action::create_client<TrajectoryAction>(
        this, "/scaled_joint_trajectory_controller/follow_joint_trajectory");
    
    joint_state_sub_ = create_subscription<JointStateMsg>("/joint_states", 10, 
        [this](const JointStateMsg::ConstSharedPtr& msg) { updateJointPositions(std::const_pointer_cast<JointStateMsg>(msg)); });
    
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
        
        // Always untangle after data collection (matching pulse_force FSM)
        moveToJointPosition(poses[0]);
    }
    
    auto result = LROMCalibrator().calibrate(samples);
    if (result.success) {
        logCalibrationSuccess(result.params);
    }
    
    return result;
}

// Extract UR joint positions by name from joint_states topic
void WrenchCalibrationNode::updateJointPositions(const JointStateMsg::SharedPtr msg) {
    current_joint_positions_.resize(6);
    
    has_joint_states_ = true;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
        if (it != msg->name.end()) {
            current_joint_positions_[i] = msg->position[std::distance(msg->name.begin(), it)];
        } else {
            has_joint_states_ = false;
            break;
        }
    }
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
    // Collect wrench data at 10Hz
    auto collectWrenchData = [this]() {
        std::vector<std::pair<Wrench, double>> data;
        data.reserve(CalibrationConstants::SAMPLES_PER_POSE);
        
        for (size_t i = 0; i < CalibrationConstants::SAMPLES_PER_POSE; ++i) {
            data.emplace_back(extractWrench(latest_wrench_), get_clock()->now().seconds());
            std::this_thread::sleep_for(Milliseconds(100));
            rclcpp::spin_some(shared_from_this());
        }
        return data;
    };
    
    // Get current pose transform (E→B to match pulse_force_estimation)
    auto getCurrentTransform = [this]() {
        return tf2::transformToEigen(
            tf_buffer_.lookupTransform(ee_frame_, base_frame_, tf2::TimePointZero));
    };
    
    const auto wrench_data = collectWrenchData();
    const auto X_EB = getCurrentTransform();
    
    // Log transform
    logTransform(X_EB);
    
    // Calculate and log average wrench
    Wrench F_P_P_raw_avg = Wrench::Zero();
    for (const auto& wd : wrench_data) {
        F_P_P_raw_avg += wd.first;
    }
    F_P_P_raw_avg /= wrench_data.size();
    
    RCLCPP_INFO(get_logger(), "Raw F/T (payload frame): F[%.2f,%.2f,%.2f]N, T[%.3f,%.3f,%.3f]Nm",
        F_P_P_raw_avg(0), F_P_P_raw_avg(1), F_P_P_raw_avg(2),  // Forces
        F_P_P_raw_avg(3), F_P_P_raw_avg(4), F_P_P_raw_avg(5)); // Torques
    
    // Create calibration samples - all 10 samples per pose (matching pulse_force)
    for (const auto& wd : wrench_data) {
        samples.push_back(CalibrationSample{wd.first, X_EB, pose_idx});
    }
}


// Logging helpers
void WrenchCalibrationNode::logCalibrationSuccess(const GravityCompensationParams& params) {
    const auto& com = params.p_CoM_P;
    RCLCPP_INFO(get_logger(), "Success! Mass: %.3fkg, COM: [%.3f,%.3f,%.3f]m",
        params.tool_mass_kg, com.x(), com.y(), com.z());
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
    if (auto goal_future = trajectory_client_->async_send_goal(createTrajectoryGoal(target_joints));
        waitForGoalAcceptance(goal_future)) {
        if (auto goal_handle = goal_future.get()) {
            return waitForMotionComplete(goal_handle);
        }
    }
    return false;
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
        JointAngles pose = current_joint_positions_;
        const double idx = i - 1.0;
        pose[3] = current_joint_positions_[3] + (idx/NUM_POSES) * M_PI/3.0 - M_PI/6.0;
        pose[4] = current_joint_positions_[4] + (std::fmod(idx, 8.0)/8.0) * M_PI/3.0 - M_PI/6.0;
        pose[5] = current_joint_positions_[5] + idx * M_PI/NUM_POSES - M_PI/2.0;
        poses[i] = pose;
    }
    return poses;
}


// Save calibration parameters to YAML file
void saveCalibrationToYaml(const GravityCompensationParams& p, const std::string& filename) {
    YAML::Emitter out;
    out << YAML::BeginMap
        << YAML::Key << "tool_mass_kg" << YAML::Value << p.tool_mass_kg;
    
    // Helper to write vectors
    auto writeVec = [&](const char* key, const Vector3d& v) {
        writeVec3Yaml(out, key, v);
    };
    
    writeVec("tool_center_of_mass", p.p_CoM_P);
    writeVec("gravity_in_base_frame", p.F_gravity_B);
    writeVec("force_bias", p.F_bias_P);
    writeVec("torque_bias", p.T_bias_P);
    
    // Rotation matrix as nested sequences
    out << YAML::Key << "rotation_sensor_to_endeffector" << YAML::Value << YAML::BeginSeq;
    for (int i = 0; i < 3; ++i) {
        out << YAML::Flow << std::vector<double>{
            p.R_PP(i,0),
            p.R_PP(i,1),
            p.R_PP(i,2)
        };
    }
    out << YAML::EndSeq;
    
    // Quaternion and metadata
    out << YAML::Key << "quaternion_sensor_to_endeffector" 
        << YAML::Value << YAML::Flow << YAML::BeginSeq
        << p.quaternion_sensor_to_endeffector[0]
        << p.quaternion_sensor_to_endeffector[1]
        << p.quaternion_sensor_to_endeffector[2]
        << p.quaternion_sensor_to_endeffector[3]
        << YAML::EndSeq
        << YAML::Key << "calibration_date" 
        << YAML::Value << std::chrono::system_clock::to_time_t(std::chrono::system_clock::now())
        << YAML::Key << "num_poses" << YAML::Value << p.num_poses_collected 
        << YAML::EndMap;
    
    std::ofstream(filename) << out.c_str();
}

} // namespace ur_admittance_controller

int main(int argc, char** argv) {
    using namespace ur_admittance_controller;
    
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<WrenchCalibrationNode>();
    auto result = node->runCalibration();
    if (result.success) {
        
        const auto config_file = std::filesystem::path(PACKAGE_SOURCE_DIR) / "config" / "wrench_calibration.yaml";
        saveCalibrationToYaml(result.params, config_file.string());
        RCLCPP_INFO(node->get_logger(), "Calibration saved to %s", config_file.c_str());
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("calibration"), "Calibration failed");
        rclcpp::shutdown();
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}