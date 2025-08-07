// F/T sensor calibration node - collects data from 32 robot poses for LROM calibration
#include "wrench_calibration_node.hpp"

namespace ur_admittance_controller {

// Frame definitions (missing from header)
namespace frames {
    constexpr const char* ROBOT_BASE_FRAME = "base_link";
    constexpr const char* ROBOT_TOOL_FRAME = "tool0";
}

// No local conversion helpers needed - using from header

WrenchCalibrationNode::WrenchCalibrationNode() : Node("wrench_calibration_node"),
    tf_buffer_(get_clock()), tf_listener_(tf_buffer_),
    robot_base_frame_(declare_parameter("robot_base_frame", frames::ROBOT_BASE_FRAME)),
    robot_tool_frame_(declare_parameter("robot_tool_frame", frames::ROBOT_TOOL_FRAME)) {

    trajectory_client_ = rclcpp_action::create_client<TrajectoryAction>(
        this, "/scaled_joint_trajectory_controller/follow_joint_trajectory");
    joint_state_sub_ = create_subscription<JointStateMsg>("/joint_states", constants::DEFAULT_QUEUE_SIZE,
        [this](const JointStateMsg::ConstSharedPtr& msg) { UpdateJointPositions(msg); });
    wrench_sub_ = create_subscription<WrenchMsg>("/netft/raw_sensor", constants::DEFAULT_QUEUE_SIZE,
        [this](const WrenchMsg::ConstSharedPtr& msg) { latest_wrench_ = *msg; has_wrench_ = true; });

    current_joint_positions_.resize(joint_names_.size());
    calibration_samples_.reserve(CalibrationConstants::TOTAL_SAMPLES);

    RCLCPP_INFO(get_logger(), "Calibration node ready");
}

void WrenchCalibrationNode::Initialize() {
    RCLCPP_INFO(get_logger(), "Initializing calibration system...");

    trajectory_client_->wait_for_action_server(constants::DEFAULT_TIMEOUT);

    sensor_msgs::msg::JointState joint_msg;
    rclcpp::wait_for_message(joint_msg, shared_from_this(), "/joint_states", constants::DEFAULT_TIMEOUT);
    UpdateJointPositions(std::make_shared<sensor_msgs::msg::JointState>(joint_msg));

    geometry_msgs::msg::WrenchStamped wrench_msg;
    rclcpp::wait_for_message(wrench_msg, shared_from_this(), "/netft/raw_sensor", constants::DEFAULT_TIMEOUT);

    GenerateCalibrationPoses();

    RCLCPP_INFO(get_logger(), "Ready with %zu poses", calibration_poses_.size());
}

void WrenchCalibrationNode::GenerateCalibrationPoses() {
    calibration_poses_.assign(CalibrationConstants::NUM_POSES, current_joint_positions_);
    for (int i = 1; i < CalibrationConstants::NUM_POSES; ++i) {
        const double idx = i - constants::CALIBRATION_INDEX_OFFSET;
        calibration_poses_[i][3] = current_joint_positions_[3] +
            (idx/CalibrationConstants::NUM_POSES) * constants::CALIBRATION_ANGLE_LARGE - constants::CALIBRATION_ANGLE_SMALL;
        calibration_poses_[i][4] = current_joint_positions_[4] +
            (std::fmod(idx, constants::CALIBRATION_MODULO_DIVISOR)/constants::CALIBRATION_MODULO_DIVISOR) * 
            constants::CALIBRATION_ANGLE_LARGE - constants::CALIBRATION_ANGLE_SMALL;
        calibration_poses_[i][5] = current_joint_positions_[5] + idx * M_PI/CalibrationConstants::NUM_POSES - M_PI/2.0;
    }
}

void WrenchCalibrationNode::UpdateJointPositions(const JointStateMsg::ConstSharedPtr& msg) {
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


void WrenchCalibrationNode::ExecuteCalibrationSequence() {
    calibration_samples_.clear();

    RCLCPP_INFO(get_logger(), "Starting %zu-pose calibration", calibration_poses_.size());

    for (size_t i = 0; i < calibration_poses_.size(); ++i) {
        RCLCPP_INFO(get_logger(), "Pose %zu/%zu", i + 1, calibration_poses_.size());

        MoveToJointPosition(calibration_poses_[i]);
        CollectSamplesAtCurrentPose(calibration_samples_, i);

        if (i > 0) {
            MoveToJointPosition(calibration_poses_[0]);
        }
    }

    RCLCPP_INFO(get_logger(), "Collected %zu samples", calibration_samples_.size());
}


void WrenchCalibrationNode::MoveToJointPosition(const JointAngles& target_joints) {
    control_msgs::action::FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = joint_names_;
    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions = target_joints;
    goal.trajectory.points[0].time_from_start = rclcpp::Duration::from_seconds(constants::CALIBRATION_TRAJECTORY_DURATION);

    auto goal_future = trajectory_client_->async_send_goal(goal);
    rclcpp::spin_until_future_complete(get_node_base_interface(), goal_future);

    auto goal_handle = goal_future.get();
    auto result_future = trajectory_client_->async_get_result(goal_handle);
    rclcpp::spin_until_future_complete(get_node_base_interface(), result_future);
}

void WrenchCalibrationNode::CollectSamplesAtCurrentPose(std::vector<CalibrationSample>& samples, size_t pose_idx) {
    std::this_thread::sleep_for(constants::SAMPLE_DELAY);

    const auto X_TB = tf2::transformToEigen(
        tf_buffer_.lookupTransform(robot_tool_frame_, robot_base_frame_, tf2::TimePointZero));

    const auto& t = X_TB.translation();
    const auto q = Eigen::Quaterniond(X_TB.rotation());
    logging::LogPose(get_logger(), "Pose:", t, q);

    Wrench6d raw_sensor_avg = Wrench6d::Zero();
    for (size_t i = 0; i < CalibrationConstants::SAMPLES_PER_POSE; ++i) {
        Wrench6d wrench = conversions::FromMsg(latest_wrench_);
        raw_sensor_avg += wrench;
        samples.push_back(CalibrationSample{wrench, X_TB, pose_idx});

        std::this_thread::sleep_for(constants::SAMPLE_DELAY);
        rclcpp::spin_some(shared_from_this());
    }
    raw_sensor_avg /= CalibrationConstants::SAMPLES_PER_POSE;

    logging::LogWrench(get_logger(), "F/T avg:", raw_sensor_avg);
}


void WrenchCalibrationNode::ComputeCalibrationParameters() {
    RCLCPP_INFO(get_logger(), "Processing %zu samples", calibration_samples_.size());

    auto f_gravity_B = ur_admittance_controller::estimateGravitationalForceInBaseFrame(calibration_samples_);
    auto [R_SE, f_bias_S] = ur_admittance_controller::estimateSensorRotationAndForceBias(calibration_samples_, f_gravity_B);
    auto [p_SCoM_S, t_bias_S] = ur_admittance_controller::estimateCOMAndTorqueBias(calibration_samples_, f_bias_S);

    double beta = std::atan2(f_gravity_B.x(), f_gravity_B.z());
    double alpha = std::atan2(-f_gravity_B.y() * std::cos(beta), f_gravity_B.z());

    calibration_params_ = {
        R_SE, f_gravity_B, f_bias_S, t_bias_S, p_SCoM_S
    };

    RCLCPP_INFO(get_logger(), "Calibration complete:");
    RCLCPP_INFO(get_logger(), "  Tool mass: %.3f kg", f_gravity_B.norm() / constants::GRAVITY);
    logging::LogVector3(get_logger(), "  COM:", p_SCoM_S);
    RCLCPP_INFO(get_logger(), "  Installation: α=%.1f°, β=%.1f°", alpha * 180/M_PI, beta * 180/M_PI);

    calibration_computed_ = true;
    SaveCalibrationToYaml();
}


void WrenchCalibrationNode::SaveCalibrationToYaml() {
    if (!calibration_computed_) {
        throw std::runtime_error("No calibration data to save");
    }

    std::string workspace = std::getenv("ROS_WORKSPACE") ?
                           std::getenv("ROS_WORKSPACE") :
                           std::string(std::getenv("HOME")) + "/ros2_ws";

    const auto config_file = std::filesystem::path(workspace) / "src" / "ur_admittance_controller" / "config" / "wrench_calibration.yaml";
    std::filesystem::create_directories(config_file.parent_path());

    YAML::Emitter out;
    out << YAML::BeginMap;

    out << YAML::Key << "tool_center_of_mass" << YAML::Value << YAML::Flow
        << std::vector<double>{calibration_params_.p_CoM_s.x(), calibration_params_.p_CoM_s.y(), calibration_params_.p_CoM_s.z()};
    out << YAML::Key << "gravity_in_base_frame" << YAML::Value << YAML::Flow
        << std::vector<double>{calibration_params_.f_grav_b.x(), calibration_params_.f_grav_b.y(), calibration_params_.f_grav_b.z()};
    out << YAML::Key << "force_bias" << YAML::Value << YAML::Flow
        << std::vector<double>{calibration_params_.f_bias_s.x(), calibration_params_.f_bias_s.y(), calibration_params_.f_bias_s.z()};
    out << YAML::Key << "torque_bias" << YAML::Value << YAML::Flow
        << std::vector<double>{calibration_params_.t_bias_s.x(), calibration_params_.t_bias_s.y(), calibration_params_.t_bias_s.z()};

    out << YAML::Key << "rotation_sensor_to_endeffector" << YAML::Value << YAML::BeginSeq;
    for (int i = 0; i < 3; ++i) {
        out << YAML::Flow << std::vector<double>{calibration_params_.R_SE(i,0), calibration_params_.R_SE(i,1), calibration_params_.R_SE(i,2)};
    }
    out << YAML::EndSeq;
    out << YAML::EndMap;

    std::ofstream file(config_file.string());
    file << out.c_str();

    RCLCPP_INFO(get_logger(), "Saved to %s", config_file.c_str());
}

}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto cleanup = rcpputils::make_scope_exit([]{ rclcpp::shutdown(); });

    auto node = std::make_shared<ur_admittance_controller::WrenchCalibrationNode>();

    node->Initialize();
    node->ExecuteCalibrationSequence();
    node->ComputeCalibrationParameters();

    RCLCPP_INFO(node->get_logger(), "Calibration completed");
    return 0;
}
