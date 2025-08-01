#pragma once

#include <atomic>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <rcpputils/scope_exit.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <yaml-cpp/yaml.h>

#include <utilities/types.hpp>
#include <utilities/error.hpp>
#include <utilities/conversions.hpp>
#include <fmt/core.h>

namespace ur_admittance_controller {

class WrenchCalibrationNode : public rclcpp::Node {
public:
    using TrajectoryAction = control_msgs::action::FollowJointTrajectory;
    using TrajectoryClient = rclcpp_action::Client<TrajectoryAction>;
    using JointStateMsg = sensor_msgs::msg::JointState;
    using WrenchMsg = geometry_msgs::msg::WrenchStamped;


    WrenchCalibrationNode();
    Status Initialize();  // Tier 2: Throws on setup failure via .value()
    Status ExecuteCalibrationSequence();
    Status ComputeCalibrationParameters();

private:
    // Data collection functionality
    void UpdateJointPositions(const JointStateMsg::ConstSharedPtr& msg);
    void CollectSamplesAtCurrentPose(std::vector<CalibrationSample>& samples, size_t pose_idx);
    void GenerateCalibrationPoses();  // New: extracted from constructor

    // New member functions (converted from free functions)
    Status MoveToJointPosition(const JointAngles& target_joints);
    Status SaveCalibrationToYaml();

    // Member variables
    TrajectoryClient::SharedPtr trajectory_client_;
    rclcpp::Subscription<JointStateMsg>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<WrenchMsg>::SharedPtr wrench_sub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;


    WrenchMsg latest_wrench_;
    JointAngles current_joint_positions_;
    PoseSequence calibration_poses_;
    std::vector<CalibrationSample> calibration_samples_;
    std::atomic<bool> has_wrench_{false};
    std::atomic<bool> has_joint_states_{false};

    // New member variables for calibration state
    GravityCompensationParams calibration_params_;
    bool calibration_computed_{false};


    inline static const JointNames joint_names_ = CalibrationConstants::UR_JOINT_NAMES;
    std::string robot_base_frame_;
    std::string robot_tool_frame_;
};

} // namespace ur_admittance_controller
