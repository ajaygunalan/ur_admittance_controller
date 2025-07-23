#pragma once

// System headers (C++ standard library)
#include <atomic>

// Third-party headers (ROS2)
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Local headers
#include <ur_admittance_controller/utilities/types.hpp>
#include <ur_admittance_controller/utilities/error.hpp>

namespace ur_admittance_controller {

class WrenchCalibrationNode : public rclcpp::Node {
public:
    using TrajectoryAction = control_msgs::action::FollowJointTrajectory;
    using TrajectoryClient = rclcpp_action::Client<TrajectoryAction>;
    using JointStateMsg = sensor_msgs::msg::JointState;
    using WrenchMsg = geometry_msgs::msg::WrenchStamped;
    
    
    WrenchCalibrationNode();
    Status initialize();  // Tier 2: Throws on setup failure via .value()
    Status executeCalibrationSequence();
    Status computeCalibrationParameters();
    
private:
    // Data collection functionality
    void updateJointPositions(const JointStateMsg::ConstSharedPtr& msg);
    void collectSamplesAtCurrentPose(std::vector<CalibrationSample>& samples, size_t pose_idx);
    void generateCalibrationPoses();  // New: extracted from constructor
    
    // Calibration math methods (implemented in wrench_compensation.cpp)
    std::pair<double, double> computeResiduals(
        const std::vector<CalibrationSample>& samples,
        const GravityCompensationParams& params) const;
    
    // New member functions (converted from free functions)
    Status moveToJointPosition(const JointAngles& target_joints);
    Status saveCalibrationToYaml();
    
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
    std::string base_frame_;
    std::string ee_frame_;
};

} // namespace ur_admittance_controller