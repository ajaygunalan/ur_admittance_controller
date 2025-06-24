#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "wrench_compensation.hpp"
#include "calibration_types.hpp"
#include <atomic>
#include <functional>

namespace ur_admittance_controller {

class WrenchCalibrationNode : public rclcpp::Node {
public:
    using TrajectoryAction = control_msgs::action::FollowJointTrajectory;
    using TrajectoryClient = rclcpp_action::Client<TrajectoryAction>;
    using JointStateMsg = sensor_msgs::msg::JointState;
    using WrenchMsg = geometry_msgs::msg::WrenchStamped;
    
    WrenchCalibrationNode();
    CalibrationResult runCalibration();
    
private:
    // Core functionality
    void updateJointPositions(const JointStateMsg::ConstSharedPtr& msg);
    void waitFor(std::function<bool()> condition);
    void collectSamplesAtCurrentPose(std::vector<CalibrationSample>& samples, size_t pose_idx);
    bool moveToJointPosition(const JointAngles& target_joints);
    TrajectoryAction::Goal createTrajectoryGoal(const JointAngles& target);
    PoseSequence generateCalibrationPoses();
    
    // Logging
    void logCalibrationSuccess(const GravityCompensationParams& params);
    void logTransform(const Transform& tf);
    void logCalibrationMathematics();
    
    template<typename FutureT>
    bool waitForGoalAcceptance(FutureT& future) {
        auto status = rclcpp::spin_until_future_complete(
            get_node_base_interface(), future);
        return status == rclcpp::FutureReturnCode::SUCCESS;
    }
    
    template<typename GoalHandleT>
    bool waitForMotionComplete(GoalHandleT goal_handle) {
        auto result_future = trajectory_client_->async_get_result(goal_handle);
        auto status = rclcpp::spin_until_future_complete(
            get_node_base_interface(), result_future);
        return status == rclcpp::FutureReturnCode::SUCCESS;
    }
    
    // Member variables
    TrajectoryClient::SharedPtr trajectory_client_;
    rclcpp::Subscription<JointStateMsg>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<WrenchMsg>::SharedPtr wrench_sub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    WrenchMsg latest_wrench_;
    JointAngles current_joint_positions_;
    std::atomic<bool> has_wrench_{false};
    std::atomic<bool> has_joint_states_{false};
    
    inline static const JointNames joint_names_ = CalibrationConstants::UR_JOINT_NAMES;
    std::string base_frame_;
    std::string ee_frame_;
};

} // namespace ur_admittance_controller