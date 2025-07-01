#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
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
    // Data collection functionality
    void updateJointPositions(const JointStateMsg::ConstSharedPtr& msg);
    void collectSamplesAtCurrentPose(std::vector<CalibrationSample>& samples, size_t pose_idx);
    PoseSequence generateCalibrationPoses();
    
    // Calibration math methods (implemented in wrench_compensation.cpp)
    [[nodiscard]] std::pair<Vector3d, Matrix3d> estimateGravityAndRotation(
        const std::vector<CalibrationSample>& samples) const;
    
    [[nodiscard]] Vector3d estimateForceBias(
        const std::vector<CalibrationSample>& samples,
        const Vector3d& gravity_in_base,
        const Matrix3d& rotation_s_to_e) const;
    
    [[nodiscard]] std::pair<Vector3d, Vector3d> estimateCOMAndTorqueBias(
        const std::vector<CalibrationSample>& samples,
        const Vector3d& force_bias) const;
    
    [[nodiscard]] std::pair<double, double> computeResiduals(
        const std::vector<CalibrationSample>& samples,
        const GravityCompensationParams& params) const;
    
    [[nodiscard]] static Matrix3d skew_symmetric(const Vector3d& v);
    
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