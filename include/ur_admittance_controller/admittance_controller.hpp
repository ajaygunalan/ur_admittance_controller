#ifndef UR_ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_
#define UR_ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include "controller_interface/chainable_controller_interface.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "kinematics_interface/kinematics_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/twist.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/chain.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainiksolvervel_pinv.hpp"
#include "kdl/chainjnttojacsolver.hpp"

#include "ur_admittance_controller/visibility_control.h"
#include "ur_admittance_controller_parameters.hpp"

namespace ur_admittance_controller
{

class AdmittanceController : public controller_interface::ChainableControllerInterface
{
public:
  UR_ADMITTANCE_CONTROLLER_PUBLIC
  AdmittanceController();

  UR_ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  UR_ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  UR_ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  UR_ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  UR_ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  UR_ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  UR_ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  UR_ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  UR_ADMITTANCE_CONTROLLER_PUBLIC
  bool on_set_chained_mode(bool chained_mode) override;

  UR_ADMITTANCE_CONTROLLER_PUBLIC
  bool get_chained_mode() const;

private:
  // Type aliases
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
  using GoalResponseCallback = std::function<void(std::shared_ptr<GoalHandleFJT>)>;
  using FeedbackCallback = std::function<void(std::shared_ptr<GoalHandleFJT>, const std::shared_ptr<const FollowJointTrajectory::Feedback>)>;
  using ResultCallback = std::function<void(const GoalHandleFJT::WrappedResult&)>;

  // Parameters
  std::shared_ptr<ur_admittance_controller::ParamListener> param_listener_;
  ur_admittance_controller::Params params_;

  // Kinematics
  std::unique_ptr<kinematics_interface::KinematicsInterface> kinematics_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cart_vel_pub_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_subscriber_;
  realtime_tools::RealtimeBuffer<geometry_msgs::msg::WrenchStamped> wrench_buffer_;

  // Action client for trajectory control
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
  std::shared_future<GoalHandleFJT::SharedPtr> goal_future_;
  FollowJointTrajectory::Goal last_goal_msg_;

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // State variables
  Eigen::Matrix<double, 6, 1> wrench_external_;
  Eigen::Matrix<double, 6, 1> pose_error_;
  Eigen::Matrix<double, 6, 1> velocity_error_;
  Eigen::Matrix<double, 6, 1> desired_acceleration_;
  Eigen::Matrix<double, 6, 1> desired_velocity_;
  Eigen::Matrix<double, 6, 1> desired_pose_;
  Eigen::Matrix<double, 6, 1> cart_vel_cmd_;
  KDL::JntArray joint_positions_;
  KDL::JntArray joint_velocities_;
  KDL::JntArray joint_velocities_cmd_;

  // Matrix parameters
  Eigen::Matrix<double, 6, 6> mass_matrix_;
  Eigen::Matrix<double, 6, 6> damping_matrix_;
  Eigen::Matrix<double, 6, 6> stiffness_matrix_;

  // Trajectory control
  bool new_velocity_command_ = false;
  rclcpp::Time last_twist_time_;
  int retry_count_ = 0;
  bool retry_pending_ = false;
  rclcpp::Time retry_time_;
  bool kdl_initialized_ = false;
  bool chained_mode_ = false;

  // Utility functions
  void calculateAdmittance(const rclcpp::Duration & period);
  void wrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  void sendTrajectory();
  bool loadKinematics();
  bool waitForTransforms();
  void retryTrajectory(const std::string& reason);
  
  // Action client callbacks
  void goalResponseCallback(GoalHandleFJT::SharedPtr goal_handle);
  void feedbackCallback(
    GoalHandleFJT::SharedPtr goal_handle,
    const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback);
  void resultCallback(const GoalHandleFJT::WrappedResult& result);
};

}  // namespace ur_admittance_controller

#endif  // UR_ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_
