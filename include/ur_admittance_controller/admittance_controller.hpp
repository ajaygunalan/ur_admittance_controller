/**
 * @file admittance_controller.hpp
 * @brief Admittance controller for Universal Robots
 *
 * This controller reads wrench data from a force/torque sensor and converts it to
 * motion commands using the admittance control law. It handles reference frame transformations,
 * generates smooth trajectories, and sends them to the joint trajectory controller.
 *
 * Data flow: F/T Sensor → Admittance Calculation → IK → Trajectory Generator → Robot
 */

#ifndef UR_ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_
#define UR_ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_

// Standard libraries
#include <memory>
#include <string>
#include <vector>

// Eigen for math operations
#include <Eigen/Dense>

// ROS2 Control and messages
#include "controller_interface/chainable_controller_interface.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

// ROS2 tools and interfaces
#include "kinematics_interface/kinematics_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// KDL for kinematics
#include "kdl/jntarray.hpp"
#include "kdl/chain.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainiksolvervel_pinv.hpp"
#include "kdl/chainjnttojacsolver.hpp"

#include "ur_admittance_controller/visibility_control.h"
#include "ur_admittance_controller/ur_admittance_controller_parameters.hpp"

namespace ur_admittance_controller
{

/**
 * @brief Admittance controller for Universal Robots that responds to external forces
 *
 * This controller implements a force-compliant behavior using the admittance control law:
 * M·a + D·v + K·x = F_ext
 *
 * It reads wrench data from a force/torque sensor, calculates the desired motion,
 * and generates smooth trajectories that are sent to the joint trajectory controller.
 * 
 * The controller is implemented as a ROS2 chainable controller, allowing it to be
 * integrated into complex control systems.
 */
class AdmittanceController : public controller_interface::ChainableControllerInterface
{
public:
  /**
   * @brief Constructor for the admittance controller
   */
  UR_ADMITTANCE_CONTROLLER_PUBLIC
  AdmittanceController();

  /**
   * @brief ROS2 Controller Lifecycle Methods
   * @{*/
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
  /** @} */

  /**
   * @brief Interface Configuration Methods
   * @{*/
  UR_ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  UR_ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  /** @} */

  /**
   * @brief Control Loop Methods
   * @{*/
  UR_ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  UR_ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  /** @} */

  /**
   * @brief Chained Controller Methods
   * @{*/
  UR_ADMITTANCE_CONTROLLER_PUBLIC
  bool on_set_chained_mode(bool chained_mode) override;

  UR_ADMITTANCE_CONTROLLER_PUBLIC
  bool get_chained_mode() const;
  /** @} */

private:
  /** @name Type Aliases
   * @{ */
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
  using GoalResponseCallback = std::function<void(std::shared_ptr<GoalHandleFJT>)>;
  using FeedbackCallback = std::function<void(std::shared_ptr<GoalHandleFJT>, 
                         const std::shared_ptr<const FollowJointTrajectory::Feedback>)>;
  using ResultCallback = std::function<void(const GoalHandleFJT::WrappedResult&)>;
  /** @} */

  /** @name Core Controller Methods
   * @{ */
  /**
   * @brief Calculate admittance control output based on external wrench
   * 
   * Implements the admittance control law: M·a + D·v + K·x = F_ext
   * to calculate desired motion in Cartesian space
   * 
   * @param period Control period for numerical integration
   */
  void calculateAdmittance(const rclcpp::Duration & period);
  
  /**
   * @brief Callback for wrench messages
   * 
   * Receives force/torque sensor data and stores it in the realtime buffer
   * 
   * @param msg Wrench message from sensor or simulation
   */
  void wrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  /** @} */
  
  /** @name Trajectory Execution Methods
   * @{ */
  /**
   * @brief Generate and send trajectory to the robot
   * 
   * Converts Cartesian velocity to joint velocities via IK,
   * generates a smooth trajectory, and sends it to the robot
   */
  void sendTrajectory();
  
  /**
   * @brief Retry failed trajectory with delay
   * 
   * @param reason Reason for retry (for logging)
   */
  void retryTrajectory(const std::string& reason);
  /** @} */

  /** @name Utility Methods
   * @{ */
  /**
   * @brief Load and initialize the kinematics solvers
   * 
   * @return true if successful, false otherwise
   */
  bool loadKinematics();
  
  /**
   * @brief Wait for necessary TF transforms to be available
   * 
   * @return true if all transforms are available, false otherwise
   */
  bool waitForTransforms();
  /** @} */

  /** @name ROS Communication
   * @{ */
  // Parameters
  std::shared_ptr<ur_admittance_controller::ParamListener> param_listener_;
  ur_admittance_controller::Params params_;

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
  /** @} */

  /** @name Kinematics
   * @{ */
  // Kinematics interface
  std::unique_ptr<kinematics_interface::KinematicsInterface> kinematics_;
  
  // KDL state
  KDL::JntArray joint_positions_;
  KDL::JntArray joint_velocities_;
  KDL::JntArray joint_velocities_cmd_;

  // Solvers
  // std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  // std::shared_ptr<KDL::ChainIkSolverVel_pinv> ik_solver_;
  // std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  /** @} */

  /** @name Control State
   * @{ */
  // Admittance parameters
  Eigen::Matrix<double, 6, 6> mass_matrix_;      ///< Mass matrix (M)
  Eigen::Matrix<double, 6, 6> damping_matrix_;    ///< Damping matrix (D)
  Eigen::Matrix<double, 6, 6> stiffness_matrix_;  ///< Stiffness matrix (K)
  
  // Dynamic state
  Eigen::Matrix<double, 6, 1> wrench_external_;     ///< External wrench (F_ext)
  Eigen::Matrix<double, 6, 1> pose_error_;          ///< Position error (x)
  Eigen::Matrix<double, 6, 1> velocity_error_;      ///< Velocity error (v)
  Eigen::Matrix<double, 6, 1> desired_acceleration_; ///< Desired acceleration (a)
  Eigen::Matrix<double, 6, 1> desired_velocity_;    ///< Desired velocity
  Eigen::Matrix<double, 6, 1> desired_pose_;        ///< Desired pose
  Eigen::Matrix<double, 6, 1> cart_vel_cmd_;        ///< Cartesian velocity command
  
  // Timing and state
  rclcpp::Time last_twist_time_;  ///< Time of last twist command
  rclcpp::Time retry_time_;       ///< Time for trajectory retry
  bool new_velocity_command_ = false;
  int retry_count_ = 0;
  bool retry_pending_ = false;
  bool kdl_initialized_ = false;
  bool chained_mode_ = false;
  /** @} */
  
  // Action client callbacks
  void goalResponseCallback(GoalHandleFJT::SharedPtr goal_handle);
  void feedbackCallback(
    GoalHandleFJT::SharedPtr goal_handle,
    const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback);
  void resultCallback(const GoalHandleFJT::WrappedResult& result);
};

}  // namespace ur_admittance_controller

#endif  // UR_ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_
