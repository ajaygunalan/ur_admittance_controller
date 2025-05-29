/**
 * @file admittance_controller.hpp
 * @brief Main controller class for UR robot admittance control
 *
 * This file defines the AdmittanceController class which implements
 * 6-DOF admittance control for Universal Robots manipulators. The controller
 * transforms force/torque sensor readings into compliant motion using a
 * mass-spring-damper model.
 *
 * @author UR Robotics Team
 * @date 2024
 */

#ifndef UR_ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_
#define UR_ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_

#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

// ROS2 Control includes
#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/handle.hpp"

// Kinematics includes
#include "kinematics_interface/kinematics_interface.hpp"
#include "pluginlib/class_loader.hpp"

// ROS2 core includes
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_buffer.hpp"

// Transform includes
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// URDF parsing
#include "urdf/model.h"

// Math library
#include <Eigen/Dense>

// Message types
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// Auto-generated parameters
#include <ur_admittance_controller/ur_admittance_controller_parameters.hpp>

// Service types
#include "std_srvs/srv/trigger.hpp"

// Local includes
#include "admittance_types.hpp"

namespace ur_admittance_controller
{

/**
 * @brief Main admittance controller class for Universal Robots
 *
 * This controller implements 6-DOF Cartesian admittance control, transforming
 * force/torque sensor measurements into compliant robot motion. It supports:
 * - Pure admittance mode (zero stiffness)
 * - Impedance mode (non-zero stiffness)
 * - Smooth transitions between modes
 * - Safe startup with gradual stiffness engagement
 * - Real-time safe operation with lock-free logging
 *
 * The admittance control law is:
 * M*a + D*v + K*x = F_external
 *
 * where M is virtual mass, D is damping, K is stiffness, and F is measured force
 */
class AdmittanceController : public controller_interface::ChainableControllerInterface,
                           public std::enable_shared_from_this<AdmittanceController>
{
public:
  /** @brief Default constructor */
  AdmittanceController();

  /** @brief Configure command interfaces (joint position commands) */
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  /** @brief Configure state interfaces (joint positions and F/T sensor) */
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  /** @brief Initialize controller and load parameters */
  controller_interface::CallbackReturn on_init() override;

  /** @brief Configure controller resources and validate parameters */
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  /** @brief Activate controller and start real-time operation */
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /** @brief Deactivate controller and stop real-time operation */
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /** @brief Clean up controller resources */
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  /** @brief Main real-time update function for reference tracking */
  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /** @brief Update and write commands (calls update_reference_from_subscribers) */
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /** @brief Get reference to joint position command vector */
  std::vector<double> & get_joint_position_references() { return joint_position_references_; }

protected:
  /** @brief Export reference interfaces for chained controllers */
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  /** @brief Handle chained mode changes (always returns true) */
  bool on_set_chained_mode(bool /*chained_mode*/) override { return true; }

  // ============================================================================
  // Parameter Management
  // ============================================================================
  
  /** @brief Track which parameters have changed */
  struct ParameterChange {
    bool mass_changed{false};
    bool stiffness_changed{false};
    bool damping_changed{false};
  };
  
  std::shared_ptr<ur_admittance_controller::ParamListener> param_listener_;  ///< Parameter listener
  ur_admittance_controller::Params params_;                                  ///< Current parameters
  
  realtime_tools::RealtimeBuffer<ur_admittance_controller::Params> param_buffer_;  ///< RT-safe param buffer
  std::atomic<bool> parameter_update_needed_{false};                              ///< Flag for param updates
  std::unique_ptr<ParameterChange> pending_parameter_change_;                      ///< Pending changes

  // ============================================================================
  // Transform Management
  // ============================================================================
  
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;              ///< TF2 buffer for transforms
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_; ///< TF2 listener

  // ============================================================================
  // Kinematics
  // ============================================================================
  
  std::shared_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>> kinematics_loader_;
  std::optional<pluginlib::UniquePtr<kinematics_interface::KinematicsInterface>> kinematics_;

  // ============================================================================
  // Joint Space Variables
  // ============================================================================
  
  std::vector<double> joint_positions_;          ///< Current joint positions
  std::vector<double> joint_position_references_; ///< Commanded joint positions
  
  std::vector<double> current_pos_;              ///< Working copy of joint positions
  std::vector<double> joint_deltas_;             ///< Joint position increments
  std::vector<double> cart_displacement_deltas_; ///< Cartesian displacement increments
  std::vector<JointLimits> joint_limits_;        ///< Joint physical limits

  // ============================================================================
  // Admittance Control Matrices
  // ============================================================================
  
  Matrix6d mass_;         ///< Virtual mass matrix (diagonal)
  Matrix6d mass_inverse_; ///< Precomputed mass inverse
  Matrix6d damping_;      ///< Damping matrix (diagonal)
  Matrix6d stiffness_;    ///< Stiffness matrix (diagonal)

  // ============================================================================
  // Error Handling
  // ============================================================================
  
  /** @brief Types of real-time errors */
  enum class RTErrorType {
    NONE = 0,              ///< No error
    UPDATE_ERROR,          ///< General update error
    SENSOR_ERROR,          ///< Force/torque sensor error
    TRANSFORM_ERROR,       ///< Transform lookup error
    CONTROL_ERROR,         ///< Control computation error
    KINEMATICS_ERROR,      ///< Inverse kinematics error
    JOINT_LIMITS_ERROR     ///< Joint limits violation
  };
  std::atomic<RTErrorType> last_rt_error_{RTErrorType::NONE};
  
  // ============================================================================
  // Control Variables
  // ============================================================================
  
  Vector6d F_sensor_base_ = Vector6d::Zero();    ///< F/T sensor reading in base frame
  Vector6d V_base_tip_base_ = Vector6d::Zero();  ///< End-effector velocity in base frame
  Vector6d wrench_filtered_ = Vector6d::Zero();  ///< Filtered wrench measurement
  Vector6d error_tip_base_;                      ///< Pose error in tip frame
  Vector6d velocity_error_;                      ///< Velocity error (unused)
  Vector6d desired_accel_;                       ///< Desired acceleration from admittance
  Vector6d desired_vel_;                         ///< Integrated desired velocity
  
  Eigen::Isometry3d X_base_tip_desired_;         ///< Desired end-effector pose
  Eigen::Isometry3d X_base_tip_current_;         ///< Current end-effector pose

  // ============================================================================
  // Interface Indices
  // ============================================================================
  
  std::vector<size_t> pos_state_indices_;              ///< Indices for joint position states
  std::vector<long> ft_indices_;                       ///< Indices for F/T sensor states
  
  std::vector<size_t> cmd_interface_to_joint_index_;   ///< Map command interfaces to joints

  // ============================================================================
  // Publishers and Subscribers
  // ============================================================================
  
  /** @brief Real-time safe publishers */
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>> rt_cart_vel_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>> rt_pose_error_pub_;
  
  /** @brief Non-real-time publishers */
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cart_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pose_error_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_holder_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr desired_pose_pub_holder_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr desired_pose_pub_;
  
  /** @brief Services */
  std::weak_ptr<rclcpp::Service<std_srvs::srv::Trigger>> reset_pose_service_;
  std::weak_ptr<rclcpp::Service<std_srvs::srv::Trigger>> move_to_pose_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_pose_service_holder_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr move_to_pose_service_holder_;
  
  /** @brief Subscriptions */
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr set_pose_sub_;
  
  // ============================================================================
  // Control State
  // ============================================================================
  
  bool executing_trajectory_ = false;                      ///< Currently executing trajectory
  geometry_msgs::msg::PoseStamped pending_desired_pose_;   ///< Pending pose command
  double stiffness_engagement_factor_ = 1.0;               ///< Stiffness ramp factor (0-1)
  bool stiffness_recently_changed_ = false;                ///< Flag for stiffness ramping
  
  SafeStartupParams safe_startup_params_;                  ///< Safe startup parameters
  // ============================================================================
  // Service Handlers
  // ============================================================================
  
  /**
   * @brief Handle reset_pose service request
   * @param request Service request (empty)
   * @param response Service response with success status
   */
  void handle_reset_pose(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
  /**
   * @brief Handle set_desired_pose topic callback
   * @param msg Desired pose message
   */
  void handle_set_desired_pose(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    
  /**
   * @brief Handle move_to_pose service request
   * @param request Service request (empty)
   * @param response Service response with success status
   */
  void handle_move_to_pose(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

private:
  // ============================================================================
  // Real-time Logging
  // ============================================================================
  
  rclcpp::Logger rt_logger_;              ///< Logger for real-time messages
  mutable RTLogger rt_log_buffer_;        ///< Lock-free log buffer
  
  /** @brief Process buffered log messages in non-RT context */
  void processRTLogs();
  
  /** @name Real-time safe logging methods
   * These methods can be safely called from real-time contexts
   * @{
   */
  
  /** @brief Log debug message (RT-safe) */
  void rtLogDebug(RTLogType type) const {
    rt_log_buffer_.push(LogLevel::DEBUG, type);
  }
  
  /** @brief Log info message (RT-safe) */
  void rtLogInfo(RTLogType type) const {
    rt_log_buffer_.push(LogLevel::INFO, type);
  }
  
  /** @brief Log warning message (RT-safe) */
  void rtLogWarn(RTLogType type) const {
    rt_log_buffer_.push(LogLevel::WARN, type);
  }
  
  /** @brief Log error message (RT-safe) */
  void rtLogError(RTLogType type) const {
    rt_log_buffer_.push(LogLevel::ERROR, type);
  }
  
  /** @brief Log debug message with parameters (RT-safe) */
  void rtLogDebug(RTLogType type, double param1, double param2 = 0.0, double param3 = 0.0) const {
    rt_log_buffer_.push(LogLevel::DEBUG, type, param1, param2, param3);
  }
  
  /** @brief Log info message with parameters (RT-safe) */
  void rtLogInfo(RTLogType type, double param1, double param2 = 0.0, double param3 = 0.0) const {
    rt_log_buffer_.push(LogLevel::INFO, type, param1, param2, param3);
  }
  
  /** @brief Log warning message with parameters (RT-safe) */
  void rtLogWarn(RTLogType type, double param1, double param2 = 0.0, double param3 = 0.0) const {
    rt_log_buffer_.push(LogLevel::WARN, type, param1, param2, param3);
  }
  
  /** @brief Log error message with parameters (RT-safe) */
  void rtLogError(RTLogType type, double param1, double param2 = 0.0, double param3 = 0.0) const {
    rt_log_buffer_.push(LogLevel::ERROR, type, param1, param2, param3);
  }
  
  /** @brief Log debug literal message (RT-safe) */
  void rtLogDebugLiteral(const char* message) const {
    rt_log_buffer_.push(LogLevel::DEBUG, message);
  }
  
  /** @brief Log info literal message (RT-safe) */
  void rtLogInfoLiteral(const char* message) const {
    rt_log_buffer_.push(LogLevel::INFO, message);
  }
  
  /** @brief Log warning literal message (RT-safe) */
  void rtLogWarnLiteral(const char* message) const {
    rt_log_buffer_.push(LogLevel::WARN, message);
  }
  
  /** @brief Log error literal message (RT-safe) */
  void rtLogErrorLiteral(const char* message) const {
    rt_log_buffer_.push(LogLevel::ERROR, message);
  }
  /** @} */

  // ============================================================================
  // Transform Management
  // ============================================================================
  
  TransformCache transform_base_tip_{};    ///< Cached transform: base -> tip
  TransformCache transform_base_ft_{};     ///< Cached transform: base -> F/T sensor
  
  /** @brief Update all transform caches from TF */
  void updateTransformCaches();
  
  /** @brief Update only end-effector transform */
  void updateEETransformOnly();
  
  std::atomic<bool> transform_update_needed_{false};  ///< Flag for transform updates

  // ============================================================================
  // Initialization and Configuration
  // ============================================================================
  
  /** @brief Cache state/command interface indices for fast access */
  void cacheInterfaceIndices();
  
  /** @brief Load kinematics plugin */
  [[nodiscard]] bool loadKinematics();
  
  /** @brief Wait for required transforms to become available */
  [[nodiscard]] bool waitForTransforms();
  
  /** @brief Load joint limits from URDF */
  [[nodiscard]] bool loadJointLimitsFromURDF(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node,
    const std::vector<std::string> & joint_names,
    std::vector<JointLimits> & limits);

  // ============================================================================
  // Real-time Control Loop
  // ============================================================================
  
  /** @brief Compute pose error between desired and current poses */
  [[nodiscard]] Vector6d computePoseError_tip_base();
  
  /** @brief Update sensor data and apply filtering */
  bool updateSensorData();
  
  /** @brief Update transform caches if needed */
  bool updateTransforms();
  
  /** @brief Update base->tip transform specifically */
  bool updateTransform_base_tip();
  
  /** @brief Update base->F/T sensor transform specifically */
  bool updateTransform_base_ft();
  
  /** @brief Check if forces are within deadband */
  bool checkDeadband();
  
  /** @brief Main admittance control computation (RK4 integration) */
  bool computeAdmittanceControl(const rclcpp::Duration& period, Vector6d& cmd_vel_out);
  
  /** @brief Update stiffness engagement factor for safe ramping */
  bool updateStiffnessEngagement(const rclcpp::Duration& period);
  
  /** @brief Apply velocity limits to Cartesian command */
  bool applyCartesianVelocityLimits();
  
  /** @brief Handle drift reset when velocity is near zero */
  bool handleDriftReset();
  
  /** @brief Convert Cartesian velocity to joint space */
  bool convertToJointSpace(const Vector6d& cart_vel, const rclcpp::Duration& period);
  
  /** @brief Apply joint position and velocity limits */
  bool applyJointLimits(const rclcpp::Duration& period);
  
  /** @brief Update command interfaces with new joint positions */
  void updateReferenceInterfaces();
  
  /** @brief Emergency stop with safe position hold */
  controller_interface::return_type safeStop();

  // ============================================================================
  // Publishing and Monitoring
  // ============================================================================
  
  /** @brief Publish Cartesian velocity for monitoring */
  void publishCartesianVelocity();
  
  /** @brief Publish pose error for monitoring */
  bool publishPoseError();
  
  /** @brief Publish all monitoring data */
  void publishMonitoringData();

  // ============================================================================
  // Parameter Updates
  // ============================================================================
  
  /** @brief Check for parameter updates from ROS */
  void checkParameterUpdates();
  
  /** @brief Prepare parameter update for RT thread */
  void prepareParameterUpdate();
  
  /** @brief Update mass matrix with logging */
  void updateMassMatrix(const ur_admittance_controller::Params& params, bool log_changes);
  
  /** @brief Update stiffness matrix with logging */
  void updateStiffnessMatrix(const ur_admittance_controller::Params& params, bool log_changes);
  
  /** @brief Update damping matrix with logging */
  void updateDampingMatrix(const ur_admittance_controller::Params& params, bool log_changes);
  
  /** @brief Update mass matrix from current params */
  void updateMassMatrix();
  
  /** @brief Update stiffness matrix from current params */
  void updateStiffnessMatrix();
  
  /** @brief Update damping matrix from current params */
  void updateDampingMatrix();

  // ============================================================================
  // Error Handling
  // ============================================================================
  
  /** @brief Report error from RT context */
  void reportRTError(RTErrorType error_type) {
    last_rt_error_.store(error_type);
  }
  
  /** @brief Process errors in non-RT context */
  void processNonRTErrors();
};

}  // namespace ur_admittance_controller

#endif  // UR_ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_
