#ifndef UR_ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_
#define UR_ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_

// Standard library includes
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
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/handle.hpp"

// Kinematics includes
#include "kinematics_interface/kinematics_interface.hpp"
#include "pluginlib/class_loader.hpp"

// ROS2 includes
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"  // For real-time safe publishers
#include "realtime_tools/realtime_buffer.hpp"    // For real-time safe parameter updates

// TF2 includes
#include "tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_eigen/tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// URDF includes
#include "urdf/model.h"

// Eigen includes
#include <Eigen/Dense>

// Message includes
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
// trajectory_msgs include removed - no longer needed in this file

// Generated parameter includes
#include <ur_admittance_controller/ur_admittance_controller_parameters.hpp>

// Service includes
#include "std_srvs/srv/trigger.hpp"

// Include types definition
#include "admittance_types.hpp"

namespace ur_admittance_controller
{

class AdmittanceController : public controller_interface::ChainableControllerInterface,
                           public std::enable_shared_from_this<AdmittanceController>
{
public:
  AdmittanceController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Get the joint position references (for internal use only)
  std::vector<double> & get_joint_position_references() { return joint_position_references_; }

protected:
  // CHAINABLE: Required override from ChainableControllerInterface
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  // Chainable override required for setting chained mode
  bool on_set_chained_mode(bool /*chained_mode*/) override { return true; }

  // Real-time safe parameter handling
  struct ParameterChange {
    bool mass_changed{false};
    bool stiffness_changed{false};
    bool damping_changed{false};
  };
  
  // Parameters
  std::shared_ptr<ur_admittance_controller::ParamListener> param_listener_;
  ur_admittance_controller::Params params_;
  
  // Real-time safe parameter buffer
  realtime_tools::RealtimeBuffer<ur_admittance_controller::Params> param_buffer_;
  std::atomic<bool> parameter_update_needed_{false};
  std::unique_ptr<ParameterChange> pending_parameter_change_;

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // Kinematics
  std::shared_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>> kinematics_loader_;
  std::optional<pluginlib::UniquePtr<kinematics_interface::KinematicsInterface>> kinematics_;

  // Joint state
  std::vector<double> joint_positions_;
  std::vector<double> joint_position_references_;  // CHAINABLE: References for downstream
  
  // Pre-allocated vectors for real-time safety
  std::vector<double> current_pos_;
  std::vector<double> joint_deltas_;
  std::vector<double> cart_displacement_deltas_;
  std::vector<JointLimits> joint_limits_;

  // Admittance control matrices (using clean typedefs)
  Matrix6d mass_;
  Matrix6d mass_inverse_;  // Pre-computed inverse for performance
  Matrix6d damping_;
  Matrix6d stiffness_;

  // Real-time safe error reporting
  enum class RTErrorType {
    NONE = 0,
    UPDATE_ERROR,
    SENSOR_ERROR,
    TRANSFORM_ERROR,
    CONTROL_ERROR,
    KINEMATICS_ERROR,
    JOINT_LIMITS_ERROR
  };
  std::atomic<RTErrorType> last_rt_error_{RTErrorType::NONE};
  
  // Control state (using clean typedefs)
  Vector6d wrench_ = Vector6d::Zero();  // Current F/T sensor reading
  Vector6d wrench_filtered_ = Vector6d::Zero();  // Filtered F/T reading
  Vector6d pose_error_;
  Vector6d velocity_error_;
  Vector6d desired_accel_;
  Vector6d desired_vel_;
  Vector6d cart_twist_;
  
  // Pose tracking for impedance control
  Eigen::Isometry3d desired_pose_;
  Eigen::Isometry3d current_pose_;

  // Interface caching for RT performance (CRITICAL)
  std::vector<size_t> pos_state_indices_;
  std::vector<long> ft_indices_;

  /**
   * @brief Real-time safe Cartesian velocity publisher
   * 
   * QoS: Best effort, depth=1, volatile durability
   * Threading: Lock-free, non-blocking publish from RT thread
   * Topic: ~/cartesian_velocity_command
   */
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>> rt_cart_vel_pub_;
  
  // Store base publishers to ensure cleanup
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cart_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pose_error_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_holder_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr desired_pose_pub_holder_;
  
  /**
   * @brief Real-time safe joint trajectory publisher  
   * 
   * QoS: Best effort, depth=1, volatile durability
   * Threading: Lock-free, non-blocking publish from RT thread
   * Topic: /scaled_joint_trajectory_controller/joint_trajectory
   * Purpose: Provides joint commands to ScaledJointTrajectoryController
   */
  // Trajectory publisher removed - using reference interfaces is sufficient for controller chaining
  
  /**
   * @brief Real-time safe pose error publisher
   * 
   * QoS: System defaults
   * Threading: Lock-free, non-blocking publish from RT thread
   * Topic: ~/pose_error
   * Purpose: Provides pose error data for monitoring and debugging impedance mode
   */
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>> rt_pose_error_pub_;
  
  // Services and topics for impedance mode control
  std::weak_ptr<rclcpp::Service<std_srvs::srv::Trigger>> reset_pose_service_;
  std::weak_ptr<rclcpp::Service<std_srvs::srv::Trigger>> move_to_pose_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_pose_service_holder_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr move_to_pose_service_holder_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr set_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr desired_pose_pub_;
  
  // State for safe startup
  bool executing_trajectory_ = false;
  geometry_msgs::msg::PoseStamped pending_desired_pose_;
  double stiffness_engagement_factor_ = 1.0;
  bool stiffness_recently_changed_ = false;
  
  // Safe startup parameters
  SafeStartupParams safe_startup_params_;
  
  // Service and subscription callbacks
  void handle_reset_pose(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
  void handle_set_desired_pose(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    
  void handle_move_to_pose(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

private:
  // Real-time safe logger
  rclcpp::Logger rt_logger_;
  
  // RT-safe logging infrastructure
  mutable RTLogger rt_log_buffer_;
  void processRTLogs();  // Process RT logs in non-RT context
  
  // RT-safe logging methods
  void rtLogDebug(const std::string& message) const {
    rt_log_buffer_.push(LogLevel::DEBUG, message);
  }
  void rtLogInfo(const std::string& message) const {
    rt_log_buffer_.push(LogLevel::INFO, message);
  }
  void rtLogWarn(const std::string& message) const {
    rt_log_buffer_.push(LogLevel::WARN, message);
  }
  void rtLogError(const std::string& message) const {
    rt_log_buffer_.push(LogLevel::ERROR, message);
  }

  // Transform caches
  TransformCache ft_transform_cache_{};
  TransformCache ee_transform_cache_{};
  
  // Non-RT transform update method (called from update loop outside RT context)
  void updateTransformCaches();
  std::atomic<bool> transform_update_needed_{false};

  // Helper methods
  void cacheInterfaceIndices();
  void publishCartesianVelocity();
  [[nodiscard]] bool loadKinematics();
  [[nodiscard]] bool waitForTransforms();
  [[nodiscard]] Vector6d computePoseError();  // Compute pose error for impedance control
  
  // Helper methods for cleaner control loop
  // Real-time safe parameter handling (split into RT and non-RT functions)
  void checkParameterUpdates();  // RT-safe - only reads from buffer
  void prepareParameterUpdate();  // non-RT - does parameter checking and buffer writing
  
  // Parameter validation methods
  bool validateParameters(const ur_admittance_controller::Params& params) const;
  bool validateMassParameters(const std::array<double, 6>& mass) const;
  bool validateStiffnessParameters(const std::array<double, 6>& stiffness) const;
  bool validateDampingParameters(const std::array<double, 6>& damping_ratio) const;
  bool validateVelocityLimits(double max_linear, double max_angular) const;
  void logParameterValidationError(const std::string& parameter_name, const std::string& reason) const;
  
  // Control matrix updates - non-RT context with params and logging control
  void updateMassMatrix(const ur_admittance_controller::Params& params, bool log_changes);
  void updateStiffnessMatrix(const ur_admittance_controller::Params& params, bool log_changes);
  void updateDampingMatrix(const ur_admittance_controller::Params& params, bool log_changes);
  
  // Legacy methods - still needed for backward compatibility
  void updateMassMatrix();
  void updateStiffnessMatrix();
  void updateDampingMatrix();
  
  // Real-time safe error reporting
  void reportRTError(RTErrorType error_type) {
    // Non-blocking error reporting - just sets an atomic flag
    last_rt_error_.store(error_type);
  }
  void processNonRTErrors(); // Called in non-RT context to handle errors
  
  bool updateSensorData();
  bool updateTransforms(); // Real-time safe - only uses cached transforms
  bool checkDeadband();
  
  bool computeAdmittanceControl(const rclcpp::Duration& period, Vector6d& cmd_vel_out);
  bool updateStiffnessEngagement(const rclcpp::Duration& period);
  bool publishPoseError();
  bool applyCartesianVelocityLimits();
  bool handleDriftReset();
  
  bool convertToJointSpace(const Vector6d& cart_vel, const rclcpp::Duration& period);
  bool applyJointLimits(const rclcpp::Duration& period);
  void updateReferenceInterfaces();
  void publishMonitoringData();
  controller_interface::return_type safeStop();
  
  // Joint limits utilities  
  [[nodiscard]] bool loadJointLimitsFromURDF(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node,
    const std::vector<std::string> & joint_names,
    std::vector<JointLimits> & limits);
};

}  // namespace ur_admittance_controller

#endif  // UR_ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_