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

#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/handle.hpp"

#include "kinematics_interface/kinematics_interface.hpp"
#include "pluginlib/class_loader.hpp"

#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_buffer.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "urdf/model.h"

#include <Eigen/Dense>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <ur_admittance_controller/ur_admittance_controller_parameters.hpp>

#include "std_srvs/srv/trigger.hpp"

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

  std::vector<double> & get_joint_position_references() { return joint_position_references_; }

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>> rt_cart_vel_pub_;
  
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cart_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pose_error_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_holder_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr desired_pose_pub_holder_;
  
  
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>> rt_pose_error_pub_;
  
  std::weak_ptr<rclcpp::Service<std_srvs::srv::Trigger>> reset_pose_service_;
  std::weak_ptr<rclcpp::Service<std_srvs::srv::Trigger>> move_to_pose_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_pose_service_holder_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr move_to_pose_service_holder_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr set_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr desired_pose_pub_;
  
  bool executing_trajectory_ = false;
  geometry_msgs::msg::PoseStamped pending_desired_pose_;
  double stiffness_engagement_factor_ = 1.0;
  bool stiffness_recently_changed_ = false;
  
  SafeStartupParams safe_startup_params_;
  
  void handle_reset_pose(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
  void handle_set_desired_pose(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    
  void handle_move_to_pose(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

private:
  rclcpp::Logger rt_logger_;
  
  mutable RTLogger rt_log_buffer_;
  void processRTLogs();
  
  void rtLogDebug(RTLogType type) const {
    rt_log_buffer_.push(LogLevel::DEBUG, type);
  }
  void rtLogInfo(RTLogType type) const {
    rt_log_buffer_.push(LogLevel::INFO, type);
  }
  void rtLogWarn(RTLogType type) const {
    rt_log_buffer_.push(LogLevel::WARN, type);
  }
  void rtLogError(RTLogType type) const {
    rt_log_buffer_.push(LogLevel::ERROR, type);
  }
  
  void rtLogDebug(RTLogType type, double param1, double param2 = 0.0, double param3 = 0.0) const {
    rt_log_buffer_.push(LogLevel::DEBUG, type, param1, param2, param3);
  }
  void rtLogInfo(RTLogType type, double param1, double param2 = 0.0, double param3 = 0.0) const {
    rt_log_buffer_.push(LogLevel::INFO, type, param1, param2, param3);
  }
  void rtLogWarn(RTLogType type, double param1, double param2 = 0.0, double param3 = 0.0) const {
    rt_log_buffer_.push(LogLevel::WARN, type, param1, param2, param3);
  }
  void rtLogError(RTLogType type, double param1, double param2 = 0.0, double param3 = 0.0) const {
    rt_log_buffer_.push(LogLevel::ERROR, type, param1, param2, param3);
  }
  
  void rtLogDebugLiteral(const char* message) const {
    rt_log_buffer_.push(LogLevel::DEBUG, message);
  }
  void rtLogInfoLiteral(const char* message) const {
    rt_log_buffer_.push(LogLevel::INFO, message);
  }
  void rtLogWarnLiteral(const char* message) const {
    rt_log_buffer_.push(LogLevel::WARN, message);
  }
  void rtLogErrorLiteral(const char* message) const {
    rt_log_buffer_.push(LogLevel::ERROR, message);
  }

  TransformCache transform_base_tip_{};
  TransformCache transform_base_ft_{};
  
  void updateTransformCaches();
  void updateEETransformOnly();
  std::atomic<bool> transform_update_needed_{false};

  void cacheInterfaceIndices();
  void publishCartesianVelocity();
  [[nodiscard]] bool loadKinematics();
  [[nodiscard]] bool waitForTransforms();
  [[nodiscard]] Vector6d computePoseError_tip_base();
  
  void checkParameterUpdates();
  void prepareParameterUpdate();
  
  
  void updateMassMatrix(const ur_admittance_controller::Params& params, bool log_changes);
  void updateStiffnessMatrix(const ur_admittance_controller::Params& params, bool log_changes);
  void updateDampingMatrix(const ur_admittance_controller::Params& params, bool log_changes);
  
  void updateMassMatrix();
  void updateStiffnessMatrix();
  void updateDampingMatrix();
  
  void reportRTError(RTErrorType error_type) {
    last_rt_error_.store(error_type);
  }
  void processNonRTErrors();
  
  bool updateSensorData();
  bool updateTransforms();
  bool updateTransform_base_tip();
  bool updateTransform_base_ft();
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
  
  [[nodiscard]] bool loadJointLimitsFromURDF(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node,
    const std::vector<std::string> & joint_names,
    std::vector<JointLimits> & limits);
};

}

#endif