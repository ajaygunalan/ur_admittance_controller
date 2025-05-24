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
#include "trajectory_msgs/msg/joint_trajectory.hpp"

// Generated parameter includes
#include <ur_admittance_controller/ur_admittance_controller_parameters.hpp>

namespace ur_admittance_controller
{

// Constants for improved code clarity and maintainability
static constexpr size_t DOF = 6;
static constexpr double DEFAULT_FILTER_COEFF = 0.95;
static constexpr double TRANSFORM_TIMEOUT = 0.1;  // seconds

// Clean Eigen typedefs for better readability
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

struct JointLimits
{
  double min_position;
  double max_position;
  double max_velocity;
  double max_acceleration;
};

class AdmittanceController : public controller_interface::ChainableControllerInterface
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

  // Parameters
  std::shared_ptr<ur_admittance_controller::ParamListener> param_listener_;
  ur_admittance_controller::Params params_;

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

  // Control state (using clean typedefs)
  Vector6d wrench_;
  Vector6d wrench_filtered_;
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
  
  /**
   * @brief Real-time safe joint trajectory publisher  
   * 
   * QoS: Best effort, depth=1, volatile durability
   * Threading: Lock-free, non-blocking publish from RT thread
   * Topic: /scaled_joint_trajectory_controller/joint_trajectory
   * Purpose: Provides joint commands to ScaledJointTrajectoryController
   */
  std::unique_ptr<realtime_tools::RealtimePublisher<trajectory_msgs::msg::JointTrajectory>> rt_trajectory_pub_;

private:
  // Real-time safe logger
  rclcpp::Logger rt_logger_;

  // Transform caches
  struct TransformCache {
    geometry_msgs::msg::TransformStamped transform{};
    Matrix6d adjoint = Matrix6d::Zero();
    rclcpp::Time last_update{};
    bool valid{false};
    
    // C++17 feature: Reset method with default member initializer
    void reset() noexcept {
      transform = geometry_msgs::msg::TransformStamped(); // Properly initialize with constructor
      adjoint = Matrix6d::Zero();
      valid = false;
    }
  };
  
  TransformCache ft_transform_cache_{};
  TransformCache ee_transform_cache_{};

  // Helper methods
  void cacheInterfaceIndices();
  void publishCartesianVelocity();
  [[nodiscard]] bool loadKinematics();
  [[nodiscard]] bool waitForTransforms();
  [[nodiscard]] Vector6d computePoseError();  // Compute pose error for impedance control
  
  // Joint limits utilities  
  [[nodiscard]] bool loadJointLimitsFromURDF(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node,
    const std::vector<std::string> & joint_names,
    std::vector<JointLimits> & limits);
};

}  // namespace ur_admittance_controller

#endif  // UR_ADMITTANCE_CONTROLLER__ADMITTANCE_CONTROLLER_HPP_