#ifndef UR_ADMITTANCE_CONTROLLER__ADMITTANCE_TYPES_HPP_
#define UR_ADMITTANCE_CONTROLLER__ADMITTANCE_TYPES_HPP_

#include <chrono>
#include <Eigen/Dense>
#include <rclcpp/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

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

// Safe startup parameters
struct SafeStartupParams {
  double trajectory_duration = 5.0;     // Time to move from home to work position
  double stiffness_ramp_time = 2.0;    // Time to gradually engage stiffness
  double max_position_error = 0.15;    // Maximum safe position error (meters)
  double max_orientation_error = 0.5;  // Maximum safe orientation error (radians)
};

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

}  // namespace ur_admittance_controller

#endif  // UR_ADMITTANCE_CONTROLLER__ADMITTANCE_TYPES_HPP_