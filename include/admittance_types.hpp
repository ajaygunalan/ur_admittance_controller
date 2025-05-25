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

// Real-time safe transform caches
struct TransformCache {
  // Data accessible by RT thread
  struct Entry {
    geometry_msgs::msg::TransformStamped transform{};
    Matrix6d adjoint = Matrix6d::Zero();
    rclcpp::Time timestamp{};
    std::atomic<bool> valid{false};
  };
  
  // Cache entries for different transform pairs
  Entry cache;
  
  // Non-RT data
  std::string target_frame;
  std::string source_frame;
  rclcpp::Time last_update{};
  
  // Methods
  void reset() noexcept {
    cache.transform = geometry_msgs::msg::TransformStamped();
    cache.adjoint = Matrix6d::Zero();
    cache.valid.store(false);
  }
  
  // Check if the transform is valid for RT thread
  bool isValid() const noexcept {
    return cache.valid.load();
  }
};

}  // namespace ur_admittance_controller

#endif  // UR_ADMITTANCE_CONTROLLER__ADMITTANCE_TYPES_HPP_