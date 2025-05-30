/**
 * @file admittance_types.hpp
 * @brief Type definitions and data structures for the UR admittance controller
 *
 * This file contains custom types, enumerations, and data structures used
 * throughout the admittance controller. It includes transform caching 
 * mechanisms and parameter structures.
 *
 * @author UR Robotics Team
 * @date 2024
 */

#ifndef UR_ADMITTANCE_CONTROLLER__ADMITTANCE_TYPES_HPP_
#define UR_ADMITTANCE_CONTROLLER__ADMITTANCE_TYPES_HPP_

#include <chrono>
#include <Eigen/Dense>
#include <rclcpp/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

namespace ur_admittance_controller
{

/** @brief Degrees of freedom for 6D Cartesian space (3 translation + 3 rotation) */
static constexpr size_t DOF = 6;

/** @brief Default low-pass filter coefficient for force/torque sensor filtering */
static constexpr double DEFAULT_FILTER_COEFF = 0.95;


/** @brief Maximum age of transform data before considered stale (seconds) */
static constexpr double TRANSFORM_TIMEOUT = 0.1;

/** @brief 6x6 matrix type for mass, damping, and stiffness matrices */
using Matrix6d = Eigen::Matrix<double, 6, 6>;

/** @brief 6D vector type for forces, velocities, and poses in Cartesian space */
using Vector6d = Eigen::Matrix<double, 6, 1>;


/**
 * @brief Joint physical limits structure for UR5e
 * 
 * This structure contains only the officially published joint limits from UR.
 * Acceleration limits are intentionally omitted as:
 * 1. UR5e has no official acceleration limits (per ur_description package)
 * 2. Smoothness is achieved through position+velocity continuity
 * 3. scaled_joint_trajectory_controller handles trajectory interpolation
 */
struct JointLimits
{
  double min_position;      ///< Minimum joint position (radians)
  double max_position;      ///< Maximum joint position (radians)  
  double max_velocity;      ///< Maximum joint velocity (rad/s)
};

/**
 * @brief Parameters for safe controller startup and error recovery
 */
struct SafeStartupParams {
  double trajectory_duration = 5.0;      ///< Duration for startup trajectory (seconds)
  double stiffness_ramp_time = 2.0;      ///< Time to ramp up stiffness (seconds)
  double max_position_error = 0.15;      ///< Maximum position error before safety response (meters)
  double max_orientation_error = 0.5;    ///< Maximum orientation error before safety response (radians)
};



}  // namespace ur_admittance_controller

#endif  // UR_ADMITTANCE_CONTROLLER__ADMITTANCE_TYPES_HPP_
