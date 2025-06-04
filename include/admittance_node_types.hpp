// Type definitions and data structures for the UR admittance controller

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

/** @brief 6x6 matrix type for mass, damping, and stiffness matrices */
using Matrix6d = Eigen::Matrix<double, 6, 6>;

/** @brief 6D vector type for forces, velocities, and poses in Cartesian space */
using Vector6d = Eigen::Matrix<double, 6, 1>;




}  // namespace ur_admittance_controller

#endif  // UR_ADMITTANCE_CONTROLLER__ADMITTANCE_TYPES_HPP_
