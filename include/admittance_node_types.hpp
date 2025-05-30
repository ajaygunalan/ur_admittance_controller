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
 * @brief Joint physical limits structure
 */
struct JointLimits
{
  double min_position;      ///< Minimum joint position (radians)
  double max_position;      ///< Maximum joint position (radians)
  double max_velocity;      ///< Maximum joint velocity (rad/s)
  double max_acceleration;  ///< Maximum joint acceleration (rad/s²)
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

/**
 * @brief Thread-safe transform cache with double buffering
 *
 * This structure provides lock-free caching of transform data for real-time
 * access. It uses double buffering to allow non-real-time updates while
 * maintaining consistent reads from the real-time thread.
 *
 * @note Thread-safe for one writer (non-RT) and multiple readers (RT)
 * @note Uses atomic operations for synchronization without locks
 */
struct TransformCache {
  /**
   * @brief Internal cache entry with double buffering
   */
  struct Entry {
    /**
     * @brief Single buffer containing transform data
     */
    struct Buffer {
      geometry_msgs::msg::TransformStamped transform{};  ///< ROS transform message
      Matrix6d adjoint = Matrix6d::Zero();               ///< Precomputed adjoint matrix
      rclcpp::Time timestamp{};                          ///< Transform timestamp
    };
    
    std::array<Buffer, 2> buffers;          ///< Double buffer for lock-free updates
    std::atomic<int> active_buffer{0};      ///< Index of currently active buffer
    std::atomic<bool> valid{false};         ///< Whether cache contains valid data
    
    /**
     * @brief Get the currently active buffer for reading
     * @return Const reference to active buffer
     */
    const Buffer& getActiveBuffer() const noexcept {
      return buffers[active_buffer.load(std::memory_order_acquire)];
    }
    
    /**
     * @brief Get the inactive buffer for writing
     * @return Reference to inactive buffer
     */
    Buffer& getInactiveBuffer() noexcept {
      return buffers[1 - active_buffer.load(std::memory_order_relaxed)];
    }
    
    /**
     * @brief Atomically swap active and inactive buffers
     */
    void swapBuffers() noexcept {
      int current = active_buffer.load(std::memory_order_relaxed);
      active_buffer.store(1 - current, std::memory_order_release);
      valid.store(true, std::memory_order_release);
    }
  };
  
  Entry cache;                    ///< The actual cache entry
  
  std::string target_frame;       ///< Target frame for transform
  std::string source_frame;       ///< Source frame for transform
  rclcpp::Time last_update{};     ///< Time of last successful update
  
  /**
   * @brief Reset cache to invalid state
   */
  void reset() noexcept {
    cache.buffers[0] = Entry::Buffer{};
    cache.buffers[1] = Entry::Buffer{};
    cache.valid.store(false, std::memory_order_release);
  }
  
  /**
   * @brief Check if cache contains valid data
   * @return true if cache is valid
   */
  bool isValid() const noexcept {
    return cache.valid.load(std::memory_order_acquire);
  }
  
  /**
   * @brief Get the cached transform data
   * @return Const reference to transform buffer
   */
  const Entry::Buffer& getTransform() const noexcept {
    return cache.getActiveBuffer();
  }
  
  /**
   * @brief Update the cached transform (thread-safe)
   * @param new_transform New transform data
   * @param new_adjoint Precomputed adjoint matrix
   * @param timestamp Transform timestamp
   */
  void updateTransform(const geometry_msgs::msg::TransformStamped& new_transform,
                      const Matrix6d& new_adjoint,
                      const rclcpp::Time& timestamp) noexcept {
    auto& buffer = cache.getInactiveBuffer();
    buffer.transform = new_transform;
    buffer.adjoint = new_adjoint;
    buffer.timestamp = timestamp;
    cache.swapBuffers();
    last_update = timestamp;
  }
};


}  // namespace ur_admittance_controller

#endif  // UR_ADMITTANCE_CONTROLLER__ADMITTANCE_TYPES_HPP_
