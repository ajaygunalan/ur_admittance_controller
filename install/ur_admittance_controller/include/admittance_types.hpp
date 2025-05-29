/**
 * @file admittance_types.hpp
 * @brief Type definitions and data structures for the UR admittance controller
 *
 * This file contains custom types, enumerations, and data structures used
 * throughout the admittance controller. It includes thread-safe logging
 * utilities, transform caching mechanisms, and parameter structures.
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
 * @brief Logging severity levels for real-time safe logging
 */
enum class LogLevel : uint8_t {
  DEBUG = 0,  ///< Detailed information for debugging
  INFO = 1,   ///< General informational messages
  WARN = 2,   ///< Warning messages for recoverable issues
  ERROR = 3,  ///< Error messages for serious problems
  FATAL = 4   ///< Fatal errors requiring immediate shutdown
};

/**
 * @brief Message types for real-time logging system
 *
 * These message types allow structured logging from real-time contexts
 * without dynamic memory allocation or blocking operations.
 */
enum class RTLogType : uint16_t {
  // System lifecycle events
  SYSTEM_INITIALIZED = 0,     ///< Controller successfully initialized
  SYSTEM_SHUTDOWN = 1,        ///< Controller shutdown initiated
  
  // Error conditions (100-199)
  ERROR_SENSOR_READ_FAILED = 100,   ///< Force/torque sensor read failure
  ERROR_TRANSFORM_INVALID = 101,    ///< Transform lookup or validity error
  ERROR_KINEMATICS_FAILED = 102,    ///< Inverse kinematics computation failed
  ERROR_JOINT_LIMITS = 103,         ///< Joint position/velocity limits exceeded
  ERROR_CONTROL_COMPUTATION = 104,  ///< Admittance control computation error
  
  // Warning conditions (200-299)
  WARN_POSE_ERROR_LIMIT = 200,     ///< Pose error exceeds safety threshold
  WARN_VELOCITY_LIMITED = 201,      ///< Velocity clamped to safety limits
  WARN_DEADBAND_ACTIVE = 202,       ///< Force below deadband threshold
  
  // Informational events (300-399)
  INFO_STIFFNESS_ENGAGED = 300,    ///< Stiffness ramping completed
  INFO_DRIFT_RESET = 301,           ///< Drift compensation reset applied
  INFO_PARAMETER_UPDATED = 302,     ///< Dynamic parameter update applied

  // Parametric logging (400-499)
  PARAM_FORCE_READING = 400,        ///< Log force sensor value with param1
  PARAM_JOINT_POSITION = 401,       ///< Log joint position (joint# in param1, value in param2)
  PARAM_CART_VELOCITY = 402         ///< Log Cartesian velocity component
};

/**
 * @brief Real-time safe log message structure
 *
 * This structure can be safely created and passed in real-time contexts
 * without dynamic memory allocation. It supports both typed messages
 * and literal string messages (using static strings only).
 */
struct RTLogMessage {
  LogLevel level;                                      ///< Severity level of the message
  RTLogType type;                                      ///< Type of message for structured logging
  std::chrono::steady_clock::time_point timestamp;     ///< High-resolution timestamp
  
  double param1;                                       ///< First numeric parameter
  double param2;                                       ///< Second numeric parameter
  double param3;                                       ///< Third numeric parameter
  
  const char* literal_msg;                             ///< Optional literal message (must be static)
  
  /** @brief Default constructor */
  RTLogMessage() : 
    level(LogLevel::INFO), 
    type(RTLogType::SYSTEM_INITIALIZED),
    timestamp(std::chrono::steady_clock::now()),
    param1(0.0), param2(0.0), param3(0.0),
    literal_msg(nullptr) {}
  
  /** @brief Constructor for typed messages without parameters */
  RTLogMessage(LogLevel lvl, RTLogType typ) : 
    level(lvl), type(typ),
    timestamp(std::chrono::steady_clock::now()),
    param1(0.0), param2(0.0), param3(0.0),
    literal_msg(nullptr) {}
  
  /** @brief Constructor for typed messages with numeric parameters */
  RTLogMessage(LogLevel lvl, RTLogType typ, double p1, double p2 = 0.0, double p3 = 0.0) : 
    level(lvl), type(typ),
    timestamp(std::chrono::steady_clock::now()),
    param1(p1), param2(p2), param3(p3),
    literal_msg(nullptr) {}
    
  /** @brief Constructor for literal messages (string must be static/literal) */
  RTLogMessage(LogLevel lvl, const char* msg) : 
    level(lvl), type(RTLogType::SYSTEM_INITIALIZED),
    timestamp(std::chrono::steady_clock::now()),
    param1(0.0), param2(0.0), param3(0.0),
    literal_msg(msg) {}
};

/**
 * @brief Lock-free circular buffer for real-time logging
 *
 * This class implements a single-producer, single-consumer lock-free
 * circular buffer for passing log messages from real-time to non-real-time
 * contexts. It uses atomic operations with proper memory ordering to
 * ensure thread safety without locks.
 *
 * @tparam N Size of the circular buffer (number of messages)
 *
 * @note Thread-safe for one producer (RT thread) and one consumer (non-RT thread)
 * @note Does not allocate memory after construction
 * @note Messages are dropped if buffer is full (fail-fast behavior)
 */
template<size_t N>
class RTLogBuffer {
private:
  std::array<RTLogMessage, N> buffer_;   ///< Fixed-size message buffer
  std::atomic<size_t> write_index_{0};   ///< Producer write position
  std::atomic<size_t> read_index_{0};    ///< Consumer read position
  
public:
  /**
   * @brief Push a typed message without parameters
   * @param level Log severity level
   * @param type Message type
   * @return true if message was pushed, false if buffer full
   */
  bool push(LogLevel level, RTLogType type) noexcept {
    size_t current_write = write_index_.load(std::memory_order_relaxed);
    size_t next_write = (current_write + 1) % N;
    
    // Check if buffer is full
    if (next_write == read_index_.load(std::memory_order_acquire)) {
      return false;
    }
    
    buffer_[current_write] = RTLogMessage(level, type);
    write_index_.store(next_write, std::memory_order_release);
    return true;
  }
  
  /**
   * @brief Push a typed message with numeric parameters
   * @param level Log severity level
   * @param type Message type
   * @param param1 First numeric parameter
   * @param param2 Second numeric parameter (default: 0.0)
   * @param param3 Third numeric parameter (default: 0.0)
   * @return true if message was pushed, false if buffer full
   */
  bool push(LogLevel level, RTLogType type, 
            double param1, double param2 = 0.0, double param3 = 0.0) noexcept {
    size_t current_write = write_index_.load(std::memory_order_relaxed);
    size_t next_write = (current_write + 1) % N;
    
    if (next_write == read_index_.load(std::memory_order_acquire)) {
      return false;
    }
    
    buffer_[current_write] = RTLogMessage(level, type, param1, param2, param3);
    write_index_.store(next_write, std::memory_order_release);
    return true;
  }
  
  /**
   * @brief Push a literal string message
   * @param level Log severity level
   * @param msg Static/literal string message
   * @return true if message was pushed, false if buffer full
   * @warning msg must point to static memory (string literal)
   */
  bool push(LogLevel level, const char* msg) noexcept {
    size_t current_write = write_index_.load(std::memory_order_relaxed);
    size_t next_write = (current_write + 1) % N;
    
    if (next_write == read_index_.load(std::memory_order_acquire)) {
      return false;
    }
    
    buffer_[current_write] = RTLogMessage(level, msg);
    write_index_.store(next_write, std::memory_order_release);
    return true;
  }
  
  /**
   * @brief Pop a message from the buffer
   * @param message Output parameter to receive the message
   * @return true if message was popped, false if buffer empty
   */
  bool pop(RTLogMessage& message) noexcept {
    size_t current_read = read_index_.load(std::memory_order_relaxed);
    if (current_read == write_index_.load(std::memory_order_acquire)) {
      return false;
    }
    
    message = buffer_[current_read];
    read_index_.store((current_read + 1) % N, std::memory_order_release);
    return true;
  }
  
  /**
   * @brief Check if buffer is empty
   * @return true if no messages in buffer
   */
  bool empty() const noexcept {
    return read_index_.load(std::memory_order_relaxed) == 
           write_index_.load(std::memory_order_relaxed);
  }
};

/** @brief Default size for real-time log buffer */
#define RT_LOG_BUFFER_SIZE 1000

/** @brief Type alias for the real-time logger used throughout the controller */
using RTLogger = RTLogBuffer<RT_LOG_BUFFER_SIZE>;

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
