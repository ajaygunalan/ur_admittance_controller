#ifndef UR_ADMITTANCE_CONTROLLER__ADMITTANCE_TYPES_HPP_
#define UR_ADMITTANCE_CONTROLLER__ADMITTANCE_TYPES_HPP_

#include <chrono>
#include <Eigen/Dense>
#include <rclcpp/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace ur_admittance_controller
{

// Constants for improved code clarity and maintainability
static constexpr size_t DOF = 6;
static constexpr double DEFAULT_FILTER_COEFF = 0.95;
static constexpr double TRANSFORM_TIMEOUT = 0.1;  // seconds

// Clean Eigen typedefs for better readability
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

// RT-safe logging infrastructure
enum class LogLevel : uint8_t {
  DEBUG = 0,
  INFO = 1,
  WARN = 2,
  ERROR = 3,
  FATAL = 4
};

// Predefined message types for RT-safe logging
// Add all possible log messages used in RT context here
enum class RTLogType : uint16_t {
  // System messages
  SYSTEM_INITIALIZED = 0,
  SYSTEM_SHUTDOWN = 1,
  
  // Errors
  ERROR_SENSOR_READ_FAILED = 100,
  ERROR_TRANSFORM_INVALID = 101,
  ERROR_KINEMATICS_FAILED = 102,
  ERROR_JOINT_LIMITS = 103,
  ERROR_CONTROL_COMPUTATION = 104,
  
  // Warnings
  WARN_POSE_ERROR_LIMIT = 200,
  WARN_VELOCITY_LIMITED = 201,
  WARN_DEADBAND_ACTIVE = 202,
  
  // Info
  INFO_STIFFNESS_ENGAGED = 300,
  INFO_DRIFT_RESET = 301,
  INFO_PARAMETER_UPDATED = 302,

  // Parameterized messages (with numeric values)
  PARAM_FORCE_READING = 400,      // With force value
  PARAM_JOINT_POSITION = 401,     // With joint index and position
  PARAM_CART_VELOCITY = 402       // With velocity component and value
};

// RT-safe log message with fixed-size buffer and no string allocations
struct RTLogMessage {
  LogLevel level;
  RTLogType type;
  std::chrono::steady_clock::time_point timestamp;
  
  // For parameterized messages (up to 3 parameters)
  double param1;
  double param2;
  double param3;
  
  // For cases where a static string literal is needed
  const char* literal_msg;
  
  RTLogMessage() : 
    level(LogLevel::INFO), 
    type(RTLogType::SYSTEM_INITIALIZED),
    timestamp(std::chrono::steady_clock::now()),
    param1(0.0), param2(0.0), param3(0.0),
    literal_msg(nullptr) {}
  
  // RT-safe: No string construction, just enum and timestamp
  RTLogMessage(LogLevel lvl, RTLogType typ) : 
    level(lvl), type(typ),
    timestamp(std::chrono::steady_clock::now()),
    param1(0.0), param2(0.0), param3(0.0),
    literal_msg(nullptr) {}
  
  // RT-safe: With parameters, no string operations
  RTLogMessage(LogLevel lvl, RTLogType typ, double p1, double p2 = 0.0, double p3 = 0.0) : 
    level(lvl), type(typ),
    timestamp(std::chrono::steady_clock::now()),
    param1(p1), param2(p2), param3(p3),
    literal_msg(nullptr) {}
    
  // RT-safe: For static string literals only (no allocation/copy)
  // WARNING: literal_msg MUST be a string literal or static char array
  // that outlives this object. Never pass a dynamically allocated string.
  RTLogMessage(LogLevel lvl, const char* msg) : 
    level(lvl), type(RTLogType::SYSTEM_INITIALIZED),
    timestamp(std::chrono::steady_clock::now()),
    param1(0.0), param2(0.0), param3(0.0),
    literal_msg(msg) {}
};

// Lock-free circular buffer for RT-safe logging
template<size_t N>
class RTLogBuffer {
private:
  std::array<RTLogMessage, N> buffer_;
  std::atomic<size_t> write_index_{0};
  std::atomic<size_t> read_index_{0};
  
public:
  // RT-safe: Add message to buffer using enum type (no string ops)
  bool push(LogLevel level, RTLogType type) noexcept {
    size_t current_write = write_index_.load(std::memory_order_relaxed);
    size_t next_write = (current_write + 1) % N;
    
    // Check if buffer is full
    if (next_write == read_index_.load(std::memory_order_acquire)) {
      return false; // Buffer full, drop message
    }
    
    // Write message (RT-safe: no dynamic memory allocation)
    buffer_[current_write] = RTLogMessage(level, type);
    write_index_.store(next_write, std::memory_order_release);
    return true;
  }
  
  // RT-safe: Add parameterized message (no string ops)
  bool push(LogLevel level, RTLogType type, 
            double param1, double param2 = 0.0, double param3 = 0.0) noexcept {
    size_t current_write = write_index_.load(std::memory_order_relaxed);
    size_t next_write = (current_write + 1) % N;
    
    // Check if buffer is full
    if (next_write == read_index_.load(std::memory_order_acquire)) {
      return false; // Buffer full, drop message
    }
    
    // Write message with parameters (RT-safe: no string operations)
    buffer_[current_write] = RTLogMessage(level, type, param1, param2, param3);
    write_index_.store(next_write, std::memory_order_release);
    return true;
  }
  
  // RT-safe: Add message with string literal (no allocation/copy)
  // WARNING: msg MUST be a string literal or static char array that outlives this object
  bool push(LogLevel level, const char* msg) noexcept {
    size_t current_write = write_index_.load(std::memory_order_relaxed);
    size_t next_write = (current_write + 1) % N;
    
    // Check if buffer is full
    if (next_write == read_index_.load(std::memory_order_acquire)) {
      return false; // Buffer full, drop message
    }
    
    // Store pointer to literal (RT-safe: no string copy)
    buffer_[current_write] = RTLogMessage(level, msg);
    write_index_.store(next_write, std::memory_order_release);
    return true;
  }
  
  // Non-RT: Retrieve message from buffer
  bool pop(RTLogMessage& message) noexcept {
    size_t current_read = read_index_.load(std::memory_order_relaxed);
    if (current_read == write_index_.load(std::memory_order_acquire)) {
      return false; // Buffer empty
    }
    
    message = buffer_[current_read];
    read_index_.store((current_read + 1) % N, std::memory_order_release);
    return true;
  }
  
  bool empty() const noexcept {
    return read_index_.load(std::memory_order_relaxed) == 
           write_index_.load(std::memory_order_relaxed);
  }
};

// RT-safe logging macros
#define RT_LOG_BUFFER_SIZE 1000
using RTLogger = RTLogBuffer<RT_LOG_BUFFER_SIZE>;

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
  // Atomic data structure for RT-safe access
  struct Entry {
    // Use double buffering for atomic updates
    struct Buffer {
      geometry_msgs::msg::TransformStamped transform{};
      Matrix6d adjoint = Matrix6d::Zero();
      rclcpp::Time timestamp{};
    };
    
    // Double buffer for atomic swapping
    std::array<Buffer, 2> buffers;
    std::atomic<int> active_buffer{0};
    std::atomic<bool> valid{false};
    
    // RT-safe read access
    const Buffer& getActiveBuffer() const noexcept {
      return buffers[active_buffer.load(std::memory_order_acquire)];
    }
    
    // Non-RT write access with atomic swap
    Buffer& getInactiveBuffer() noexcept {
      return buffers[1 - active_buffer.load(std::memory_order_relaxed)];
    }
    
    // Atomically activate the updated buffer
    void swapBuffers() noexcept {
      int current = active_buffer.load(std::memory_order_relaxed);
      active_buffer.store(1 - current, std::memory_order_release);
      valid.store(true, std::memory_order_release);
    }
  };
  
  // Cache entries for different transform pairs
  Entry cache;
  
  // Non-RT data
  std::string target_frame;
  std::string source_frame;
  rclcpp::Time last_update{};
  
  // Methods
  void reset() noexcept {
    cache.buffers[0] = Entry::Buffer{};
    cache.buffers[1] = Entry::Buffer{};
    cache.valid.store(false, std::memory_order_release);
  }
  
  // Check if the transform is valid for RT thread
  bool isValid() const noexcept {
    return cache.valid.load(std::memory_order_acquire);
  }
  
  // RT-safe access to transform data
  const Entry::Buffer& getTransform() const noexcept {
    return cache.getActiveBuffer();
  }
  
  // Non-RT update of transform data
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