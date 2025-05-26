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

// RT-safe logging infrastructure
enum class LogLevel : uint8_t {
  DEBUG = 0,
  INFO = 1,
  WARN = 2,
  ERROR = 3,
  FATAL = 4
};

struct LogMessage {
  LogLevel level;
  std::chrono::steady_clock::time_point timestamp;
  std::string message;
  
  LogMessage() = default;
  LogMessage(LogLevel lvl, const std::string& msg) 
    : level(lvl), timestamp(std::chrono::steady_clock::now()), message(msg) {}
};

// Lock-free circular buffer for RT-safe logging
template<size_t N>
class RTLogBuffer {
private:
  std::array<LogMessage, N> buffer_;
  std::atomic<size_t> write_index_{0};
  std::atomic<size_t> read_index_{0};
  
public:
  // RT-safe: Add message to buffer (non-blocking)
  bool push(LogLevel level, const std::string& message) noexcept {
    size_t current_write = write_index_.load(std::memory_order_relaxed);
    size_t next_write = (current_write + 1) % N;
    
    // Check if buffer is full
    if (next_write == read_index_.load(std::memory_order_acquire)) {
      return false; // Buffer full, drop message
    }
    
    // Write message
    buffer_[current_write] = LogMessage(level, message);
    write_index_.store(next_write, std::memory_order_release);
    return true;
  }
  
  // Non-RT: Retrieve message from buffer
  bool pop(LogMessage& message) noexcept {
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