#ifndef UR_ADMITTANCE_CONTROLLER__ADMITTANCE_TYPES_HPP_
#define UR_ADMITTANCE_CONTROLLER__ADMITTANCE_TYPES_HPP_

#include <chrono>
#include <Eigen/Dense>
#include <rclcpp/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace ur_admittance_controller
{

static constexpr size_t DOF = 6;
static constexpr double DEFAULT_FILTER_COEFF = 0.95;
static constexpr double TRANSFORM_TIMEOUT = 0.1;

using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

enum class LogLevel : uint8_t {
  DEBUG = 0,
  INFO = 1,
  WARN = 2,
  ERROR = 3,
  FATAL = 4
};

enum class RTLogType : uint16_t {
  SYSTEM_INITIALIZED = 0,
  SYSTEM_SHUTDOWN = 1,
  
  ERROR_SENSOR_READ_FAILED = 100,
  ERROR_TRANSFORM_INVALID = 101,
  ERROR_KINEMATICS_FAILED = 102,
  ERROR_JOINT_LIMITS = 103,
  ERROR_CONTROL_COMPUTATION = 104,
  
  WARN_POSE_ERROR_LIMIT = 200,
  WARN_VELOCITY_LIMITED = 201,
  WARN_DEADBAND_ACTIVE = 202,
  
  INFO_STIFFNESS_ENGAGED = 300,
  INFO_DRIFT_RESET = 301,
  INFO_PARAMETER_UPDATED = 302,

  PARAM_FORCE_READING = 400,
  PARAM_JOINT_POSITION = 401,
  PARAM_CART_VELOCITY = 402
};

struct RTLogMessage {
  LogLevel level;
  RTLogType type;
  std::chrono::steady_clock::time_point timestamp;
  
  double param1;
  double param2;
  double param3;
  
  const char* literal_msg;
  
  RTLogMessage() : 
    level(LogLevel::INFO), 
    type(RTLogType::SYSTEM_INITIALIZED),
    timestamp(std::chrono::steady_clock::now()),
    param1(0.0), param2(0.0), param3(0.0),
    literal_msg(nullptr) {}
  
  RTLogMessage(LogLevel lvl, RTLogType typ) : 
    level(lvl), type(typ),
    timestamp(std::chrono::steady_clock::now()),
    param1(0.0), param2(0.0), param3(0.0),
    literal_msg(nullptr) {}
  
  RTLogMessage(LogLevel lvl, RTLogType typ, double p1, double p2 = 0.0, double p3 = 0.0) : 
    level(lvl), type(typ),
    timestamp(std::chrono::steady_clock::now()),
    param1(p1), param2(p2), param3(p3),
    literal_msg(nullptr) {}
    
  RTLogMessage(LogLevel lvl, const char* msg) : 
    level(lvl), type(RTLogType::SYSTEM_INITIALIZED),
    timestamp(std::chrono::steady_clock::now()),
    param1(0.0), param2(0.0), param3(0.0),
    literal_msg(msg) {}
};

template<size_t N>
class RTLogBuffer {
private:
  std::array<RTLogMessage, N> buffer_;
  std::atomic<size_t> write_index_{0};
  std::atomic<size_t> read_index_{0};
  
public:
  bool push(LogLevel level, RTLogType type) noexcept {
    size_t current_write = write_index_.load(std::memory_order_relaxed);
    size_t next_write = (current_write + 1) % N;
    
    if (next_write == read_index_.load(std::memory_order_acquire)) {
      return false;
    }
    
    buffer_[current_write] = RTLogMessage(level, type);
    write_index_.store(next_write, std::memory_order_release);
    return true;
  }
  
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
  
  bool pop(RTLogMessage& message) noexcept {
    size_t current_read = read_index_.load(std::memory_order_relaxed);
    if (current_read == write_index_.load(std::memory_order_acquire)) {
      return false;
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

#define RT_LOG_BUFFER_SIZE 1000
using RTLogger = RTLogBuffer<RT_LOG_BUFFER_SIZE>;

struct JointLimits
{
  double min_position;
  double max_position;
  double max_velocity;
  double max_acceleration;
};

struct SafeStartupParams {
  double trajectory_duration = 5.0;
  double stiffness_ramp_time = 2.0;
  double max_position_error = 0.15;
  double max_orientation_error = 0.5;
};

struct TransformCache {
  struct Entry {
    struct Buffer {
      geometry_msgs::msg::TransformStamped transform{};
      Matrix6d adjoint = Matrix6d::Zero();
      rclcpp::Time timestamp{};
    };
    
    std::array<Buffer, 2> buffers;
    std::atomic<int> active_buffer{0};
    std::atomic<bool> valid{false};
    
    const Buffer& getActiveBuffer() const noexcept {
      return buffers[active_buffer.load(std::memory_order_acquire)];
    }
    
    Buffer& getInactiveBuffer() noexcept {
      return buffers[1 - active_buffer.load(std::memory_order_relaxed)];
    }
    
    void swapBuffers() noexcept {
      int current = active_buffer.load(std::memory_order_relaxed);
      active_buffer.store(1 - current, std::memory_order_release);
      valid.store(true, std::memory_order_release);
    }
  };
  
  Entry cache;
  
  std::string target_frame;
  std::string source_frame;
  rclcpp::Time last_update{};
  
  void reset() noexcept {
    cache.buffers[0] = Entry::Buffer{};
    cache.buffers[1] = Entry::Buffer{};
    cache.valid.store(false, std::memory_order_release);
  }
  
  bool isValid() const noexcept {
    return cache.valid.load(std::memory_order_acquire);
  }
  
  const Entry::Buffer& getTransform() const noexcept {
    return cache.getActiveBuffer();
  }
  
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

}

#endif