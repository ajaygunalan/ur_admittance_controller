#pragma once

#include <chrono>

namespace ur_admittance_controller {
namespace constants {

// Physics constants
constexpr double GRAVITY = 9.81;  // m/s^2

// Timing constants
constexpr auto DEFAULT_TIMEOUT = std::chrono::seconds(10);
constexpr auto SERVICE_TIMEOUT = std::chrono::seconds(5);
constexpr int CONTROL_LOOP_HZ = 100;
constexpr int DEFAULT_QUEUE_SIZE = 10;

// Throttling for logging (milliseconds)
constexpr int LOG_THROTTLE_MS = 1000;

// Thresholds
constexpr double FORCE_THRESHOLD = 0.1;   // N
constexpr double TORQUE_THRESHOLD = 0.1;  // Nm

// Delays
constexpr auto SAMPLE_DELAY = std::chrono::milliseconds(100);

} // namespace constants
} // namespace ur_admittance_controller