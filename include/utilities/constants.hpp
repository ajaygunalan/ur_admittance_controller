#pragma once

#include <chrono>
#include <cmath>

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

// Movement constants
constexpr double DEFAULT_MOVEMENT_DURATION = 12.0;  // seconds

// Calibration angles
constexpr double CALIBRATION_ANGLE_LARGE = M_PI / 3.0;   // 60 degrees
constexpr double CALIBRATION_ANGLE_SMALL = M_PI / 6.0;   // 30 degrees

// Calibration constants
constexpr double CALIBRATION_INDEX_OFFSET = 1.0;
constexpr double CALIBRATION_MODULO_DIVISOR = 8.0;
constexpr double CALIBRATION_TRAJECTORY_DURATION = 3.0;  // seconds

// Workspace limits
constexpr double WORKSPACE_X_MIN = -1.0;  // m
constexpr double WORKSPACE_X_MAX = 1.0;   // m
constexpr double WORKSPACE_Y_MIN = -1.0;  // m
constexpr double WORKSPACE_Y_MAX = 1.0;   // m
constexpr double WORKSPACE_Z_MIN = 0.0;   // m
constexpr double WORKSPACE_Z_MAX = 1.0;   // m

// Control constants
constexpr double ARM_MAX_ACCELERATION = 1.0;  // rad/s^2

} // namespace constants
} // namespace ur_admittance_controller