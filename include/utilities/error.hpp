#pragma once

#include <string>

#include <tl/expected.hpp>

namespace ur_admittance_controller {

enum class ErrorCode {
  // Setup failures (throw during initialization)
  kFileNotFound,
  kInvalidConfiguration,
  kKinematicsInitFailed,

  // Runtime failures (return Status in real-time loops)
  kIKSolverFailed,
  kTrajectoryExecutionFailed,
  kTimeout,
  kCommunicationTimeout,
  kCalibrationFailed
};

struct Error {
  ErrorCode code;
  std::string message;
};

template<typename T>
using Result = tl::expected<T, Error>;

using Status = Result<void>;

inline Error MakeError(ErrorCode code, const std::string& msg) {
  return {code, msg};
}

// Invariant check - for conditions that should NEVER be false
#define ENSURE(cond, msg) \
  do { \
    if (!(cond)) { \
      throw std::runtime_error(std::string("Invariant violated: ") + msg); \
    } \
  } while(0)

}  // namespace ur_admittance_controller