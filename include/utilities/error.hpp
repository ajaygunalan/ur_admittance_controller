#pragma once

#include <string>
#include <tl/expected.hpp>

namespace ur_admittance_controller {

enum class ErrorCode {
  kFileNotFound,
  kInvalidConfiguration,
  kKinematicsInitFailed,  // Keep for initialization
  kCalibrationFailed,
  kIKSolverFailed,        // Keep for IK computation
  kTrajectoryExecutionFailed,
  kTimeout,
  kCommunicationTimeout
};

struct Error {
  ErrorCode code;
  std::string message;
};

template<typename T>
using Result = tl::expected<T, Error>;

using Status = Result<void>;  // Keep for initialization code

inline Error MakeError(ErrorCode code, const std::string& msg) {
  return {code, msg};
}

// ENSURE macro has been deleted - no longer using exceptions in control loops

}  // namespace ur_admittance_controller