#pragma once

#include <string>
#include <tl/expected.hpp>
#include <fmt/core.h>

namespace ur_admittance_controller {

// Error codes following Drake's three-tier model
enum class ErrorCode {
  // Tier 2: Setup/config failures (throw during initialization)
  kFileNotFound,
  kInvalidConfiguration,
  kKinematicsInitFailed,
  
  // Tier 3: Runtime failures (return Status in real-time loops)
  kIKSolverFailed,
  kTrajectoryExecutionFailed,
  kTimeout,
  kCommunicationTimeout   // General communication timeout
};

// Simple error struct with code and message
struct Error {
  ErrorCode code;
  std::string message;
};

// Type aliases for cleaner code
template<typename T>
using Result = tl::expected<T, Error>;

using Status = Result<void>;

// Helper to create errors
inline Error make_error(ErrorCode code, const std::string& msg) {
  return Error{code, msg};
}

// Drake philosophy: Avoid macros that hide control flow
// Instead, use explicit status checking:
//   if (auto status = some_function(); !status) {
//     return status;  // Clear, debuggable, no hidden control flow
//   }

// Drake-style invariant macro (Tier 1: programmer errors)
// Use for conditions that should NEVER be false in correct code
#define ENSURE(cond, msg) \
  do { \
    if (!(cond)) { \
      throw std::runtime_error(std::string("Invariant violated: ") + msg); \
    } \
  } while(0)

}  // namespace ur_admittance_controller