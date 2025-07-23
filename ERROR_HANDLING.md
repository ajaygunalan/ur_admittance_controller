# Error Handling in UR Admittance Controller

This document describes the error handling patterns used in the UR Admittance Controller, following Drake's three-tier error model.

## Overview

We implement Drake's three-tier error handling model with `tl::expected` for type-safe error propagation:

- **Tier 1: Programmer mistakes** → Assertions/throw (should never happen in correct code)
- **Tier 2: One-shot setup failures** → Throw exceptions (initialization, configuration)
- **Tier 3: Runtime faults in loops** → Return `Status`/`Result<T>` objects (real-time safe)

## Key Types

```cpp
// Error code enumeration
enum class ErrorCode {
  // Setup failures (used in Tier 2 contexts - throw via .value())
  kFileNotFound,
  kInvalidConfiguration,
  kKinematicsInitFailed,
  kCalibrationLoadFailed,
  
  // Runtime failures (used in Tier 3 contexts - return as Status)
  kIKSolverFailed,
  kTransformLookupFailed,
  kTrajectoryExecutionFailed,
  kTimeout
};

// Error struct with code and message
struct Error {
  ErrorCode code;
  std::string message;
};

// Type aliases for cleaner code
template<typename T>
using Result = tl::expected<T, Error>;

using Status = Result<void>;
```

## Error Handling Patterns

### Pattern 1: Tier 1 (Programmer Mistakes) - Assert Invariants

For conditions that should never be false in correct code:

```cpp
// Use ENSURE macro for invariants
ENSURE(num_joints_ == 6, "UR robots must have exactly 6 joints");
ENSURE(ik_solver_ != nullptr, "IK solver must be initialized before use");
```

### Pattern 2: Tier 2 (One-Shot Setup) - Throw on Failure

For initialization and configuration that runs once, use `.value()` to throw on error:

```cpp
// In wrench_calibration_node.cpp
auto f_gravity_B = estimateGravitationalForceInBaseFrame(samples).value();
//                                                         ^^^^^^^^
// Throws std::bad_expected_access if function returns error
```

### Pattern 3: Tier 3 (Runtime Faults) - Return Status

For real-time loops that must handle faults gracefully, explicitly check and propagate errors:

```cpp
Status AdmittanceNode::control_cycle() {
  // Drake-style explicit error checking
  if (auto fk_status = get_X_BP_current(); !fk_status) {
    return fk_status;  // Propagate error
  }
  
  // Continue with next operation
  compute_pose_error();
  
  if (auto ik_status = compute_and_pub_joint_velocities(); !ik_status) {
    return ik_status;  // Propagate error
  }
  
  return {};  // Success
}
```

### Pattern 4: Main Loop Fault Tolerance

Main loops should handle runtime faults gracefully and continue operating:

```cpp
while (rclcpp::ok()) {
  executor.spin_some();
  
  if (auto status = node->control_cycle(); !status) {
    RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                         "Control cycle error: %s", status.error().message.c_str());
    // Continue running - real-time loops should be resilient
  }
  
  loop_rate.sleep();
}
```

## Drake Philosophy: Explicit Good, Implicit Bad

We follow Drake's principle of making error handling explicit and visible:

### ❌ DON'T: Hide Control Flow
```cpp
// Bad - macro hides control flow
RETURN_IF_ERROR(get_X_BP_current());
```

### ✅ DO: Explicit Error Checking
```cpp
// Good - control flow is visible
if (auto status = get_X_BP_current(); !status) {
  return status;
}
```

## Real-Time Considerations

1. **No Exceptions in RT**: Tier 3 code returns `Status`, never throws
2. **Pre-allocated Errors**: Error objects are stack-allocated (no heap allocation)
3. **Throttled Logging**: Use `RCLCPP_ERROR_THROTTLE` to prevent log flooding
4. **Fail-Safe Behavior**: On faults, execute safety action before returning error
   - IK failure → publish zero velocities
   - Transform failure → skip compensation
   - Trajectory failure → stop motion

## Function Categories by Tier

### Tier 1: Programmer Mistakes (ENSURE macro, assertions)
- Array bounds checks (should never exceed robot DOF)
- Null pointer checks (solvers must be initialized)
- State consistency (kinematics must be loaded before use)
- Precondition violations (matrix dimensions must match)

### Tier 2: One-Shot Setup (throw via .value() or direct throw)
- `WrenchNode` constructor - missing calibration files
- `AdmittanceNode::initialize()` - URDF parsing failures  
- `load_kinematics()` - invalid robot description
- Calibration algorithms - insufficient data samples
- Parameter loading - malformed YAML files

### Tier 3: Runtime Faults in Loops (return Status/Result)
- `control_cycle()` - FK/IK solver numerical failures
- `get_X_BP_current()` - forward kinematics singularities
- `compute_and_pub_joint_velocities()` - IK solver failures, NaN results
- `moveToJointPosition()` - trajectory execution timeouts
- `wrench_callback()` - transform lookup failures

## Error Code Guidelines

Choose error codes based on the failure category:
- `kFileNotFound`: Missing config files, packages
- `kInvalidConfiguration`: Bad YAML, invalid parameters
- `kKinematicsInitFailed`: URDF parsing, chain extraction, FK/IK setup
- `kIKSolverFailed`: Runtime IK failures, NaN results
- `kTransformLookupFailed`: TF2 lookup failures
- `kTrajectoryExecutionFailed`: Motion planning/execution failures

## Migration from Bool Returns

Before (bool returns, no context):
```cpp
bool load_kinematics() {
  if (!kdl_parser::treeFromString(urdf, tree)) {
    RCLCPP_ERROR(get_logger(), "Invalid URDF");
    return false;  // No error details!
  }
  return true;
}
```

After (Status returns, rich context):
```cpp
Status load_kinematics() {
  if (!kdl_parser::treeFromString(urdf, tree)) {
    RCLCPP_ERROR(get_logger(), "Invalid URDF");
    return tl::unexpected(make_error(ErrorCode::kKinematicsInitFailed,
                                   "Failed to parse URDF into KDL tree"));
  }
  return {};  // Success
}
```

## Testing Error Paths

### Tier 1: Programmer Mistakes
- Use debug builds with assertions enabled
- Write unit tests that violate invariants
- Verify immediate crash with clear message

### Tier 2: Setup Failures  
- Provide invalid URDF files
- Delete required calibration files
- Pass malformed YAML configurations
- Verify clean shutdown with descriptive errors

### Tier 3: Runtime Faults
- Inject IK solver failures (singular configurations)
- Simulate transform timeouts
- Force NaN values in computations
- Verify:
  - Errors propagate without crashes
  - Safety actions execute (zero velocities)
  - Loop continues operating
  - Error messages are throttled

## Performance Impact

The `tl::expected` type is designed for zero-overhead error handling:
- No dynamic allocations
- No exceptions in hot paths
- Compiler optimizes success path
- Stack-allocated error objects

Modern compilers (GCC 9+, Clang 10+) optimize the explicit error checking pattern to be as efficient as macro-based approaches while maintaining readability.