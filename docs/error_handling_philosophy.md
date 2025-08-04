# ROS2-Compliant Error Handling - Based on Nav2 and MoveIt2 Patterns

## IMPLEMENTATION INSTRUCTIONS FOR NEW SESSION

**Purpose**: Align error handling with ROS2 best practices from Nav2 and MoveIt2, removing excessive defensive programming while keeping standard ROS2 error patterns.

**Starting Point**: This document contains complete instructions to transform the codebase to match ROS2 standards as used in Nav2 controller_server and MoveIt2 servo nodes.

**Key Changes**:
1. Delete ALL 15 ENSURE checks across entire codebase (not used in ROS2)
2. Change control loop functions from Status to void (ROS2 pattern)
3. Keep try-catch for transforms (standard ROS2 practice)
4. Keep initialization throws, remove runtime throws
5. Simple validation checks without ENSURE macros

**Follow the phases below in order. Each phase has complete code replacements.**

---

## Philosophy: "Like Nav2 and MoveIt2 - Log and Continue"

### What Nav2 and MoveIt2 Actually Do (Our Model)
```cpp
// Nav2 controller_server pattern:
void ControllerServer::computeControl() {
  // Transform handling with try-catch (ROS2 standard)
  try {
    transform = tf_buffer_->lookupTransform(
      target_frame, source_frame, tf2::TimePointZero, timeout);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(logger_, "Transform failed: %s", ex.what());
    return;  // Skip cycle, use last valid
  }
  
  // Simple validation (no ENSURE)
  if (msg->ranges.empty() || msg->range_max <= 0.0) {
    RCLCPP_WARN(logger_, "Invalid sensor data");
    return;
  }
  
  // Just compute and send
  computeVelocity();
  publishCommand();
}
```
**Pattern: Try-catch ONLY for transforms, simple checks for data, NO Status returns!**

### What's Wrong With Our Code vs ROS2 Standards?
- **Not ROS2**: ENSURE macros (no ROS2 node uses these)
- **Not ROS2**: Status/Result<T> in control loops (ROS2 uses void)
- **Not ROS2**: Throwing exceptions in runtime (ROS2 logs and continues)
- **OK by ROS2**: Try-catch for transforms (this IS the standard)

## Our New Approach: ROS2 Standard Compliance

### Step 1: DELETE All Unnecessary Checks
**Philosophy**: "If it was initialized once, it won't fail"

**DELETE these ENSURE checks completely:**
- `admittance_computations.cpp:258-259` - FK solver exists (set once at startup)
- `admittance_computations.cpp:294-296` - Vector sizes (always 6, set at startup)  
- `admittance_computations.cpp:325-326` - IK solver exists (set once at startup)
- `admittance_computations.cpp:239-242` - Admittance parameter validation (checked at startup)
- `admittance_node.cpp:11-12` - Joint count (from URDF, never changes)
- `wrench_node.cpp:101` - TF buffer exists (initialized once)

### Step 2: Simplify Control Loop (ROS2 Style)
```cpp
// OLD: Complex error propagation
Status ControlCycle() {
  if (auto status = GetXBPCurrent(); !status) 
    return status;  // Propagate error
  ComputePoseError();
  ComputeAdmittance();
  LimitToWorkspace();
  if (auto status = ComputeAndPubJointVelocities(); !status)
    return status;  // Propagate error
}

// NEW: Just run everything
void ControlCycle() {
  if (!joint_states_received_) return;  // Skip until ready
  
  GetXBPCurrent();           // Compute FK
  ComputePoseError();        
  ComputeAdmittance();       
  LimitToWorkspace();        
  ComputeAndPubJointVelocities();  // Compute IK & publish
  // That's it! No error checking needed
}
```

### Step 3: Simplify Return Types
- `GetXBPCurrent()`: Status → void
- `ComputeAndPubJointVelocities()`: Status → void  
- `ControlCycle()`: Status → void
- Keep Result<T> ONLY for file I/O

## Implementation Plan

### Phase 1: Delete ENSURE from Control Loops (6 deletions)

**admittance_computations.cpp:**

1. **In UpdateAdmittanceParameters() function**, find and DELETE:
```cpp
ENSURE(p.mass.size() == 6 && p.stiffness.size() == 6 && p.damping.size() == 6,
       "Admittance parameter vectors must all have exactly 6 elements");
ENSURE((Eigen::Map<const Eigen::VectorXd>(p.mass.data(), 6).array() > 0).all(),
       "All mass values must be positive to avoid division by zero");
```

2. **In GetXBPCurrent() function**, find and DELETE:
```cpp
ENSURE(num_joints_ == 6 && q_current_.size() == num_joints_ && fk_pos_solver_,
       "FK preconditions violated: joints must be 6 and solver initialized");
```

3. **In ComputeAdmittance() function**, find and DELETE:
```cpp
ENSURE(M_inverse_diag.size() == 6 && D_diag.size() == 6 && K_diag.size() == 6 &&
       !F_P_B.hasNaN() && !V_P_B_commanded.hasNaN() && !X_BP_error.hasNaN(),
       "Admittance computation preconditions violated");
```

4. **In ComputeAndPubJointVelocities() function**, find and DELETE:
```cpp
ENSURE(ik_vel_solver_ && V_P_B_commanded.size() == 6 && !V_P_B_commanded.hasNaN(),
       "IK preconditions violated: solver, velocity, or joint arrays invalid");
```

**admittance_node.cpp:**

5. **In MapJointStates() function**, find and DELETE:
```cpp
ENSURE(params_.joints.size() == 6,
       "UR robot must have exactly 6 joints configured");
```

**wrench_node.cpp:**

6. **In WrenchCallback() function**, find and DELETE:
```cpp
ENSURE(tf_buffer_ != nullptr, "TF buffer not initialized");
```

### Phase 2: Update Header Files

**In `include/admittance_node.hpp`**, change these function declarations:

1. Find `Status ControlCycle();` and change to:
```cpp
void ControlCycle();
```

2. Find `Status ComputeAndPubJointVelocities();` and change to:
```cpp
void ComputeAndPubJointVelocities();
```

3. Find `Status GetXBPCurrent();` and change to:
```cpp
void GetXBPCurrent();
```

### Phase 3: Simplify Function Implementations

**In `admittance_computations.cpp`:**

1. **Replace the entire GetXBPCurrent() function** with:
```cpp
void AdmittanceNode::GetXBPCurrent() {
  RCLCPP_DEBUG_ONCE(get_logger(), "FK solver: %zu joints, %d segments",
                    num_joints_, kdl_chain_.getNrOfSegments());

  auto result = ComputeForwardKinematics(q_current_, fk_pos_solver_.get(), X_W3P);
  if (!result) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), constants::LOG_THROTTLE_MS, "FK failed: %s",
                          result.error().message.c_str());
    return;  // Continue with last valid pose
  }

  X_BP_current = result.value();

  // Cache wrist transform for IK
  for (size_t i = 0; i < num_joints_; ++i) {
    q_kdl_(i) = q_current_[i];
  }
  fk_pos_solver_->JntToCart(q_kdl_, X_BW3);
}
```

2. **Replace the entire ComputeAndPubJointVelocities() function** with:
```cpp
void AdmittanceNode::ComputeAndPubJointVelocities() {
  auto result = ComputeInverseKinematicsVelocity(
      V_P_B_commanded, X_BP_current, X_BW3, q_current_, ik_vel_solver_.get());

  if (!result) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), constants::LOG_THROTTLE_MS,
        "IK failed: vel=[%.3f,%.3f,%.3f]m/s - safety stop",
        V_P_B_commanded(0), V_P_B_commanded(1), V_P_B_commanded(2));
    std::fill(q_dot_cmd_.begin(), q_dot_cmd_.end(), 0.0);
  } else {
    q_dot_cmd_ = result.value();
  }

  velocity_msg_.data = q_dot_cmd_;
  velocity_pub_->publish(velocity_msg_);
}
```

**In `admittance_node.cpp`:**

3. **Replace the entire ControlCycle() function** with:
```cpp
void AdmittanceNode::ControlCycle() {
  if (!joint_states_received_) {
    RCLCPP_INFO_ONCE(get_logger(), "Waiting for initial joint states...");
    return;
  }

  GetXBPCurrent();
  ComputePoseError();
  ComputeAdmittance();
  LimitToWorkspace();
  ComputeAndPubJointVelocities();
}
```

### Phase 4: Update Main Loop

**In `admittance_node.cpp` main() function**, find this code around line 185-190:
```cpp
auto status = node->ControlCycle();
if (!status) {
  RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), ur_admittance_controller::constants::LOG_THROTTLE_MS,
                       "Control cycle error: %s", status.error().message.c_str());
  // Continue running for now - real-time loops should be resilient
}
```

**Replace it with:**
```cpp
node->ControlCycle();
```

### Phase 5: Clean Up error.hpp

**Replace the entire content of `include/utilities/error.hpp`** with:
```cpp
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
```

### What We Keep (ROS2 Standard Patterns)

1. **Transform handling (Nav2/MoveIt2 pattern)** - Standard ROS2 practice
   - Try-catch blocks for `lookupTransform()` - THIS IS THE ROS2 WAY
   - `canTransform()` checks before lookup
   - Log with RCLCPP_WARN/ERROR and continue
   - **Note**: This is NOT excessive - it's how ALL ROS2 nodes handle transforms

2. **Sensor data validation (Nav2 pattern)** - Simple checks
   - `if (!data.allFinite())` - reject bad data
   - `hasNaN()` checks on sensor readings
   - No ENSURE, just simple if statements

3. **Initialization exceptions (MoveIt2 pattern)** - OK to throw
   - Calibration loading must succeed
   - Kinematics initialization must succeed
   - Equilibrium pose must be loaded
   - **Pattern**: Throw during setup, never during runtime

4. **File I/O errors** - Can genuinely fail
   - Keep Result<T> for file operations
   - Keep try-catch for YAML parsing

### Phase 6: Simplify ALL Other Files

**wrench_node.cpp:**
- Remove ENSURE check line 101 - TF buffer check (already counted in Phase 1)
- Remove ENSURE check line 155 - rotation matrix size check
- Remove ENSURE check line 167 - calibration vector size check
- **KEEP line 78 throw** - Calibration loading (initialization phase)
- **REMOVE line 192 throw** - Transform failure (runtime - should log and continue)
- Keep try/catch for transform lookups at lines 112, 131, 185, 211 (ROS2 standard)
- Keep NaN checks with `allFinite()` (sensor data validation)

**wrench_calibration_node.cpp:**
- Keep Status returns - this is a one-time calibration process, not real-time
- Keep all error handling as-is (calibration can genuinely fail)

**wrench_calibration_algorithm.cpp:**
- Remove ENSURE check line 15 - sample validation
- Remove ENSURE check line 82 - gravity vector validation
- Remove ENSURE check line 130 - force bias validation  
- Remove ENSURE check line 202 - gravity vector for mass extraction
- Keep Result<T> returns for calibration computations (can mathematically fail)

**init_robot.cpp:**
- Keep all error handling - this is initialization, not real-time control
- Throwing exceptions here is fine (one-time setup)

**admittance_computations.cpp (additional):**
- Remove ENSURE in LoadKinematics() line ~385 checking joint count
- **KEEP line 222 throw** - Equilibrium loading (initialization phase)
- Keep the Result<T> return for LoadKinematics() (initialization function)

**admittance_node.cpp (additional):**
- Remove 2 ENSURE checks in Initialize() and main() (lines ~121, ~163)
- **KEEP line 125 throw** - Kinematics initialization (setup phase)
- Keep try/catch in main() for initialization failures

**utilities/conversions.cpp:**
- No changes needed - simple conversion functions, no error handling

**utilities/file_io.cpp:**
- Keep Result<T> for LoadConfigFile() - file I/O can genuinely fail
- Keep try/catch for YAML parsing errors

**utilities/kinematics.cpp:**
- Keep Result<T> for InitializeFromUrdf() - URDF parsing can fail
- No ENSURE macros to remove

## Summary

**The Goal:** Align with ROS2 standards as used in Nav2 and MoveIt2 - balanced error handling.

**What We're Doing (Following ROS2 Patterns):**
1. **DELETE 15 total ENSURE checks** - No ROS2 node uses these
2. **Simplify 3 return types** from Status to void - Nav2/MoveIt2 pattern
3. **Clean up error.hpp** - Remove ENSURE macro completely
4. **Keep try-catch for transforms** - This IS the ROS2 standard way
5. **Keep initialization throws** - Standard ROS2 practice for setup

**ROS2 Patterns We're Following:**
- **Nav2 pattern**: Try-catch for transforms, simple if-checks for sensors
- **MoveIt2 pattern**: Throw during init, log-and-continue during runtime
- **No ENSURE macros**: Not used in any ROS2 packages
- **No Status in loops**: ROS2 uses void functions in control loops

**Result:**
- **~50% less error handling** - Balanced reduction (not 70% - that's too aggressive)
- **ROS2 compliant** - Matches Nav2 controller_server and MoveIt2 servo
- **No runtime crashes** - Exceptions only during initialization
- **Proper transform handling** - Try-catch is the ROS2 way, not excessive

## Quick Implementation Checklist

| Phase | Action | Files | Details |
|-------|--------|-------|---------|
| 1 | DELETE ENSURE in control loops | admittance_computations.cpp (4), admittance_node.cpp (1), wrench_node.cpp (1) | 6 checks total |
| 2 | Update header declarations | admittance_node.hpp | Change 3 functions from Status to void |
| 3 | Simplify function implementations | admittance_computations.cpp, admittance_node.cpp | Replace with simplified versions |
| 4 | Update main loop | admittance_node.cpp main() | Remove error checking on ControlCycle() |
| 5 | Clean error.hpp | include/utilities/error.hpp | Remove ENSURE macro completely |
| 6 | Clean remaining files | All other src/ files | Remove 9 more ENSURE checks (see Phase 6 details), keep initialization throws |

**Verification Steps:**
1. Build the package: `cd ~/ros2_ws && colcon build --packages-select ur_admittance_controller`
2. If build fails, check that all Status return types were changed to void
3. Verify no ENSURE macros remain: `grep -r "ENSURE" src/`

**Time to implement:** ~30 minutes
**Code deleted:** ~100 lines
**Complexity removed:** ~50%