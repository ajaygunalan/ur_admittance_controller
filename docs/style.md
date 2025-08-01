# C++ Style Guide

This codebase follows **Drake C++ style** (based on Google C++ style).

> **Terminology Note**: 
> - `PascalCase` = First letter uppercase (e.g., `ComputePoseError`)
> - `camelCase` = First letter lowercase (e.g., `computePoseError`)
> - Google/Drake use PascalCase for functions, not camelCase

## Core Principles

### Naming
| Type | Convention | Example |
|------|------------|---------|
| Functions | `PascalCase()` | `ComputePoseError()` |
| Classes | `PascalCase` | `WrenchNode` |
| Variables | `snake_case` | `joint_state` |
| Members | `snake_case_` | `tf_buffer_` |
| Constants | `kPascalCase` | `kTimeout` |
| Namespaces | `lowercase` | `ur_admittance_controller` |

### Formatting
- **Line length**: 100 characters max
- **Indentation**: 2 spaces (no tabs)
- **Braces**: Same line (K&R style)

## Code Organization

### Include Order
```cpp
#pragma once

#include <memory>                    // C++ standard library
#include <vector>

#include <rclcpp/rclcpp.hpp>        // ROS2 headers
#include <sensor_msgs/msg/joint_state.hpp>

#include <Eigen/Core>               // Third-party libraries
#include <kdl/chain.hpp>

#include <ur_admittance_controller/utilities/types.hpp>  // Project headers
```

### Namespace Layout
```cpp
namespace ur_admittance_controller {

class WrenchNode : public rclcpp::Node {  // No indentation
  // ...
};

}  // namespace ur_admittance_controller
```

## Detailed Conventions

### Functions & Parameters
```cpp
// Short functions - single line
void SetVelocity(double vel) { velocity_ = vel; }

// Long parameter lists - break after opening paren
void ProcessWrench(
    const geometry_msgs::msg::WrenchStamped& msg,
    const Transform& tool_transform);

// Constructor initialization
WrenchNode::WrenchNode()
    : Node("wrench_node"),
      tf_buffer_(std::make_unique<tf2_ros::Buffer>(get_clock())) {
  // body
}
```

### Enums
```cpp
enum class ErrorCode {
  kSuccess,
  kFileNotFound,
  kInvalidParameter
};
```

### Comments & Documentation
```cpp
// Single-line comments for clarity
Vector6d ComputePoseError();  // Returns pose error in base frame

// Mathematical references preserved
// Apply damping limits (Yu et al. Eq. 24-34)
```

## Project-Specific Patterns

### Error Handling
```cpp
Result<Vector3d> EstimateGravity();  // Returns error or value
Status LoadCalibration();            // Returns error status
ENSURE(condition, "message");        // Runtime invariants
```

### Domain Notation
- **Transforms**: `X_AB` (transform from B to A)
- **Forces**: `F_P_B` (force at P in frame B)
- **Type aliases**: `Force3d` instead of generic `Vector3d`

### File Extensions
- **This project**: `.hpp/.cpp` (ROS2 convention)
- **Drake standard**: `.h/.cc`
- We chose `.hpp/.cpp` for better ROS2 ecosystem integration