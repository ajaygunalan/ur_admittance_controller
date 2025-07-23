# UR Admittance Controller Refactoring Plan (Revised)

## Executive Summary

This refactoring plan reorganizes the UR Admittance Controller codebase (~2000 lines) to separate algorithms from ROS infrastructure, eliminate duplication, and create maintainable architecture.

**Timeline**: 10-12 days  
**Risk**: Low (incremental changes, no functional modifications)  
**Goal**: Clean code organization without changing external behavior

### What Changes
- **IN SCOPE**: Extract algorithms, eliminate duplication, reorganize files
- **OUT OF SCOPE**: Changing algorithms, adding features, unit tests, extensive documentation

---

## 1. What We Have (Current State)

#### **Admittance System (610 lines total)**
- `admittance_node.cpp` (211 lines): ROS orchestration + embedded joint mapping
- `admittance_computations.cpp` (399 lines): Mixed bag of:
  - Pure algorithms: Admittance equation solver (lines 154-202)
  - Pure algorithms: Pose error calculation (lines 120-151)
  - Pure algorithms: Workspace limiting (lines 205-244)
  - Infrastructure: KDL setup, FK/IK wrappers

#### **Wrench System (201 lines)**
- `wrench_node.cpp`: Contains embedded Yu et al. compensation algorithm
  - Lines 74-87: Pure gravity compensation math
  - Lines 87-99: Pure adjoint transform math
  - Rest: ROS plumbing and YAML loading

#### **Calibration System (723 lines total)**
- `wrench_calibration_algorithm.cpp` (339 lines): **Already perfect!** Pure algorithms
  - LROM gravity estimation
  - Procrustes alignment
  - COM estimation
  - All using only Eigen, no ROS
- `wrench_calibration_node.cpp` (384 lines): ROS wrapper for robot motion

#### **Key Finding**: The calibration system already demonstrates our target architecture!

---

## 2. What Are The Issues

#### 🔴 **Critical Issues**
1. **Algorithm-Infrastructure Coupling**: 
   - Admittance equation solver buried in `admittance_computations.cpp`
   - Wrench compensation embedded in `wrench_node.cpp`
   - Cannot test algorithms without launching ROS

#### 🟡 **Moderate Issues**  
2. **Code Duplication Found**:
   - Eigen ↔ ROS wrench conversion (4 locations)
   - KDL initialization (`admittance_computations.cpp` + `init_robot.cpp`)
   - Joint state extraction (3 variants across nodes)
   - Adjoint transform math (2 locations)

3. **Mixed Responsibilities**:
   - `admittance_computations.cpp`: FK/IK + control law + safety limits
   - `wrench_node.cpp`: I/O + calibration loading + compensation math
   - Each file doing 3-4 different jobs

4. **Build Organization**: 
   - No shared library, everything builds as separate executables
   - Algorithms cannot be reused across nodes

#### 🟢 **Working Well**
- `wrench_calibration_algorithm.cpp` shows perfect separation
- Existing `error.hpp` with Result<T> pattern
- Clear Eigen-based interfaces for algorithms

---

## 3. What Is The Solution

**Core Idea**: Separate algorithms from ROS infrastructure following Drake's architecture pattern.

**Three Layers**:
1. **Utilities Layer**: Shared code used by everyone (types, conversions, math)
2. **Algorithms Layer**: Pure C++ computation (no ROS dependencies)
3. **Nodes Layer**: Thin ROS wrappers (just Subscribe → Algorithm → Publish)

**Build Structure**: 
- One small `ur_admittance_utilities` library for truly shared code
- Each node compiles with its own algorithms (faster builds, smaller binaries)

---

## 4. How The Solution Is Implemented

### 4.1 Example Transformations

**BEFORE** (admittance_computations.cpp, lines 154-202):
```cpp
void AdmittanceComputations::compute_admittance(...) {
    // Algorithm mixed with ROS logging and safety checks
    RCLCPP_DEBUG(node_->get_logger(), "Computing admittance...");
    
    // Pure algorithm buried here
    acceleration = M_inv * (F_ext - D * velocity - K * pose_error);
    velocity += acceleration * dt;
    
    // Mixed with safety limits
    if (velocity.norm() > max_velocity) {
        RCLCPP_WARN(node_->get_logger(), "Velocity limited!");
        velocity = velocity.normalized() * max_velocity;
    }
    
    // More ROS coupling...
}
```

**AFTER** (algorithms/admittance_controller.cpp):
```cpp
Result<Vector6d> AdmittanceController::computeVelocity(
    const Vector6d& F_ext,
    const Vector6d& pose_error,
    double dt) const {
    
    // Pure algorithm, no ROS
    Vector6d acceleration = M_inv * (F_ext - D * velocity - K * pose_error);
    Vector6d new_velocity = velocity + acceleration * dt;
    
    // Safety as pure function
    if (new_velocity.norm() > params.max_velocity) {
        new_velocity = new_velocity.normalized() * params.max_velocity;
    }
    
    velocity = new_velocity;
    return new_velocity;
}
```

#### Wrench Compensation Transformation

**BEFORE** (wrench_node.cpp, lines 74-99):
```cpp
void WrenchNode::wrench_callback(...) {
    // Get transforms from TF
    geometry_msgs::msg::TransformStamped transform_msg = 
        tf_buffer_->lookupTransform("base_link", "tool0", tf2::TimePointZero);
    
    // Algorithm embedded in callback
    Eigen::Matrix3d R = tf2::transformToEigen(transform_msg).rotation();
    Eigen::Vector3d force_compensated = raw_force - R * gravity_force - force_bias;
    
    // More embedded math...
    Eigen::Vector3d torque_compensated = raw_torque - 
        center_of_mass.cross(R * gravity_force) - torque_bias;
    
    // Publishing mixed with algorithm
    geometry_msgs::msg::WrenchStamped compensated_msg;
    // ... fill message ...
    compensated_pub_->publish(compensated_msg);
}
```

**AFTER** (algorithms/wrench_compensator.cpp):
```cpp
Wrench6d WrenchCompensator::compensate(
    const Wrench6d& raw_wrench,
    const Transform& X_EB) const {
    
    // Pure compensation algorithm
    Vector3d F_gravity_E = X_EB.rotation() * params.F_gravity_B;
    
    Wrench6d compensated;
    compensated.head<3>() = raw_wrench.head<3>() - F_gravity_E - params.F_bias_S;
    compensated.tail<3>() = raw_wrench.tail<3>() - 
        params.p_SCoM_S.cross(F_gravity_E) - params.T_bias_S;
    
    return compensated;
}
```

### 4.2 Target Directory Structure

```
ur_admittance_controller/
├── include/
│   ├── ur_admittance_controller/
│   │   ├── algorithms/              # Pure algorithms (header-only or with .cpp)
│   │   │   ├── admittance_controller.hpp
│   │   │   ├── wrench_compensator.hpp
│   │   │   └── calibration_solver.hpp
│   │   └── utilities/               # Shared utilities only
│   │       ├── types.hpp           # Consolidated from admittance_node_types.hpp
│   │       ├── error.hpp           # Moved from parent directory
│   │       ├── conversions.hpp     # New: Eigen ↔ ROS
│   │       ├── kinematics_utils.hpp # New: KDL wrappers
│   │       └── spatial_math.hpp    # New: Adjoint transforms
│   ├── fmt/                         # Existing vendored library (keep as-is)
│   └── tl/                          # Existing vendored library (keep as-is)
├── src/
│   ├── algorithms/                  # Algorithm implementations
│   │   ├── admittance_controller.cpp
│   │   ├── wrench_compensator.cpp
│   │   └── calibration_solver.cpp
│   ├── utilities/                   # Shared utility implementations
│   │   ├── conversions.cpp
│   │   ├── kinematics_utils.cpp
│   │   └── spatial_math.cpp
│   └── nodes/                       # ROS nodes
│       ├── admittance_node.cpp
│       ├── wrench_node.cpp
│       └── wrench_calibration_node.cpp
└── CMakeLists.txt                   # Build configuration
```

### 4.3 Build Structure (Drake-Style)
```
ROS Nodes ──compile with──> Node-Specific Algorithms
    ↓                              ↓
    └──────── both use ────────> Common Utilities
                                  (conversions, types, spatial_math)

Example:
- admittance_node compiles with admittance_controller.cpp
- wrench_node compiles with wrench_compensator.cpp  
- Both link to ur_admittance_utilities library
```

---

### 4.4 Step-by-Step Implementation Plan (4 Sprints)

#### Sprint 1: Create Shared Utilities (2 days)
**From:** 150 lines of duplicated code across 6 files  
**To:** Single implementation of each utility function

**Concrete Changes:**
- 4 copies of Eigen↔ROS conversion → 1 implementation in `conversions.hpp`
- 2 copies of KDL setup → 1 implementation in `kinematics_utils.hpp`  
- 2 copies of adjoint transforms → 1 implementation in `spatial_math.hpp`
- 3 different joint mappings → 1 standardized approach

#### Sprint 2: Extract Algorithms (4 days)
**From:** Algorithms buried in 400+ line files with ROS dependencies  
**To:** Pure C++ classes that compile with their respective nodes

**What Gets Extracted:**
```
admittance_computations.cpp (399 lines) → algorithms/admittance_controller.cpp
  └─ Extract: Admittance equation, pose error, workspace limits
  └─ Leave: ROS parameters, subscribers, publishers

wrench_node.cpp (201 lines) → algorithms/wrench_compensator.cpp
  └─ Extract: Gravity compensation, bias removal
  └─ Leave: TF lookups, message publishing

wrench_calibration_algorithm.cpp → algorithms/calibration_solver.cpp
  └─ Already pure! Just reorganize location
```

#### Sprint 3: Refactor Nodes (3 days)
**From:** Fat nodes doing everything  
**To:** Simple I/O adapters

**Node Transformation:**
```cpp
// BEFORE: 200+ lines mixing everything
void control_cycle() {
    // Get joints, do FK, compute error, apply control law,
    // check limits, do IK, publish - all mixed together
}

// AFTER: Simple orchestration only  
void control_cycle() {
    auto inputs = getInputs();
    auto result = controller_->compute(inputs);
    publishOutputs(result);
}
```

#### Sprint 4: Documentation & Cleanup (1-2 days)
**From:** "Where is that algorithm? Which file does safety checks?"  
**To:** Clear 3-layer architecture anyone can understand

---

## 5. Detailed Sprint Breakdown

### Sprint 1: Shared Utilities & Types (2 days)

#### Objectives
- Consolidate existing types and create missing utilities
- Eliminate all code duplication (150 lines)
- Set foundation for algorithm extraction

#### Tasks
| ID | Task | Effort | Assignee | Done |
|----|------|--------|----------|------|
| S1.1 | Consolidate types: merge `admittance_node_types.hpp` → `utilities/types.hpp` | 2h | | ☐ |
| S1.2 | Move `ur_admittance_controller/error.hpp` → `utilities/error.hpp` | 1h | | ☐ |
| S1.3 | Create `utilities/conversions.hpp` for Eigen ↔ ROS (eliminate 4 duplications) | 3h | | ☐ |
| S1.4 | Create `utilities/kinematics_utils.hpp` for KDL setup (eliminate 2 duplications) | 3h | | ☐ |
| S1.5 | Extract adjoint transforms to `utilities/spatial_math.hpp` (eliminate 2 duplications) | 2h | | ☐ |
| S1.6 | Update all nodes to use shared utilities | 3h | | ☐ |
| S1.7 | Verify zero code duplication remains | 1h | | ☐ |

#### Implementation Details

**types.hpp** (consolidating existing types):
```cpp
#pragma once
#include <Eigen/Dense>
#include <chrono>
#include <rclcpp/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

namespace ur_admittance_controller {

// From existing admittance_node_types.hpp
static constexpr size_t DOF = 6;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

// Additional type aliases for clarity
using Wrench6d = Vector6d;
using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;
using Transform = Eigen::Isometry3d;
using JointVector = std::vector<double>;

} // namespace
```

**conversions.hpp**:
```cpp
#pragma once
#include "types.hpp"
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace ur_admittance_controller::conversions {

// Wrench conversions
inline Wrench6d fromMsg(const geometry_msgs::msg::WrenchStamped& msg) {
    Wrench6d wrench;
    wrench << msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
              msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z;
    return wrench;
}

inline geometry_msgs::msg::WrenchStamped toMsg(
    const Wrench6d& wrench, 
    const std::string& frame_id,
    const rclcpp::Time& stamp) {
    geometry_msgs::msg::WrenchStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.wrench.force.x = wrench(0);
    msg.wrench.force.y = wrench(1);
    msg.wrench.force.z = wrench(2);
    msg.wrench.torque.x = wrench(3);
    msg.wrench.torque.y = wrench(4);
    msg.wrench.torque.z = wrench(5);
    return msg;
}

} // namespace
```

**spatial_math.hpp**:
```cpp
#pragma once
#include "types.hpp"

namespace ur_admittance_controller::spatial {

// Adjoint transformation for wrench (force/torque) vectors
// Transforms wrench from frame A to frame B given transform X_BA
inline Wrench6d transformWrench(const Wrench6d& wrench_A, 
                                const Transform& X_BA) {
    Wrench6d wrench_B;
    const Matrix3d& R_BA = X_BA.rotation();
    const Vector3d& p_BA = X_BA.translation();
    
    // Force transformation: f_B = R_BA * f_A
    wrench_B.head<3>() = R_BA * wrench_A.head<3>();
    
    // Torque transformation: tau_B = R_BA * tau_A + p_BA × (R_BA * f_A)
    wrench_B.tail<3>() = R_BA * wrench_A.tail<3>() + 
                         p_BA.cross(R_BA * wrench_A.head<3>());
    
    return wrench_B;
}

} // namespace
```

#### Deliverables
- [ ] Common type definitions in place
- [ ] Zero code duplication for conversions
- [ ] Frame conventions clearly documented
- [ ] All nodes updated to use utilities

#### Success Criteria
- All Eigen ↔ ROS conversions use common utilities
- KDL initialization code unified
- Frame naming consistent across codebase

---

### Sprint 2: Algorithm Extraction (4 days)

#### Objectives
- Extract pure algorithms from ROS infrastructure
- Create clean separation of concerns
- Keep algorithms with their respective nodes (Drake-style)

#### Tasks
| ID | Task | Effort | Assignee | Done |
|----|------|--------|----------|------|
| S2.1 | Move `wrench_calibration_algorithm.cpp` to `algorithms/calibration_solver.cpp` | 1h | | ☐ |
| S2.2 | Extract admittance math from `admittance_computations.cpp` to `algorithms/admittance_controller.cpp` | 8h | | ☐ |
| S2.3 | Extract wrench compensation from `wrench_node.cpp` to `algorithms/wrench_compensator.cpp` | 6h | | ☐ |
| S2.4 | Create parameter structs for each algorithm class | 3h | | ☐ |
| S2.5 | Update CMakeLists.txt for Drake-style fine-grained compilation | 2h | | ☐ |
| S2.6 | Ensure algorithms have no ROS dependencies | 4h | | ☐ |

#### Implementation Details

**CalibrationSolver Class**:
```cpp
namespace ur_admittance_controller::core {

class CalibrationSolver {
public:
    struct CalibrationResult {
        Matrix3d R_SE;              // Sensor to end-effector rotation
        Vector3d force_bias;        // Force bias in sensor frame
        Vector3d torque_bias;       // Torque bias in sensor frame
        Vector3d center_of_mass;    // Tool COM in sensor frame
        Vector3d gravity_in_base;   // Gravity vector in base frame
    };

    // Pure function - no ROS dependencies
    static Result<CalibrationResult> solve(
        const std::vector<CalibrationSample>& samples);
};

} // namespace
```

**AdmittanceController Class**:
```cpp
namespace ur_admittance_controller::core {

class AdmittanceController {
public:
    struct Parameters {
        Vector6d mass;
        Vector6d damping;
        Vector6d stiffness;
        double max_velocity;
        double max_acceleration;
        Vector6d workspace_limits;
    };

    explicit AdmittanceController(const Parameters& params);

    // Pure computation - no ROS dependencies
    Result<Vector6d> computeVelocity(
        const Vector6d& external_wrench,
        const Transform& current_pose,
        const Transform& desired_pose,
        double dt);
};

} // namespace
```

**CMakeLists.txt Drake-style fine-grained structure**:
```cmake
# Create shared utilities library (only truly shared code)
add_library(ur_admittance_utilities STATIC
  src/utilities/conversions.cpp
  src/utilities/kinematics_utils.cpp
  src/utilities/spatial_math.cpp
)

target_include_directories(ur_admittance_utilities PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(ur_admittance_utilities
  PUBLIC
    Eigen3::Eigen
    orocos_kdl
)

# Each node compiles with its own algorithms
add_executable(admittance_node
  src/nodes/admittance_node.cpp
  src/algorithms/admittance_controller.cpp  # Node-specific algorithm
)

target_link_libraries(admittance_node
  ur_admittance_utilities
  ur_admittance_controller_parameters
  ${NODE_DEPENDENCIES}
)

add_executable(wrench_node
  src/nodes/wrench_node.cpp
  src/algorithms/wrench_compensator.cpp  # Node-specific algorithm
)

target_link_libraries(wrench_node
  ur_admittance_utilities
  yaml-cpp
  rclcpp
  geometry_msgs
  # ... other dependencies
)

add_executable(wrench_calibration_node
  src/nodes/wrench_calibration_node.cpp
  src/algorithms/calibration_solver.cpp  # Node-specific algorithm
)

target_link_libraries(wrench_calibration_node
  ur_admittance_utilities
  yaml-cpp
  ${CALIBRATION_DEPENDENCIES}
)
```

#### Deliverables
- [ ] Three algorithm classes extracted with no ROS dependencies
- [ ] Clear parameter structures for configuration
- [ ] Consistent error handling with existing Result<T> from error.hpp
- [ ] CMakeLists.txt updated for fine-grained compilation
- [ ] Utilities library created for shared code only

#### Success Criteria
- Algorithms compile without ROS dependencies
- Each node compiles only with its needed algorithms
- No code duplication remains
- Node files become thin wrappers (just I/O orchestration)
- Faster incremental builds (change algorithm → rebuild only affected node)

---

### Sprint 3: Node Refactoring & Integration (3 days)

#### Objectives
- Refactor ROS nodes to use core algorithms
- Create thin wrapper nodes
- Ensure backward compatibility

#### Tasks
| ID | Task | Effort | Assignee | Done |
|----|------|--------|----------|------|
| S3.1 | Refactor admittance_node to use AdmittanceController | 8h | | ☐ |
| S3.2 | Refactor wrench_node to use WrenchCompensator | 6h | | ☐ |
| S3.3 | Refactor calibration_node to use CalibrationSolver | 6h | | ☐ |
| S3.4 | Ensure all ROS interfaces remain unchanged | 2h | | ☐ |
| S3.5 | Update error handling to use Result<T> consistently | 2h | | ☐ |

#### Implementation Details

**Refactored Node Structure**:
```cpp
// nodes/admittance_node.cpp - Thin ROS wrapper
class AdmittanceNode : public rclcpp::Node {
private:
    // Algorithm instance (no ROS dependencies)
    std::unique_ptr<AdmittanceController> controller;
    
    // Kinematics utility
    std::unique_ptr<KinematicsWrapper> kinematics;
    
public:
    void control_cycle() {
        // 1. Get inputs via ROS
        auto wrench = conversions::fromMsg(latest_wrench);
        auto joints = conversions::fromMsg(latest_joints);
        
        // 2. Compute kinematics
        auto fk_result = kinematics->computeFK(joints);
        if (!fk_result) {
            RCLCPP_ERROR(get_logger(), "FK failed: %s", fk_result.error().message.c_str());
            return;
        }
        
        // 3. Call pure algorithm
        auto result = controller->computeVelocity(
            wrench, fk_result.value(), desired_pose, dt);
        
        // 4. Convert and publish
        if (result) {
            velocity_pub->publish(conversions::toMsg(result.value()));
        } else {
            RCLCPP_ERROR(get_logger(), "Control failed: %s", result.error().message.c_str());
        }
    }
};
```

#### Deliverables
- [ ] All nodes refactored to use core algorithms
- [ ] ROS interfaces remain unchanged (backward compatible)
- [ ] Consistent error handling throughout
- [ ] Nodes become thin I/O wrappers

#### Success Criteria
- Each node is a thin wrapper around core algorithms
- No business logic in ROS nodes
- All ROS topics/services work as before

---

### Sprint 4: Documentation & Cleanup (1-2 days)

#### Objectives
- Complete documentation
- Final code cleanup
- Ensure maintainability

#### Tasks
| ID | Task | Effort | Assignee | Done |
|----|------|--------|----------|------|
| S4.1 | Create architecture documentation with diagrams | 3h | | ☐ |
| S4.2 | Add inline documentation for all public APIs | 3h | | ☐ |
| S4.3 | Remove all remaining TODOs and dead code | 2h | | ☐ |
| S4.4 | Update README with new architecture | 2h | | ☐ |
| S4.5 | Create migration guide for developers | 2h | | ☐ |

#### Deliverables
- [ ] Complete architecture documentation
- [ ] API documentation in headers
- [ ] Updated README
- [ ] Clean codebase with no TODOs

#### Success Criteria
- New developer can understand architecture in < 1 hour
- All public APIs documented
- Code ready for production use

---

## 6. Risk Management

### Technical Risks
| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| Breaking existing functionality | High | Low | Incremental changes, extensive testing |
| Performance regression | Medium | Medium | Continuous benchmarking |
| Integration issues | Medium | Low | Keep interfaces stable |

### Process Risks
| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| Scope creep | Medium | High | Strict sprint boundaries |
| Time overrun | Low | Medium | Buffer time in estimates |

---

## 7. Tracking and Metrics

### Sprint Burndown Tracking
```
Sprint 1: ░░░░░░░░░░░░░░░░░░░░   0% Complete  
Sprint 2: ░░░░░░░░░░░░░░░░░░░░   0% Complete
Sprint 3: ░░░░░░░░░░░░░░░░░░░░   0% Complete
Sprint 4: ░░░░░░░░░░░░░░░░░░░░   0% Complete
```

### Key Metrics
- **Code Duplication**: 150 lines → 0 lines
- **Node Complexity**: Mixed responsibilities → Thin I/O wrappers
- **Algorithm Organization**: Mixed in nodes → Separate algorithm files
- **Build Structure**: Everything in executables → Utilities library + node-specific algorithms
- **Incremental Build Speed**: Full rebuild → Only affected node rebuilds
- **Architecture Layers**: 1 → 3 (nodes, algorithms, utilities)

### Weekly Checkpoint Questions
1. Are we on track with the sprint goals?
2. Any blockers or risks emerging?
3. Do we need to adjust the plan?
4. What did we learn this week?

---

## 8. Post-Migration Maintenance

### Coding Standards
- Use Result<T> for error handling
- Document frame conventions in comments
- No ROS dependencies in core algorithms
- Keep nodes as thin wrappers

### Review Checklist
- [ ] Core algorithms compile without ROS
- [ ] No new duplication
- [ ] All conversions use common utilities
- [ ] Documentation updated
- [ ] ROS interfaces unchanged

---


## Appendix: Command Reference

### Building
```bash
# Clean build
colcon build --packages-select ur_admittance_controller --cmake-clean-cache

# Regular build
colcon build --packages-select ur_admittance_controller

# Build with compile commands for IDE
colcon build --packages-select ur_admittance_controller --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

### Running Nodes
```bash
# Run admittance controller
ros2 run ur_admittance_controller admittance_node

# Run wrench compensation
ros2 run ur_admittance_controller wrench_node

# Run calibration
ros2 run ur_admittance_controller wrench_calibration_node
```

### Checking Architecture
```bash
# List include dependencies
find src -name "*.cpp" -exec grep -H "^#include" {} \; | sort

# Check for ROS dependencies in core
grep -r "rclcpp\|ros2\|msg" include/ur_admittance_controller/core/

# Count lines per file
find src -name "*.cpp" -exec wc -l {} \; | sort -n
```

---

## Summary: Why This Refactoring Matters

### Before Refactoring
- **Problem**: "I need to test the admittance equation" → Must launch entire ROS system
- **Problem**: "Bug in wrench conversion" → Fix in 4 different places
- **Problem**: "What does this 400-line file do?" → It does 5 different things
- **Problem**: "Can we reuse the LROM algorithm?" → It's buried in a ROS node

### After Refactoring  
- **Solution**: Test algorithms with simple unit tests, no ROS needed
- **Solution**: Fix bugs once in `utilities/conversions.hpp`
- **Solution**: Each file has one clear purpose
- **Solution**: Import `#include <ur_admittance_controller/algorithms/calibration_solver.hpp>` anywhere

### The Bottom Line
Same 6 ROS nodes, same external behavior, but internally organized following Drake's proven architecture:
- Shared utilities in a single library (reduce duplication)
- Algorithms compile with their respective nodes (faster builds, smaller binaries)
- Clean separation without over-engineering
The calibration system already shows this pattern works - we're just applying it consistently.

---

## Sign-off

**Plan Approved By**: ___________________ Date: ___________

**Technical Lead**: ___________________ Date: ___________

**Project Manager**: ___________________ Date: ___________