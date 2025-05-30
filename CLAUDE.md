# CLAUDE.md

## 📋 **ur_admittance_controller: Comprehensive Package Overview**

> **Standalone 6-DOF force-compliant motion control for Universal Robots - push the robot and it moves!**

### **🎯 Package Purpose**
A standalone ROS2 node that provides **6-DOF admittance control** for Universal Robots, enabling compliant behavior where external forces are transformed into smooth robot motions using the classic admittance equation: **M·ẍ + D·ẋ + K·x = F**

### **🏗️ Architecture & Integration**

#### **Ecosystem Integration**
- **✅ ur_simulation_gz**: Receives robot description and publishes to F/T sensor topics
- **✅ ur_description (system)**: Official UR5e joint limits and specifications  
- **✅ scaled_joint_trajectory_controller**: Optimized trajectory execution with speed scaling
- **✅ Gazebo/Real UR Driver**: Seamless operation in simulation and real hardware

#### **Data Flow Pipeline**
```
[F/T Sensor] → [Admittance Node] → [scaled_joint_trajectory_controller] → [Robot]
     ↑               ↑                           ↑                          ↓
[/wrist_ft_sensor]  [Official UR5e Specs]  [Position+Velocity]      [Joint Motion]
                    [from /robot_description]    [Trajectory]
```

### **📂 Package Structure (Minimized Architecture)**

#### **Core Source Files (Consolidated)**
```
src/
├── admittance_node.cpp          # Main node, ROS2 interfaces, initialization + transforms
├── admittance_computations.cpp  # Core admittance algorithm + inlined matrix utilities
└── sensor_handling.cpp          # F/T sensor processing and transform handling
```

#### **Configuration & Launch (Streamlined)**
```
config/
└── admittance_config.yaml       # Essential parameter definitions (86 lines)

launch/
└── ur_admittance.launch.py      # Simplified launch file (20 lines)

scripts/
├── test_suite.py                # Consolidated testing (50 lines)
└── validate.py                  # Frame notation validation (30 lines)
```

#### **Headers & Types (Optimized)**
```
include/
├── admittance_node.hpp           # Main node class definition
├── admittance_node_types.hpp     # Data structures (JointLimits, etc.)
└── admittance_constants.hpp      # Essential mathematical constants only
```

### **⚙️ Core Algorithm Implementation**

#### **1. Admittance Control Law**
**Equation**: `M·acceleration + D·velocity + K·position_error = F_external`

**Implementation** (Runge-Kutta 4th order integration):
```cpp
// Compute pose error
Vector6d error = computePoseError_tip_base();

// RK4 integration for smooth acceleration computation  
Vector6d k1 = mass_inverse_ * (wrench_filtered_ - damping_ * v0 - stiffness_ * error);
Vector6d k2 = mass_inverse_ * (wrench_filtered_ - damping_ * v1 - stiffness_ * error);
// ... k3, k4 steps
desired_vel_ = v0 + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
```

#### **2. Transform Processing**
- **Wrench transformation**: F/T sensor frame → robot base frame
- **Pose computation**: Current vs desired end-effector pose  
- **Direct tf2 calls**: No caching, uses tf2::TimePointZero with 50ms timeout

#### **3. Joint Space Conversion**
- **Inverse kinematics**: Cartesian velocity → joint space via `kinematics_interface`
- **Joint limiting**: Position + velocity limits from official UR5e specs
- **No acceleration limiting**: Trusts `scaled_joint_trajectory_controller`

### **🔧 Control Features**

#### **Control Modes**
1. **Pure Admittance** (default): `stiffness = [0,0,0,0,0,0]`
   - Force → motion mapping only
   - No position return spring

2. **Impedance Control**: `stiffness = [100,100,100,10,10,10]`  
   - Position regulation + force compliance
   - Configurable stiffness per axis

3. **Selective Compliance**: `enabled_axes = [true,true,false,false,false,true]`
   - Enable/disable specific DOF
   - Common: XY translation + Z rotation

#### **Safety Features**
- **Safe startup**: Gradual stiffness engagement over configurable ramp time
- **Deadband filtering**: Minimum force threshold to prevent noise-triggered motion
- **Joint limits**: Official UR5e position/velocity limits enforcement
- **Velocity limiting**: Cartesian and joint space velocity caps
- **Emergency stop**: Safe position hold on errors

### **📡 ROS2 Interface**

#### **Subscriptions**
```yaml
/robot_description:        std_msgs/String           # Official UR5e URDF specs
/joint_states:            sensor_msgs/JointState     # Current robot joint positions  
/wrist_ft_sensor:         geometry_msgs/WrenchStamped # Force/torque measurements
```

#### **Publishers** 
```yaml
/scaled_joint_trajectory_controller/joint_trajectory:  # Main control output
    trajectory_msgs/JointTrajectory                     # Position + velocity commands

/admittance_cartesian_velocity:  geometry_msgs/Twist   # Monitoring: commanded Cartesian velocity
/admittance_pose_error:         geometry_msgs/Twist    # Monitoring: current pose error
```

### **🎛️ Key Innovations**

#### **1. Official UR5e Integration**
- **Joint limits**: Loaded from system `ur_description` package via `/robot_description` topic
- **Automatic updates**: Changes when UR packages are updated
- **Fallback system**: Hardcoded UR5e defaults if topic unavailable

#### **2. Optimized Trajectory Control**  
- **Target**: `scaled_joint_trajectory_controller` (UR-specific design)
- **Message format**: Position + velocity (no acceleration - UR5e has none officially)
- **Benefits**: Speed scaling, safety handling, proper interpolation

#### **3. Minimized Architecture (41% Codebase Reduction)**
- **Files eliminated**: 4 files removed (node_integration.cpp, matrix_utilities.hpp, etc.)
- **Lines eliminated**: 1,060 lines removed (2,584 → 1,524 total lines)
- **Scripts consolidated**: 6 files → 2 files (test_suite.py, validate.py)
- **Documentation streamlined**: Verbose comments → concise implementation notes
- **Matrix utilities inlined**: Direct implementations, fewer abstractions
- **Result**: Faster builds, cleaner code, easier maintenance

#### **4. Real-time Performance (Enhanced)**
- **Control frequency**: 500Hz (2ms period)
- **Thread-based**: Dedicated control thread for consistent timing
- **Memory efficient**: Pre-allocated messages, no dynamic allocation in RT loop
- **Build time**: Improved compilation speed with fewer files and includes
- **Code clarity**: Direct implementations vs complex inheritance patterns

### **🔍 Technical Specifications**

#### **Supported Robots**
- **Primary**: UR5e (tested and optimized)
- **Compatible**: All UR e-Series robots (UR3e, UR10e, UR16e, UR20e)
- **Joint limits**: Loaded from official ur_description specifications

#### **Dependencies**
```yaml
Core ROS2:     rclcpp, std_msgs, geometry_msgs, sensor_msgs, trajectory_msgs
Transforms:    tf2, tf2_ros, tf2_eigen, tf2_geometry_msgs  
Kinematics:    kinematics_interface, pluginlib, urdf
Parameters:    generate_parameter_library
Math:          Eigen3 (system dependency)
```

## MCP Tool Usage Strategy

### 1. **Sequential Thinking** (`mcp__sequential-thinking__sequentialthinking`)
**When**: Complex debugging, architecture design, multi-step problems
```
Examples:
- Debug control loop instabilities
- Design perception-to-planning pipeline
- Trace transform chain issues
- Plan major refactoring
```

### 2. **Web Search** (`mcp__brave-search__brave_web_search`)
**When**: Need latest practices, specifications, or solutions
```
Examples:
- "ROS2 [specific API] best practices"
- "[Sensor/Robot] specifications"
- "Real-time control patterns ROS2"
```

### 3. **Memory** (`mcp__memory__create_entities/relations`)
**When**: Track what works, build project knowledge
```
Examples:
- Working parameter sets → Entity: "Config_[UseCase]"
- Performance benchmarks → Entity: "Benchmark_[Component]"
- Solved issues → Entity: "Solution_[Problem]"
```

### 4. **Fetch** (`mcp__fetch__fetch`)
**When**: Need official docs or implementation details
```
Key URLs:
- ROS2 docs: docs.ros.org
- Package repos: github.com/ros2/[package]
- Hardware manuals: manufacturer sites
```

## Efficient Patterns

**Debug Issues**: Sequential thinking → Search similar → Fetch docs → Store solution

**Add Features**: Search examples → Plan with sequential → Implement → Create entities

**Optimize**: Benchmark → Search techniques → Apply → Update benchmarks

## Build & Test

```bash
# Build
cd ~/ros2_ws
colcon build --packages-select ur_admittance_controller --symlink-install
source install/setup.bash

# Test
ros2 run ur_admittance_controller admittance_node --ros-args --log-level debug
ros2 launch ur_admittance_controller ur_admittance.launch.py use_sim:=true
```

## Common Tasks

### Debug Performance
```bash
# Monitor real-time
ros2 topic hz /scaled_joint_trajectory_controller/joint_trajectory
ros2 topic echo /wrist_ft_sensor --once
ros2 node info /admittance_node
```

### Modify Parameters
1. Edit config YAML
2. Rebuild for parameter generation
3. Use `ros2 param set` for runtime changes

### Trace Data Flow
```
Perception → Processing → Control → Actuation
    ↓            ↓           ↓          ↓
 Sensors    Transforms   Algorithms  Commands
```

## Troubleshooting Patterns

### No Response to Input
- Check thresholds/deadbands
- Verify data flow with echo
- Confirm transforms valid

### Instability/Oscillation  
- Increase damping/filtering
- Reduce gains/stiffness
- Check control frequency

### Performance Issues
- Profile with `ros2 topic hz`
- Use thread-based loops
- Pre-allocate messages

## 📊 Codebase Minimization Achievements

### **Total Reduction: 41% (1,060 lines eliminated)**

| **Component** | **Before** | **After** | **Lines Saved** | **Reduction** |
|---------------|------------|-----------|-----------------|---------------|
| **Python Scripts** | 650+ lines | 80 lines | **570 lines** | **88%** |
| **Launch Files** | 75 lines | 20 lines | **55 lines** | **73%** |
| **Config Files** | 190 lines | 86 lines | **104 lines** | **55%** |
| **C++ Sources** | 1,112 lines | 1,063 lines | **49 lines** | **4%** |
| **C++ Headers** | 557 lines | 275 lines | **282 lines** | **51%** |
| **TOTAL** | **2,584 lines** | **1,524 lines** | **1,060 lines** | **41%** |

### **Files Eliminated**
- ❌ `node_integration.cpp` (35 lines) → merged into `admittance_node.cpp`
- ❌ `matrix_utilities.hpp` (169 lines) → inlined into `admittance_computations.cpp`
- ❌ `ur_admittance_tests.py` (320 lines) → consolidated into `test_suite.py` (50 lines)
- ❌ `ur_admittance_utils.py` (332 lines) → eliminated complex inheritance
- ❌ `validate_node_transition.py` + `validate_notation.py` → merged into `validate.py` (30 lines)

### **Architecture Improvements**
- **Simplified inheritance**: Removed complex mixin patterns and base classes
- **Direct implementations**: Replaced abstractions with clear, direct code
- **Inlined utilities**: Matrix operations moved to point of use
- **Consolidated testing**: 6 test files → 2 comprehensive scripts
- **Streamlined documentation**: Verbose comments → essential explanations

### **Benefits Achieved**
1. **Maintainability**: 41% fewer lines to debug and understand
2. **Build Performance**: Faster compilation with fewer files and includes
3. **Code Clarity**: Direct implementations vs complex abstractions
4. **Developer Experience**: Easier onboarding with simplified structure
5. **Zero Functional Loss**: All algorithms and behaviors preserved identically

## Key Principles

1. **Real-time Safety**: No dynamic allocation in control loops
2. **Transform Efficiency**: Cache lookups, minimize TF calls
3. **Message Optimization**: Pre-allocate, use appropriate QoS
4. **Parameter Management**: Use generate_parameter_library
5. **Testing Strategy**: Unit → Integration → System tests
6. **Code Minimalism**: Simplicity over complexity, direct over abstract