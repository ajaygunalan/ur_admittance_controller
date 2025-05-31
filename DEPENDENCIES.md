# DEPENDENCIES.md

## 📋 **ur_admittance_controller Package Dependencies**

> **Complete dependency mapping for the 6-DOF admittance control system for Universal Robots**

### **🏗️ System Architecture Overview**

The `ur_admittance_controller` operates within a complete robotics ecosystem that provides robot descriptions, simulation capabilities, and control infrastructure.

```
[ur_description] ──→ [Robot URDF] ──→ [ur_admittance_controller]
                                            ↓
[ur_simulation_gz] ──→ [F/T Sensor Data] ──→ [Admittance Algorithm]
                                            ↓
[Gazebo Harmonic] ──→ [Physics Simulation] ──→ [scaled_joint_trajectory_controller]
                                            ↓
[ROS2 Jazzy] ──→ [Core Infrastructure] ──→ [Robot Motion]
```

### **🎯 Core Ecosystem Dependencies**

#### **1. Universal Robots Ecosystem**

| **Package** | **Version** | **Purpose** | **Usage in Package** |
|-------------|-------------|-------------|---------------------|
| **ur_description** | System | Official UR5e URDF specifications | Joint limits loaded via `/robot_description` topic |
| **ur_simulation_gz** | 2.2.0+ | Gazebo simulation and sensors | F/T sensor data provider via `/wrist_ft_sensor` |
| **ur_controllers** | System | UR-specific trajectory controllers | Target: `scaled_joint_trajectory_controller` |

**How They're Used:**
- **ur_description**: Provides official UR5e joint limits via `/robot_description` topic → parsed in `loadJointLimitsFromURDF()` (admittance_node.cpp:236)
- **ur_simulation_gz**: Publishes F/T sensor data → consumed in `wrenchCallback()` (admittance_node.cpp:134)
- **ur_controllers**: Receives trajectory commands → published to `/scaled_joint_trajectory_controller/joint_trajectory` (admittance_node.cpp:61)

#### **2. ROS2 Jazzy Infrastructure**

| **Component** | **Purpose** | **Implementation Location** |
|---------------|-------------|----------------------------|
| **rclcpp** | Core ROS2 C++ client library | Main node class: `admittance_node.hpp:37` |
| **tf2/tf2_ros/tf2_eigen** | Transform management | Transform lookups: `sensor_handling.cpp:60,93` |
| **trajectory_msgs** | Joint trajectory communication | Pre-allocated message: `admittance_node.cpp:85` |
| **sensor_msgs** | Joint state and sensor data | Joint state callback: `admittance_node.cpp:163` |
| **geometry_msgs** | Pose and wrench messages | Wrench callback: `admittance_node.cpp:134` |

#### **3. Gazebo Harmonic Simulation**

| **Component** | **Integration** | **Data Flow** |
|---------------|-----------------|---------------|
| **gz_ros2_control** | Robot control interface | Provides joint states → `joint_state_sub_` |
| **ros_gz_bridge** | Gazebo ↔ ROS2 communication | F/T sensor bridge → `/wrist_ft_sensor` topic |
| **gz_physics** | Physics simulation | Force/torque generation → sensor measurements |

### **📦 Build and Development Dependencies**

#### **Build Tools** (CMakeLists.txt:28-51)
```yaml
Build System:      ament_cmake
C++ Standard:      C++17 (cmake minimum 3.16)
Parameter Gen:     generate_parameter_library  # Auto-generates parameter headers
Plugin System:     pluginlib                   # Kinematics plugin loading
```

#### **Core ROS2 Libraries** (package.xml:37-54)
```yaml
ROS2 Core:
  - rclcpp                    # Node implementation
  - std_msgs                  # Basic message types
  - geometry_msgs             # Poses, twists, wrenches
  - sensor_msgs               # Joint states, F/T data
  - trajectory_msgs           # Joint trajectory commands

Transform System:
  - tf2                       # Core transform library
  - tf2_ros                   # ROS2 transform integration
  - tf2_eigen                 # Eigen matrix conversions

Robot Model:
  - urdf                      # Robot description parsing
  - kdl_parser                # KDL kinematics parsing
```

#### **Mathematical Libraries**
```yaml
Linear Algebra:    Eigen3 (system dependency)      # 6D vectors, matrices
Kinematics:        kinematics_interface            # Inverse kinematics
                   kinematics_interface_kdl        # KDL implementation
```

### **🔄 Runtime Data Flow Dependencies**

#### **Input Topics** (Dependencies → ur_admittance_controller)
```yaml
/robot_description:    std_msgs/String              # FROM: ur_description
                      → loadJointLimitsFromURDF()   # admittance_node.cpp:236

/joint_states:        sensor_msgs/JointState        # FROM: ur_simulation_gz
                      → jointStateCallback()        # admittance_node.cpp:163

/wrist_ft_sensor:     geometry_msgs/WrenchStamped  # FROM: ur_simulation_gz
                      → wrenchCallback()            # admittance_node.cpp:134
```

#### **Output Topics** (ur_admittance_controller → Dependencies)
```yaml
/scaled_joint_trajectory_controller/joint_trajectory:  # TO: ur_controllers
    trajectory_msgs/JointTrajectory                     # admittance_node.cpp:61
    → Position + velocity commands for smooth motion

/admittance_cartesian_velocity:   geometry_msgs/Twist  # Monitoring output
/admittance_pose_error:          geometry_msgs/Twist   # Debugging output
```

#### **Transform Dependencies** (tf2 lookups)
```yaml
base_link ← ft_sensor_link:    # F/T sensor transform
    Used in: transformWrench()             # sensor_handling.cpp:60
    Purpose: Convert sensor forces to robot base frame

base_link ← tool0:            # End-effector transform  
    Used in: getCurrentEndEffectorPose()  # sensor_handling.cpp:93
    Purpose: Current robot pose for admittance control
```

### **🔧 Configuration Dependencies**

#### **Parameter System** (generate_parameter_library)
```yaml
Source:        config/admittance_config.yaml    # Parameter definitions
Generated:     ur_admittance_controller_parameters.hpp  # Auto-generated header
Usage:         param_listener_->get_params()    # admittance_node.cpp:14
```

#### **Plugin Configuration** (kinematics_interface)
```yaml
Plugin Package: "kinematics_interface"                    # config: line 79
Plugin Name:    "kinematics_interface_kdl/KinematicsInterfaceKDL"  # config: line 84
Usage:          Cartesian → joint space conversion       # admittance_computations.cpp:424
```

### **🚀 Launch System Dependencies**

#### **ur_admittance.launch.py** (Single Node Launch)
```python
Package:     ur_admittance_controller
Executable:  admittance_node                    # Built from src/admittance_node.cpp
Config:      config/admittance_config.yaml      # Parameter file
```

#### **Integration with ur_simulation_gz**
```python
# Typical launch sequence:
1. ur_simulation_gz/launch/ur_sim_control.launch.py
   ├── Starts Gazebo with UR5e model
   ├── Loads controllers (scaled_joint_trajectory_controller)
   ├── Publishes /robot_description
   └── Provides /wrist_ft_sensor data

2. ur_admittance_controller/launch/ur_admittance.launch.py  
   ├── Starts admittance_node
   ├── Loads admittance parameters
   └── Connects to existing ecosystem
```

### **🧪 Testing Dependencies** (package.xml:60-62)
```yaml
Test Framework:     ament_cmake_gmock        # C++ testing
Control Testing:    controller_manager       # Controller integration tests  
Test Assets:        ros2_control_test_assets # Test configurations
```

### **💡 Key Dependency Patterns**

#### **1. Official UR Integration**
- **Joint limits**: Dynamically loaded from official `ur_description` URDF
- **Controller interface**: Direct integration with `scaled_joint_trajectory_controller`
- **Fallback system**: Hardcoded UR5e limits if official data unavailable

#### **2. Real-time Performance**
- **Transform caching**: Direct tf2 lookups with 50ms timeout
- **Message pre-allocation**: Pre-allocated trajectory messages for 500Hz control
- **Thread-based control**: Dedicated control thread for consistent timing

#### **3. Ecosystem Compatibility**
- **Standard interfaces**: Uses ROS2 standard message types
- **Plugin architecture**: Kinematics via pluginlib for flexibility
- **Parameter system**: Auto-generated parameters for type safety

### **📊 Dependency Summary**

| **Category** | **Dependencies** | **Purpose** |
|--------------|------------------|-------------|
| **UR Ecosystem** | ur_description, ur_simulation_gz, ur_controllers | Robot model, simulation, control |
| **ROS2 Core** | rclcpp, tf2, standard message packages | Infrastructure and communication |
| **Mathematics** | Eigen3, kinematics_interface | Linear algebra and robot kinematics |
| **Simulation** | Gazebo Harmonic, gz_ros2_control | Physics simulation and sensor data |
| **Build Tools** | ament_cmake, generate_parameter_library | Build system and code generation |

### **🔗 Critical Integration Points**

1. **Robot Description**: Must receive `/robot_description` from `ur_description` for joint limits
2. **F/T Sensor**: Requires `/wrist_ft_sensor` from `ur_simulation_gz` for force data  
3. **Joint Control**: Must connect to `scaled_joint_trajectory_controller` for motion commands
4. **Transform Tree**: Depends on tf2 tree provided by robot description and simulation

### **⚙️ Version Compatibility**

```yaml
ROS2 Distribution:    Jazzy (tested and optimized)
Gazebo Version:       Harmonic (official support)
UR Packages:          Latest (ur_robot_driver v2.0+)
C++ Standard:         C++17 (required for kinematics_interface)
```

This dependency structure ensures seamless integration with the Universal Robots ecosystem while maintaining compatibility with standard ROS2 patterns and tools.