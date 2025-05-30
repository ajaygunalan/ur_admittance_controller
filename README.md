# UR Admittance Node

> **Standalone force-compliant motion control node for Universal Robots - push the robot and it moves!**
>
> **Note**: This package has been refactored from a ROS2 Control controller to a standalone ROS2 node that interfaces directly with trajectory controllers.

## 📚 Table of Contents

- [🚀 Installation](#-installation)
- [🎮 Quick Start - Simulation](#-quick-start---simulation)
- [🤖 Real Robot Setup](#-real-robot-setup)
- [🎯 Control Modes](#-control-modes)
- [⚡ Safe Impedance Transition](#-safe-impedance-transition)
- [⚙️ Key Parameters](#️-key-parameters)
- [🧪 Testing](#-testing)
- [📍 Common Issues](#-common-issues)
- [🔧 Services & Topics](#-services--topics)
- [📐 Technical Documentation](#-technical-documentation)
- [📝 Additional Resources](#-additional-resources)

## 🚀 Installation

```bash
mkdir -p ~/ur_ws/src && cd ~/ur_ws/src
git clone https://github.com/ajaygunalan/ur_simulation_gz.git
git clone https://github.com/ajaygunalan/ur_admittance_controller.git

cd ~/ur_ws && rosdep install --from-paths src --ignore-src -r -y
colcon build && source install/setup.bash
```

## 🎮 Quick Start - Simulation

Launch UR5e + F/T sensor in Gazebo:
```
ros2 launch ur_simulation_gz ur_sim_control.launch.py
```

Launch admittance node (default is simulation mode):
```
ros2 launch ur_admittance_controller ur_admittance.launch.py
```

The node automatically uses topic mode and subscribes to Gazebo's `/wrist_ft_sensor` topic and publishes to `/scaled_joint_trajectory_controller/joint_trajectory`.

Apply force and watch robot move:
```
ros2 topic pub /wrist_ft_sensor geometry_msgs/WrenchStamped \
  "{header: {frame_id: 'ft_sensor_link'}, wrench: {force: {x: 10.0}}}" --once
```

**Gazebo Tip**: Press `F` key to enable force mode, then drag the robot!

## 🤖 Real Robot Setup

```bash
# Terminal 1: Connect to UR robot
ros2 launch ur_robot_driver ur_control.launch.py \
  robot_ip:=192.168.1.100 \
  launch_rviz:=false

# Verify the required downstream controller is active
ros2 controller list  # Should show "scaled_joint_trajectory_controller: active"
# If not active, activate it:
# ros2 controller set_state scaled_joint_trajectory_controller active

# Terminal 2: Launch admittance controller (hardware mode)
ros2 launch ur_admittance_controller ur_admittance.launch.py \
  use_sim:=false

# Terminal 3: On teach pendant
# 1. Load program: Installation > URCaps > External Control > Dashboard
# 2. Run program (press Play)
# 3. Robot now responds to external forces!
```

## 🎯 Control Modes

### Pure Admittance (Default)
Push robot → moves and **stays** there
```bash
# This is the default - K=0
ros2 param set /ur_admittance_controller admittance.stiffness [0,0,0,0,0,0]
```

### Impedance Mode  
Push robot → moves and **springs back**
```bash
# Robot returns to position like a spring
ros2 param set /ur_admittance_controller admittance.stiffness [100,100,100,10,10,10]
```

### Task-Specific Modes
```bash
# Surface following: Free XY, maintain Z height
ros2 param set /ur_admittance_controller admittance.stiffness [0,0,200,0,0,0]

# Assembly: XY centering, free Z insertion  
ros2 param set /ur_admittance_controller admittance.stiffness [50,50,0,5,5,0]
```

## ⚡ Safe Impedance Transition

**Problem**: Enabling stiffness (K>0) causes robot to jump to desired position  
**Solution**: Use the transition service

```bash
# Move robot to desired position first (manually or programmatically)
# Then set stiffness - it engages gradually
ros2 param set /admittance_node admittance.stiffness [100,100,100,10,10,10]
```

## 🔄 F/T Sensor Modes

The node operates in topic-based mode for F/T sensor data:

- **Simulation Mode (default)**: `ros2 launch ur_admittance_controller ur_admittance.launch.py`
  - Subscribes to `/wrist_ft_sensor` topic from Gazebo
  
- **Hardware Mode**: `ros2 launch ur_admittance_controller ur_admittance.launch.py use_sim:=false`
  - Subscribes to F/T sensor topic from real robot driver

## ⚙️ Key Parameters

| Parameter | Purpose | Default | Typical Range |
|-----------|---------|---------|---------------|
| `control_frequency` | Control loop rate | 0 Hz (max rate) | 0 (thread) or 100-1000 Hz |
| `admittance.mass` | Inertia (responsiveness) | [8,8,8,0.8,0.8,0.8] | 3-20 kg |
| `admittance.damping_ratio` | Stability | [0.8,0.8,0.8,0.8,0.8,0.8] | 0.7-1.2 |
| `admittance.stiffness` | Position control | [0,0,0,0,0,0] | 0-200 N/m |
| `admittance.min_motion_threshold` | Force deadband | 1.5 N | 0.5-5.0 N |
| `max_linear_velocity` | Safety limit | 0.5 m/s | 0.1-1.0 m/s |

## 🧪 Testing

```bash
# Check system status (uses ur_admittance_tests.py)
ros2 run ur_admittance_controller ur_admittance_tests.py status

# Test impedance modes  
ros2 run ur_admittance_controller ur_admittance_tests.py impedance

# Monitor continuously
ros2 run ur_admittance_controller ur_admittance_tests.py monitor
```

## 📍 Common Issues

### No Motion?
```bash
# Check force threshold
ros2 param get /admittance_node admittance.min_motion_threshold
# Lower it if needed
ros2 param set /admittance_node admittance.min_motion_threshold 0.5

# Verify F/T data
ros2 topic echo /wrist_ft_sensor
```

### Oscillating?
```bash
# Increase damping (>1.0 for overdamped)
ros2 param set /admittance_node admittance.damping_ratio [1.2,1.2,1.2,1.2,1.2,1.2]

# Or increase mass (slower response)
ros2 param set /admittance_node admittance.mass [15,15,15,1.5,1.5,1.5]
```

### Real Robot Issues
- Teach pendant must show "Program Running" with External Control active
- Check robot mode: `ros2 topic echo /ur_hardware_interface/robot_mode`
- Ensure network connectivity: `ping <robot_ip>`

## 🔧 Topics

**Published Topics:**
- `/scaled_joint_trajectory_controller/joint_trajectory` - Joint trajectory commands
- `/admittance_cartesian_velocity` - Current Cartesian velocity
- `/admittance_pose_error` - Error in impedance mode

**Subscribed Topics:**
- `/wrist_ft_sensor` - Force/torque sensor data
- `/joint_states` - Current joint positions

## 📐 Technical Documentation

### Core Architecture
- **Node Type**: Standalone ROS2 Node
- **Update Rate**: Maximum possible frequency by default (thread-based), or timer-based (1-1000 Hz)
- **Force Processing**: Transforms sensor data from any frame to base frame
- **Performance Optimized**: Pre-allocated messages, cached transforms, 20ms trajectory timing
- **Kinematics**: Plugin-based (supports KDL, MoveIt, custom implementations)
- **Output Interface**: Publishes to trajectory controllers

### Key Features
- **6-DOF Admittance Control**: Independent control for each Cartesian axis
- **Smooth Transitions**: Gradual stiffness engagement prevents sudden movements
- **Drift Prevention**: Automatic integrator reset when stationary
- **Safety Limits**: Configurable velocity and acceleration bounds

### Documentation Links
- [**Architecture Overview**](docs/ARCHITECTURE.md) - Detailed system design and implementation
- [**Notation Guide**](docs/NOTATION_GUIDE.md) - Coordinate frame conventions and naming
- [**Setup Guide**](docs/SETUP_GUIDE.md) - Controller selection rationale and configuration
- [**API Reference**](docs/API_REFERENCE.md) - Complete API documentation and examples

## 📝 Additional Resources

### Code Structure
```
ur_admittance_controller/
├── include/               # Header files
│   ├── admittance_controller.hpp
│   ├── admittance_types.hpp
│   ├── admittance_constants.hpp
│   └── matrix_utilities.hpp
├── src/                   # Implementation files
│   ├── admittance_controller.cpp    # Main controller logic
│   ├── realtime_computations.cpp    # RT-safe calculations
│   ├── sensor_interface.cpp         # F/T sensor handling
│   ├── controller_integration.cpp   # ROS2 Control interface
│   └── utilities.cpp                # Helper functions
├── config/                # Configuration
│   └── admittance_config.yaml       # Parameter definitions
├── launch/                # Launch files
│   └── ur_admittance.launch.py
└── scripts/               # Testing utilities
    ├── ur_admittance_tests.py
    ├── ur_admittance_utils.py
    └── validate_notation.py
```

### Theory Background
The UR5e has a built-in force/torque sensor at the tool flange. This controller implements the admittance control law:
```
M·ẍ + D·ẋ + K·(x - x_desired) = F_external
```
Where:
- M: Virtual mass matrix
- D: Damping matrix (computed from mass and damping ratio)
- K: Stiffness matrix (0 for pure admittance)
- F_external: Measured external forces/torques