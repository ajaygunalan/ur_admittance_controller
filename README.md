# UR Admittance Controller

> **Force-compliant motion control for Universal Robots - push the robot and it moves!**

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

```bash
# Terminal 1: Gazebo with UR5e + F/T sensor 
ros2 launch ur_simulation_gz ur_sim_control.launch.py

# Terminal 2: Launch admittance controller
ros2 launch ur_admittance_controller ur_admittance.launch.py

# Terminal 3: Apply force and watch robot move
ros2 topic pub /ft_sensor_readings geometry_msgs/WrenchStamped \
  "{header: {frame_id: 'tool0'}, wrench: {force: {x: 10.0}}}" --once
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

# Terminal 2: Launch admittance controller
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
# Then safely enable impedance mode:
ros2 service call /ur_admittance_controller/move_to_start_pose std_srvs/srv/Trigger

# Now set stiffness - it engages gradually
ros2 param set /ur_admittance_controller admittance.stiffness [100,100,100,10,10,10]
```

## ⚙️ Key Parameters

| Parameter | Purpose | Default | Typical Range |
|-----------|---------|---------|---------------|
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
ros2 param get /ur_admittance_controller admittance.min_motion_threshold
# Lower it if needed
ros2 param set /ur_admittance_controller admittance.min_motion_threshold 0.5

# Verify F/T data
ros2 topic echo /ft_sensor_readings
```

### Oscillating?
```bash
# Increase damping (>1.0 for overdamped)
ros2 param set /ur_admittance_controller admittance.damping_ratio [1.2,1.2,1.2,1.2,1.2,1.2]

# Or increase mass (slower response)
ros2 param set /ur_admittance_controller admittance.mass [15,15,15,1.5,1.5,1.5]
```

### Real Robot Issues
- Teach pendant must show "Program Running" with External Control active
- Check robot mode: `ros2 topic echo /ur_hardware_interface/robot_mode`
- Ensure network connectivity: `ping <robot_ip>`

## 🔧 Services & Topics

**Services:**
- `/ur_admittance_controller/reset_desired_pose` - Set desired = current pose
- `/ur_admittance_controller/move_to_start_pose` - Safe impedance transition

**Published Topics:**
- `/ur_admittance_controller/cartesian_velocity_command` - Current velocity
- `/ur_admittance_controller/pose_error` - Error in impedance mode
- `/ur_admittance_controller/current_pose` - Current TCP pose
- `/ur_admittance_controller/desired_pose` - Target pose (impedance mode)

**Subscribed Topics:**
- `/ur_admittance_controller/set_desired_pose` - Set target pose

## 📐 Technical Documentation

### Core Architecture
- **Controller Type**: ChainableControllerInterface (ROS2 Control)
- **Update Rate**: Determined by controller_manager (typically 200-500 Hz)
- **Force Processing**: Transforms sensor data from any frame to base frame
- **Real-time Safe**: Pre-allocated memory, cached transforms, lock-free publishing
- **Kinematics**: Plugin-based (supports KDL, MoveIt, custom implementations)

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