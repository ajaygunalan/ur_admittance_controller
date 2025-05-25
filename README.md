# UR Admittance Controller

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/en/rolling/)
[![Build Status](https://img.shields.io/badge/Build-Passing-success)](https://github.com/ajaygunalan/ur_admittance_controller)

> **ROS2 package for task-space (cartesian) force-compliant motion control (admittance control) for Universal Robots manipulators**



```
External Force → Compliant Motion
     10N push  →  Smooth movement in force direction
```

**Core Equation**: `M·a + D·v + K·x = F_ext`
- **M**: Virtual mass (inertia) - controls responsiveness
- **D**: Damping - controls stability  
- **K**: Stiffness - controls position behavior:
  - **K=0** (Pure Admittance Mode): Robot moves freely and stays where pushed
  - **K>0** (Impedance Mode): Robot returns to desired position like a spring
  - Set different K values per axis for mixed behavior!
- **F_ext**: External forces from F/T sensor

### Operating Modes

#### Pure Admittance Mode (K=0)
Robot acts like a free-floating mass. Push it, and it stays at the new position.
```bash
# Default configuration
ros2 param set /ur_admittance_controller admittance.stiffness [0,0,0,0,0,0]
```

#### Impedance Mode (K>0)
Robot acts like a mass-spring-damper system. Push it, and it springs back to the desired position.
```bash
# Full 6DOF impedance control
ros2 param set /ur_admittance_controller admittance.stiffness [100,100,100,10,10,10]
```

#### Mixed Mode
Different behavior per axis - incredibly powerful for task-specific compliance!
```bash
# XY compliant (stays where pushed), Z returns to height
ros2 param set /ur_admittance_controller admittance.stiffness [0,0,200,0,0,0]
```

## 🚀 Quick Start

### Prerequisites
- ROS2 Jazzy/Humble
- Universal Robots ROS2 driver
- Gazebo (for simulation)

### 1. Installation

```bash
# Create workspace and clone
mkdir -p ~/ur_ws/src && cd ~/ur_ws/src
git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation.git
git clone https://github.com/ajaygunalan/ur_admittance_controller.git

# Install dependencies
cd ~/ur_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --packages-select ur_admittance_controller
source install/setup.bash
```

### 2. Simulation Setup

**Launch Gazebo with UR5e + F/T sensor:**
```bash
ros2 launch ur_simulation_gz ur_sim_control.launch.py \
  description_package:=ur_admittance_controller \
  description_file:=ur5e_admittance_sim.urdf.xacro
```

**Start admittance control:**
```bash
ros2 launch ur_admittance_controller ur_admittance_system.launch.py
```

### 3. Test Force Response

```bash
# Apply 10N force in X direction
ros2 topic pub /ft_sensor_readings geometry_msgs/WrenchStamped \
  "{header: {frame_id: 'tool0'}, wrench: {force: {x: 10.0}}}" --once

# Monitor robot motion
ros2 topic echo /ur_admittance_controller/cartesian_velocity_command
```

### 4. Real Robot

```bash
# Connect to UR robot (replace with your robot's IP)
ros2 launch ur_admittance_controller ur_admittance_system.launch.py \
  use_sim:=false robot_ip:=192.168.1.100
```

## 🎮 Interactive Testing in Gazebo

1. **Enable Force Mode**: Press `F` key in Gazebo
2. **Drag Robot**: Click and drag the end-effector  
3. **Observe**: Robot moves compliantly in drag direction
4. **Tune Live**: Adjust parameters while running (see below)

## ⚙️ Live Parameter Tuning

Adjust behavior in real-time without restarting:

```bash
# Make robot more responsive (lower mass)
ros2 param set /ur_admittance_controller admittance.mass [5.0,5.0,5.0,0.5,0.5,0.5]

# Increase stability (higher damping)
ros2 param set /ur_admittance_controller admittance.damping_ratio [0.9,0.9,0.9,0.9,0.9,0.9]

# Enable only vertical compliance (Z-axis)
ros2 param set /ur_admittance_controller admittance.enabled_axes [false,false,true,false,false,false]

# Reduce sensitivity (higher force threshold)
ros2 param set /ur_admittance_controller admittance.min_motion_threshold 3.0
```

## 🎯 Impedance Mode Examples

### Basic Position Control
Make the robot return to its starting position when pushed:
```bash
# Set moderate stiffness on all axes
ros2 param set /ur_admittance_controller admittance.stiffness [100,100,100,10,10,10]

# Test: Push robot and watch it spring back
ros2 topic pub /ft_sensor_readings geometry_msgs/WrenchStamped \
  "{wrench: {force: {x: 20.0}}}" --once
```

### Task-Specific Configurations

#### Surface Contact (Polishing/Sanding)
Maintain constant Z-height while allowing XY movement:
```bash
ros2 param set /ur_admittance_controller admittance.mass [3,3,1,0.3,0.3,0.3]
ros2 param set /ur_admittance_controller admittance.stiffness [0,0,150,0,0,0]
```

#### Peg-in-Hole Assembly
Gentle XY centering with free Z insertion:
```bash
ros2 param set /ur_admittance_controller admittance.stiffness [50,50,0,5,5,0]
```

#### Collaborative Handover
Human can move robot, but it returns to position when released:
```bash
ros2 param set /ur_admittance_controller admittance.stiffness [80,80,80,8,8,8]
ros2 param set /ur_admittance_controller admittance.damping_ratio [1.0,1.0,1.0,1.0,1.0,1.0]
```

### Understanding Stiffness Values

- 0 N/m: No position control (pure admittance)
- 10-50 N/m: Very gentle return force
- 50-200 N/m: Moderate spring feeling
- 200-500 N/m: Strong return force
- 500+ N/m: Very stiff, hard to move

### Monitoring Impedance Behavior
```bash
# Watch pose error (only non-zero with K>0)
ros2 topic echo /ur_admittance_controller/pose_error

# See return velocity after releasing force
ros2 topic echo /ur_admittance_controller/cartesian_velocity_command
```

## 📊 System Monitoring

**Built-in status checker:**
```bash
ros2 run ur_admittance_controller system_status.py
```

**Sample output:**
```
========== STATUS CHECK ==========

🎮 Controllers:
  ✅ scaled_joint_trajectory_controller
  ✅ joint_state_broadcaster  
  ✅ force_torque_sensor_broadcaster
  ✅ ur_admittance_controller

📡 Data flow:
  ✅ joint_states
  ✅ ft_sensor
  ✅ admittance_velocity

✅ SYSTEM READY
```

## 🚀 Safe Startup Procedure

### Automated Safe Startup

For moving from home position to working position safely:

```bash
# 1. Start robot (simulation or real)
ros2 launch ur_simulation_gz ur_sim_control.launch.py

# 2. Launch admittance controller
ros2 launch ur_admittance_controller ur_admittance_system.launch.py

# 3. Execute safe startup to working position
ros2 launch ur_admittance_controller safe_startup.launch.py \
  target_pose:="[0.5, 0.3, 0.4, 0.0, 1.57, 0.0]" \
  impedance_stiffness:="[100, 100, 100, 10, 10, 10]"
```

The startup sequence will:

1. Ensure pure admittance mode (K=0)
2. Move robot smoothly to target pose (5 seconds)
3. Set desired pose at target
4. Gradually enable impedance control (2 seconds)

### Manual Safe Startup

If you prefer manual control:

```bash
# 1. Ensure pure admittance
ros2 param set /ur_admittance_controller admittance.stiffness [0,0,0,0,0,0]

# 2. Move to working position
ros2 service call /ur_admittance_controller/move_to_start_pose \
  std_srvs/srv/Trigger "{}"

# 3. Wait for movement completion (5 seconds)

# 4. Enable impedance
ros2 param set /ur_admittance_controller admittance.stiffness [100,100,100,10,10,10]
```
  ✅ admittance_velocity

✅ SYSTEM READY
```

## 🔧 Configuration

### Key Parameters

| Parameter | Purpose | Default | Range |
|-----------|---------|---------|--------|
| `admittance.mass` | Virtual inertia [X,Y,Z,Rx,Ry,Rz] | `[8,8,8,0.8,0.8,0.8]` | `(0.1,100]` |
| `admittance.damping_ratio` | Stability control | `[0.8,0.8,0.8,0.8,0.8,0.8]` | `[0.1,2.0]` |
| `admittance.stiffness` | Position control (0=pure admittance) | `[0,0,0,0,0,0]` | `[0,2000]` |
| `admittance.enabled_axes` | Enable/disable each DOF | `[true×6]` | `bool[6]` |
| `admittance.min_motion_threshold` | Force deadband (N/Nm) | `1.5` | `[0.1,10]` |
| `max_linear_velocity` | Safety limit (m/s) | `0.5` | `[0.1,1.0]` |

### Application-Specific Configs

**Surface Following** (light contact):
```yaml
admittance:
  mass: [3.0, 3.0, 1.0, 0.3, 0.3, 0.3]  # Light in Z
  damping_ratio: [0.7, 0.7, 0.9, 0.8, 0.8, 0.8]
  enabled_axes: [true, true, true, false, false, false]  # XYZ only
```

**Precise Assembly** (stable):
```yaml
admittance:
  mass: [15.0, 15.0, 15.0, 1.5, 1.5, 1.5]  # High mass
  damping_ratio: [0.95, 0.95, 0.95, 0.95, 0.95, 0.95]  # High damping
  min_motion_threshold: 0.5  # Very sensitive
```

**Human Collaboration** (safe):
```yaml
admittance:
  mass: [5.0, 5.0, 5.0, 0.5, 0.5, 0.5]  # Responsive
  max_linear_velocity: 0.2  # Slow and safe
  max_angular_velocity: 0.5
```

## 🏗️ Architecture

### Controller Chain
```
F/T Sensor → Admittance Controller → Joint Trajectory Controller → Robot Hardware
   (Forces)     (Joint References)       (Motor Commands)           (Motion)
```

### Data Flow
```
tool0 frame → Transform → Filter → Admittance → Kinematics → Joint Limits → Output
  [Forces]     [base_link]  [smooth]   [velocity]   [joint Δ]    [safety]    [refs]
```

**Key Features:**
- **500Hz Control**: Real-time deterministic execution
- **<1ms Latency**: Interface-level chaining, no message overhead  
- **Memory Safe**: Pre-allocated vectors, no dynamic allocation
- **Transform Caching**: Non-blocking TF lookups for RT safety
- **Real-Time Publishing**: Lock-free publishers using `realtime_tools`

For detailed technical information, see [Architecture Document](ur_admittance_architecture.md).

## 🛡️ Safety Features

- ✅ **Joint Limits**: Auto-loaded from robot URDF
- ✅ **Velocity Limits**: Separate linear/angular Cartesian limits  
- ✅ **Force Deadband**: Prevents motion from sensor noise
- ✅ **Safe Startup**: Gradual stiffness engagement and trajectory-based movements
- ✅ **Drift Prevention**: Auto-reset when stationary (<1mm/s)
- ✅ **Exception Recovery**: Safe fallbacks on any error
- ✅ **Real-Time Safe**: No blocking operations in control loop
- ✅ **Lock-Free Publishing**: Non-blocking data output

## 🔄 Final Testing Procedure

### Simulation Test:

```bash
# Terminal 1
ros2 launch ur_simulation_gz ur_sim_control.launch.py

# Terminal 2
ros2 launch ur_admittance_controller ur_admittance_system.launch.py

# Terminal 3
ros2 launch ur_admittance_controller safe_startup.launch.py

# Terminal 4 (monitoring)
ros2 topic echo /ur_admittance_controller/pose_error
```

### Real Robot Test (with reduced speeds):

```bash
# Same as above but with:
ros2 launch ur_admittance_controller safe_startup.launch.py \
  trajectory_duration:=10.0 \
  impedance_stiffness:="[50, 50, 50, 5, 5, 5]"
```

## 🔍 Troubleshooting

### No Motion Response
```bash
# Check force sensor
ros2 topic echo /ft_sensor_readings --once

# Verify controller is active
ros2 control list_controllers

# Check force threshold
ros2 param get /ur_admittance_controller admittance.min_motion_threshold
```

### Unstable/Oscillating Motion
```bash
# Increase damping (more stable)
ros2 param set /ur_admittance_controller admittance.damping_ratio [1.0,1.0,1.0,1.0,1.0,1.0]

# Increase mass (slower response)  
ros2 param set /ur_admittance_controller admittance.mass [15.0,15.0,15.0,1.5,1.5,1.5]
```

### Position Drift
```bash
# Check drift threshold (lower = more sensitive)
ros2 param get /ur_admittance_controller admittance.drift_reset_threshold

# Monitor velocity for drift detection
ros2 topic echo /ur_admittance_controller/cartesian_velocity_command
```

## 🤝 Universal Robots Integration


### UR5e Specifications

- 6 DOF, 5kg payload capacity, 850 mm maximum reach
- Sensing/actuation capabilities:
  - Joint position feedback
  - Joint velocity feedback
  - No direct joint torque control (only estimates via motor current)
  - 6-axis force/torque sensing at the TCP


the defaul model is `UR5e` but can work with any UR robot

### Why This Implementation?

We use the default `scaled_joint_trajectory_controller` for safety and reliability. For detailed information about UR robot controllers and why we stciked with `default` controller and why we choose cartesian (task-space) admittance controller , see the [UR Controllers Reference](docs/ur_controllers.md).


## 🧪 Testing & Validation

### Simulation Testing
```bash
# Complete system test in Gazebo
ros2 launch ur_admittance_controller ur_admittance_system.launch.py

# Apply test forces and verify motion
ros2 topic pub /ft_sensor_readings geometry_msgs/WrenchStamped \
  "{wrench: {force: {x: 5, y: 0, z: 0}}}" --once
```

### Real Robot Validation
```bash
# Gentle hand-guided testing
# 1. Start admittance control
# 2. Gently push/pull end-effector
# 3. Verify smooth, proportional motion
# 4. Test emergency stop functionality
```

## 🤝 Contributing

Contributions welcome! Please read our contributing guidelines and submit pull requests for improvements.

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

