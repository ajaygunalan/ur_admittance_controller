# UR Admittance Controller

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/en/rolling/)

## Package Purpose

This ROS 2 package implements force-compliant motion control for Universal Robots manipulators using the admittance control law: **M·a + D·v + K·x = F_ext**

The controller:
- Reads force/torque sensor data from the robot's TCP 
- Transforms forces from sensor frame (`tool0`) to base frame (`base_link`)
- Computes compliant Cartesian motion using configurable virtual mass/damping
- Outputs joint position references via controller chaining
- Publishes trajectory messages to `/scaled_joint_trajectory_controller/joint_trajectory`

For detailed implementation and architecture, see the [Architecture Document](ur_admittance_architecture.md).

## Key Features

- **Cartesian Admittance Control**: Converts external forces to compliant motion in task space
- **Dual Output Interface**: Exports reference positions AND publishes trajectory messages
- **Drift Prevention**: Automatic position reset when nearly stationary (< 1mm/s)
- **Real-time Optimized**: Pre-allocated memory, cached indices, non-blocking transforms
- **Safety Limits**: Cartesian velocity limits and URDF-based joint limits
- **Live Parameter Tuning**: Adjust admittance behavior without restarting

## Installation

### Dependencies

This package depends on:
- `ros2_control` - ROS 2 control framework
- `ros2_controllers` - Standard ROS 2 controllers
- `ur_robot_driver` - Universal Robots ROS 2 driver
- `ur_description` - UR robot descriptions
- `gazebo_ros2_control` - Gazebo integration (for simulation)

### Build

```bash
cd ~/ros2_ws/src
git clone https://github.com/ajaygunalan/ur_admittance_controller.git
cd ~/ros2_ws
colcon build --packages-select ur_admittance_controller
source install/setup.bash
```

## Quick Start

### Gazebo Simulation

**1. Start the Gazebo simulation with the UR robot:**
```bash
ros2 launch ur_simulation_gz ur_sim_control.launch.py
```

**2. Launch admittance control system:**
```bash

ros2 launch ur_admittance_controller ur_admittance_system.launch.py
```

### Real Robot

**1. Connect to your UR robot:**
```bash
# Launch complete system with robot IP
ros2 launch ur_admittance_controller ur_admittance_system.launch.py \
  use_sim:=false robot_ip:=192.168.1.100
```

**2. Or add to existing robot driver:**
```bash
# If robot driver already running
ros2 launch ur_admittance_controller ur_admittance_system.launch.py \
  use_sim:=false add_to_existing:=true
```

## Usage & Testing

### 1. Monitor System Status

The package includes a streamlined status monitoring tool to check the system health:

```bash
ros2 run ur_admittance_controller system_status.py
```

Sample output:
```
========== STATUS CHECK ==========

🎮 Controllers:
  ✅ scaled_joint_trajectory_controller
  ✅ joint_state_broadcaster
  ✅ force_torque_sensor_broadcaster
  ⚠️  ur_admittance_controller [inactive]
     → ros2 control set_controller_state ur_admittance_controller start

📡 Data flow:
  ✅ joint_states
  ✅ ft_sensor
  ❌ admittance_velocity

⚠️  Controllers OK, but missing data

📋 Quick commands:
  • ros2 control list_controllers
  • ros2 topic echo /ft_sensor_readings --once
  • ros2 param set /ur_admittance_controller mass.0 5.0
```

#### Status Monitoring Options

```bash
# Basic check
ros2 run ur_admittance_controller system_status.py

# Focus on one controller
ros2 run ur_admittance_controller system_status.py --ros-args -p focus_controller:=ur_admittance_controller

# With real-time logging
ros2 run ur_admittance_controller system_status.py --ros-args -p realtime_logging:=true
```

### 2. Test Force Response

```bash
# Apply 10N force in X direction
ros2 topic pub /ft_sensor_readings geometry_msgs/WrenchStamped \
  "{header: {frame_id: 'tool0'}, wrench: {force: {x: 10.0, y: 0.0, z: 0.0}}}" --once
```

### 3. Live Parameter Tuning

```bash
# Reduce mass for faster response
ros2 param set /ur_admittance_controller admittance.mass [5.0,5.0,5.0,0.5,0.5,0.5]

# Increase damping for stability
ros2 param set /ur_admittance_controller admittance.damping_ratio [0.9,0.9,0.9,0.9,0.9,0.9]

# Enable only XYZ translation
ros2 param set /ur_admittance_controller admittance.enabled_axes [true,true,true,false,false,false]
```

## Configuration

### Key Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `admittance.mass` | double[6] | [8.0, 8.0, 8.0, 0.8, 0.8, 0.8] | Virtual inertia [X,Y,Z,Rx,Ry,Rz] |
| `admittance.damping_ratio` | double[6] | [0.8, 0.8, 0.8, 0.8, 0.8, 0.8] | Damping coefficients |
| `admittance.stiffness` | double[6] | [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] | Position stiffness (0 = pure admittance) |
| `admittance.min_motion_threshold` | double | 1.5 | Force threshold to trigger motion (N/Nm) |
| `admittance.filter_coefficient` | double | 0.15 | Low-pass filter coefficient |
| `admittance.drift_reset_threshold` | double | 0.001 | Velocity threshold for drift prevention (m/s) |
| `max_linear_velocity` | double | 0.5 | Maximum Cartesian velocity (m/s) |
| `max_angular_velocity` | double | 1.0 | Maximum angular velocity (rad/s) |

### Example Configurations

**Surface Following (Light in Z)**
```yaml
admittance:
  mass: [2.0, 2.0, 0.5, 0.2, 0.2, 0.2]
  damping_ratio: [0.7, 0.7, 0.9, 0.8, 0.8, 0.8]
  enabled_axes: [true, true, true, false, false, false]
```

**Precise Assembly (High Stability)**
```yaml
admittance:
  mass: [10.0, 10.0, 10.0, 1.0, 1.0, 1.0]
  damping_ratio: [0.9, 0.9, 0.9, 0.9, 0.9, 0.9]
  min_motion_threshold: 0.5
```

## UR Robot Integration

### Universal Robots Specifications
- **6 DOF**: Full cartesian control capability
- **Force/Torque Sensing**: Built-in TCP sensor at tool flange
- **Update Rate**: 500Hz (e-Series)
- **Force Range**: ±200N typical
- **Torque Range**: ±10Nm typical
- **Supported Models**: UR3/UR3e, UR5/UR5e, UR10/UR10e, UR16e

### Controller Framework

This package implements a **chainable controller** using `controller_interface::ChainableControllerInterface`:

- **Direct Interface Access**: No message serialization overhead
- **500Hz Control Loop**: Deterministic real-time execution  
- **<0.5ms Latency**: 10-20x faster than action-based controllers
- **Zero-Copy**: Reference interfaces share memory with downstream controllers


The controller is designed to **chain directly with the `scaled_joint_trajectory_controller`** to ensure optimal, real-time performance. By leveraging interface-level chaining rather than ROS 2 action clients, we achieve industrial-grade responsiveness and reduced latency.
For a detailed technical comparison, refer to the [UR Controllers Reference](ur_controllers.md).



## Safety Features

1. **Joint Limits**: Automatically loaded from robot URDF
2. **Cartesian Velocity Limits**: Separate linear/angular limits
3. **Force Deadband**: Prevents motion from sensor noise
4. **Drift Prevention**: Automatic reset when stationary
5. **Transform Caching**: Non-blocking TF lookups
6. **Error Recovery**: Safe fallback on exceptions

## Troubleshooting

### No Motion Response
```bash
# Check force sensor data
ros2 topic echo /ft_sensor_readings

# Verify threshold
ros2 param get /ur_admittance_controller admittance.min_motion_threshold

# Check enabled axes
ros2 param get /ur_admittance_controller admittance.enabled_axes
```

### Unstable Motion
```bash
# Increase damping
ros2 param set /ur_admittance_controller admittance.damping_ratio [1.0,1.0,1.0,1.0,1.0,1.0]

# Increase mass
ros2 param set /ur_admittance_controller admittance.mass [15.0,15.0,15.0,1.5,1.5,1.5]
```

### Position Drift
```bash
# Check drift threshold (default 0.001 m/s)
ros2 param get /ur_admittance_controller admittance.drift_reset_threshold
```

## Advanced Features

### Impedance Mode
Enable position control with non-zero stiffness:
```bash
ros2 param set /ur_admittance_controller admittance.stiffness [100,100,100,10,10,10]
```

### Force Amplification
Reduce virtual mass for amplified response:
```bash
ros2 param set /ur_admittance_controller admittance.mass [0.5,0.5,0.5,0.05,0.05,0.05]
```

### Directional Compliance
Enable compliance only in specific directions:
```bash
# Z-axis only
ros2 param set /ur_admittance_controller admittance.enabled_axes [false,false,true,false,false,false]
```