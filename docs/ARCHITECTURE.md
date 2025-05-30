# UR Admittance Node Architecture

**Last Updated**: January 2025  
**Package Version**: 2.0.0  
**ROS2 Version**: Humble/Iron/Jazzy/Rolling

## Why This Node Exists

Universal Robots only accept **position/velocity commands**, not force/torque commands. But many tasks require force control:
- **Assembly**: Insert parts without jamming
- **Polishing**: Maintain constant contact force  
- **Human collaboration**: Safe physical interaction

**Solution**: Admittance control - convert forces into compliant motion via a standalone ROS2 node.

## Core Concept

```
External Force → Virtual Mass-Spring-Damper → Motion
     F(t)      →    M·ẍ + D·ẋ + K·x = F   →   Δx(t)
```

The robot behaves as if it has adjustable:
- **Mass (M)**: How quickly it responds to forces
- **Damping (D)**: How smoothly it moves (no oscillations)  
- **Stiffness (K)**: Whether it returns to position (spring behavior)

## System Architecture

```
┌─────────────┐     ┌──────────────────┐     ┌─────────────────────┐     ┌───────┐
│ F/T Sensor  │────►│ Admittance Node  │────►│ Trajectory Controller│────►│ Robot │
└─────────────┘     └──────────────────┘     └─────────────────────┘     └───────┘
    Wrench             Joint Trajectory         Position Commands          Motion
    Topic              Message @ Max Hz         @ 125Hz streaming      
```

### Standalone Node Design

The admittance controller is implemented as a **standalone ROS2 node** that:
- **Subscribes** to force/torque sensor data via topics
- **Computes** admittance dynamics in a real-time safe manner
- **Publishes** joint trajectory commands to the robot's trajectory controller

Key advantages:
- **Simplicity**: No complex ROS2 Control integration
- **Flexibility**: Works with any trajectory controller
- **Performance**: Thread-based control loop for maximum frequency
- **Portability**: Easy to use with different robot drivers

## Data Flow

### 1. Force/Torque Input
```
/wrist_ft_sensor (geometry_msgs/WrenchStamped)
    ↓
Transform to base frame
    ↓
Low-pass filtering
```

### 2. Admittance Computation
```
Filtered Force F
    ↓
Admittance Law: M·ẍ + D·ẋ + K·(x-x₀) = F
    ↓
Integrate to get Cartesian velocity
    ↓
Safety limits & drift prevention
```

### 3. Motion Output
```
Cartesian velocity
    ↓
Inverse kinematics (plugin-based)
    ↓
Joint velocities
    ↓
/scaled_joint_trajectory_controller/joint_trajectory
```

## Node Implementation

### Core Components

1. **admittance_node.cpp**: Main node with control loop
   - Thread-based execution for maximum frequency
   - Pre-allocated messages for performance
   - Configurable timer-based mode available

2. **admittance_computations.cpp**: Dynamics calculations
   - Real-time safe matrix operations
   - Numerical integration (Euler method)
   - Safety limiting and filtering

3. **sensor_handling.cpp**: F/T data processing
   - Transform management
   - Data validation and timeout handling
   - Frame conversion to base coordinates

4. **node_integration.cpp**: ROS2 interfaces
   - Parameter management via generate_parameter_library
   - Dynamic reconfiguration support
   - Service interfaces for mode changes

### Control Loop Design

```cpp
// Thread-based control loop (default)
void controlThreadFunction() {
    rclcpp::Rate rate(1000);  // Limit max rate if needed
    while (rclcpp::ok()) {
        controlLoop();  // Main computation
        executor.spin_some(0);  // Process callbacks
    }
}
```

When `control_frequency = 0.0` (default), the node runs as fast as possible using a dedicated thread.

## Parameter System

Parameters are defined in `config/admittance_config.yaml` and auto-generated into a type-safe C++ interface:

- **Dynamics**: mass, damping_ratio, stiffness
- **Safety**: max_linear_velocity, max_angular_velocity  
- **Performance**: control_frequency, trajectory timing
- **Sensor**: topic names, timeout, transform settings

## Kinematics Plugin System

The node uses a plugin-based kinematics interface:
```
Plugin Interface (kinematics_interface)
    ↓
KDL Implementation (default)
MoveIt Implementation (optional)
Custom Implementation (user-provided)
```

This allows switching between different IK solvers without recompiling.

## Performance Optimizations

1. **Pre-allocated Messages**: All ROS messages allocated once
2. **Cached Transforms**: TF lookups minimized  
3. **Thread-based Loop**: No timer overhead
4. **Direct Publishing**: No intermediate buffers
5. **Optimized Trajectory Timing**: 20ms time_from_start

## Safety Features

- **Velocity Limiting**: Hard limits on Cartesian space
- **Timeout Handling**: Automatic stop on stale data
- **Drift Prevention**: Position reset when stationary
- **Smooth Transitions**: Gradual stiffness engagement
- **Emergency Stop**: Via service interface

## Integration with UR Robots

### Simulation Mode
```
Gazebo F/T Plugin → /wrist_ft_sensor → Admittance Node → Joint Trajectory
```

### Real Robot Mode  
```
UR F/T Sensor → Robot Driver → /wrist_ft_sensor → Admittance Node → Joint Trajectory
```

The node automatically adapts based on the `use_sim` parameter.