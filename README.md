# ur_admittance_controller

> **6-DOF force-compliant motion control for Universal Robots - push the robot and it moves!**

## Overview
Admittance control for Universal Robots using **M·ẍ + D·ẋ + K·x = F**. Prototype-first design philosophy: trust the system, minimize defensive code, fail fast.

Key features:
- **100Hz control** with 487Hz joint state feedback
- **Real-time M/K/D tuning** via ros2 param
- **Dual-node architecture**: wrench preprocessing + admittance control
- **All UR robots supported**: UR3/5e/10/16/20

## Quick Start

### Installation
```bash
mkdir -p ~/ur_ws/src && cd ~/ur_ws/src
git clone https://github.com/ajaygunalan/ur_simulation_gz.git
git clone https://github.com/ajaygunalan/ur_admittance_controller.git

cd ~/ur_ws && rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select ur_simulation_gz && source install/setup.bash
colcon build --packages-select ur_admittance_controller && source install/setup.bash
```


### Running
```bash
# Terminal 1: Launch robot
ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur5e

# Terminal 2: Initialize and run admittance control
ros2 run ur_admittance_controller init_robot
ros2 launch ur_admittance_controller ur_admittance.launch.py

# Terminal 3: Test with force command
ros2 topic pub /wrench_tcp_base_raw geometry_msgs/WrenchStamped \
  "{header: {frame_id: 'base_link'}, wrench: {force: {x: 10.0}}}" --once
```

### Hardware
```bash
# Real robot (set use_sim_time:=false)
ros2 launch ur_robot_driver ur_control.launch.py robot_ip:=192.168.1.100
ros2 launch ur_admittance_controller ur_admittance.launch.py use_sim_time:=false
```

### Running Nodes Directly
```bash
# Run individual nodes (useful for debugging)
ros2 run ur_admittance_controller admittance_node

# With debug logging:
ros2 run ur_admittance_controller admittance_node --ros-args --log-level debug

# With simulation time (if using Gazebo):
ros2 run ur_admittance_controller admittance_node --ros-args -p use_sim_time:=true

# Run wrench node separately:
ros2 run ur_admittance_controller wrench_node --ros-args --log-level debug
```

## Architecture

```
[F/T Sensor] → /wrench_tcp_base_raw → [Wrench Node] → /wrench_tcp_base → [Admittance Node] → /forward_velocity_controller/commands → [Robot]
                                          ↓                                      ↓
                                   (Bias + Filter)                       (M·ẍ + D·ẋ + K·x = F)
```

### Components

**Wrench Node**: F/T preprocessing
- Auto bias removal on startup
- EMA filter (hardcoded α=0.8)
- Deadband threshold (1.5 N/Nm)
- Service: `~/reset_bias`

**Admittance Node**: 6-DOF control at 100Hz
- KDL kinematics (WDLS solver, λ=0.01)
- Immediate stop on IK failure
- Hard workspace limits only
- Dynamic parameters: M/K/D only

### Topics

**Inputs**:
- `/wrench_tcp_base_raw` → wrench_node
- `/wrench_tcp_base` → admittance_node  
- `/joint_states` → admittance_node (487Hz)
- `/admittance_node/desired_pose` → admittance_node (optional)

**Outputs**:
- `/wrench_tcp_base` ← wrench_node
- `/forward_velocity_controller/commands` ← admittance_node (100Hz)

## Parameters

### Dynamic Parameters (Runtime Tuning)
```bash
# Virtual mass [kg, kg·m²] - default: [2,2,2,2,2,2]
ros2 param set /admittance_node admittance.mass "[1.0,1.0,1.0,0.5,0.5,0.5]"

# Damping [Ns/m, Nms/rad] - default: [12,12,12,10,10,10]
ros2 param set /admittance_node admittance.damping "[20.0,20.0,20.0,15.0,15.0,15.0]"

# Stiffness [N/m, Nm/rad] - default: [10,20,10,10,10,10]
ros2 param set /admittance_node admittance.stiffness "[0,0,0,0,0,0]"  # Pure admittance
```

### Control Modes
- **Pure Admittance**: K=[0,0,0,0,0,0] - no position return
- **Impedance Control**: K>0 - virtual spring to equilibrium
- **Free-float**: M=small, D=small, K=0 - minimal resistance

## Key Features

### Equilibrium Setting
```bash
# Set desired position (orientation fixed)
ros2 topic pub /admittance_node/desired_pose geometry_msgs/PoseStamped \
  "{pose: {position: {x: 0.3, y: 0.2, z: 0.5}}}" --once
```

### Safety Limits
- Workspace: X/Y: ±0.5m, Z: 0-0.7m (hard limits)
- Velocity: 1.5 m/s max
- Acceleration: 1.0 m/s² max

### Services
```bash
ros2 service call /wrench_node/reset_bias std_srvs/srv/Trigger  # Reset F/T bias
```

## Troubleshooting

**Robot doesn't move**: Check force threshold (hardcoded 1.5N) and verify sensor data:
```bash
ros2 topic echo /wrench_tcp_base --once
```

**Unstable motion**: Increase damping or mass:
```bash
ros2 param set /admittance_node admittance.damping "[25,25,25,20,20,20]"
ros2 param set /admittance_node admittance.mass "[5,5,5,2,2,2]"
```

**IK failures**: Robot stops immediately (safety feature). Check workspace limits.

## Design Philosophy

- **Trust the system**: 487Hz joint states don't need tracking flags
- **Fail fast**: IK failure = immediate stop (no decay)
- **Minimize defense**: Remove checks where guarantees exist
- **Simple > Complex**: Hard limits only, fixed filter params

## Implementation
- 100Hz control via ROS2 timer
- KDL WDLS solver (λ=0.01)
- Forward Euler integration
- Pre-allocated messages

## References
- **CLAUDE.md**: Architecture decisions and development notes
- **ur_simulation_gz**: Gazebo simulation package