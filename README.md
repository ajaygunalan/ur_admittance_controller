# ur_admittance_controller

> **6-DOF force-compliant motion control for Universal Robots with gravity compensation**

## Overview
Admittance control for Universal Robots using **M·ẍ + D·ẋ + K·x = F** with proper F/T sensor gravity compensation. Prototype-first design philosophy: trust the system, minimize defensive code, fail fast.

Key features:
- **100Hz control** with 487Hz joint state feedback
- **Real-time M/K/D tuning** via ros2 param
- **Gravity compensation** using Yu et al. LROM calibration
- **Modular compensation architecture** (ready for dynamic compensation)
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

### Calibration (Required for gravity compensation)
```bash
# 1. Launch robot
ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur5e

# 2. Move to safe starting pose
ros2 run ur_admittance_controller init_robot

# 3. Run calibration (32 poses, ~5 minutes)
ros2 run ur_admittance_controller wrench_calibration_node

# 4. Switch back to velocity controller for admittance control
ros2 control switch_controllers --deactivate scaled_joint_trajectory_controller --activate forward_velocity_controller

# Calibration saved to: config/wrench_calibration.yaml
```

### Running
```bash
# Terminal 1: Launch robot
ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur5e

# Terminal 2: Initialize robot to equilibrium position
ros2 run ur_admittance_controller init_robot
# This moves robot to safe working pose and saves equilibrium to config
# Note: init_robot now gets robot_description from the /robot_description topic

# Terminal 3: Run admittance control with saved equilibrium
ros2 control switch_controllers --deactivate scaled_joint_trajectory_controller --activate forward_velocity_controller
ros2 run ur_admittance_controller admittance_node --ros-args --params-file src/ur_admittance_controller/config/equilibrium.yaml

# Terminal 4: Run wrench node
ros2 run ur_admittance_controller wrench_node

# Terminal 5: Test with force command
ros2 topic pub /F_P_B geometry_msgs/WrenchStamped \
  "{header: {frame_id: 'base_link'}, wrench: {force: {x: 10.0}}}" --once
```

### Hardware
```bash
# Real robot (set use_sim_time:=false)
ros2 launch ur_robot_driver ur_control.launch.py robot_ip:=192.168.1.100
ros2 launch ur_admittance_controller ur_admittance.launch.py use_sim_time:=false
```

## Architecture

```
[F/T Sensor] → /F_P_P_raw → [Wrench Node] → /F_P_B → [Admittance Node] → /forward_velocity_controller/commands → [Robot]
                               ↓                  ↓                               ↓
                      (Gravity + Bias Compensation)    (Transform to Base)    (M·ẍ + D·ẋ + K·x = F)
```

### Drake-Style Variable Naming Convention

This package follows Drake robotics notation for clarity and consistency:

**Format**: `<Quantity>_<Point>_<Frame>` where:
- **Quantity**: Type of variable (F=Force/Wrench, V=Velocity/Twist, X=Transform, p=Position)
- **Point**: Where it's measured/applied (P=Payload, W3=Wrist3, B=Base)
- **Frame**: Reference frame for expression (P=Payload, W3=Wrist3, B=Base)

**Examples**:
- `F_P_B`: Force at Payload, expressed in Base frame
- `V_P_B_commanded`: Commanded velocity at Payload in Base frame
- `X_BP`: Transform from Base to Payload
- `p_PW3_W3`: Position of Payload relative to Wrist3, in Wrist3 frame

**Key Variables**:
| Variable | Meaning |
|----------|---------|
| `V_P_B_commanded` | Commanded Cartesian velocity at payload in base frame |
| `X_BP_current` | Current transform from base to payload |
| `X_BP_desired` | Desired transform from base to payload |
| `X_BW3` | Transform from base to wrist3 (cached for efficiency) |
| `X_W3P` | Fixed transform from wrist3 to payload |
| `p_CoM_P` | Center of mass position in payload frame |
| `F_P_B` | External wrench at payload in base frame |
| `F_P_P_raw` | Raw sensor wrench at payload in payload frame |
| `F_P_P` | Compensated wrench at payload in payload frame |
| `F_gravity_B` | Gravity force vector in base frame |
| `F_bias_P` | Force bias in payload frame |
| `T_bias_P` | Torque bias in payload frame |

### Kinematic Chain

```
base_link → [6 joints] → wrist_3_link → [fixed] → tool_payload (F/T sensor location)
```

- **base_link**: Robot base reference frame
- **wrist_3_link**: Last movable joint (where IK is computed)
- **tool_payload**: F/T sensor and payload mounting point

### Components

**Init Robot Node**: Initializes robot to safe equilibrium position
- Defines equilibrium in joint space for reliable positioning
- Moves robot from home to working pose (elbow bent ~90°)
- Computes forward kinematics to get Cartesian equilibrium
- Saves calculated pose to `config/equilibrium.yaml`
- Separates runtime values from parameter definitions

**Wrench Calibration Node**: One-time gravity compensation calibration
- Moves robot through 32 LROM poses automatically
- Estimates tool mass, center of mass, sensor bias, and rotation
- Saves parameters to `config/wrench_calibration.yaml`

**Wrench Node**: F/T preprocessing with gravity compensation
- Loads calibration from YAML file
- Compensates gravity forces based on robot orientation
- EMA filter (α=0.8) and deadband threshold (1.5 N/Nm)
- Modular compensator interface (gravity_bias, future: dynamic)

**Admittance Node**: 6-DOF control at 100Hz
- KDL kinematics (WDLS solver, λ=0.01)
- Immediate stop on IK failure
- Hard workspace limits only
- Dynamic parameters: M/K/D only
- Reads equilibrium from config file (set by init_robot)

### Topics

**Inputs**:
- `/F_P_P_raw` → wrench_node (raw F/T data at Payload in Payload frame)
- `/F_P_B` → admittance_node (compensated wrench at Payload in Base frame)
- `/joint_states` → admittance_node (487Hz)
- `/admittance_node/desired_pose` → admittance_node (optional)

**Outputs**:
- `/F_P_B` ← wrench_node (compensated wrench at Payload in Base frame)
- `/forward_velocity_controller/commands` ← admittance_node (100Hz)

## Gravity Compensation

### Problem Solved
The F/T sensor reads not just contact forces, but also:
- **Tool gravity** (changes with orientation) 
- **Sensor bias** (constant offset)

Simple bias removal treats gravity as constant, causing Z-axis drift when robot tilts.

### Solution: Yu et al. LROM Method
- **32-pose calibration** identifies all unknowns
- **Compensates orientation-dependent gravity** 
- **Estimates tool mass and center of mass**
- **Works for any tool mounting**

### Configuration Files

**Parameter Library Configuration** (`config/admittance_config.yaml`):
```yaml
# Used by generate_parameter_library for code generation
ur_admittance_controller:
  admittance:
    mass:
      type: double_array
      default_value: [2.0, 2.0, 2.0, 2.0, 2.0, 2.0]
    stiffness:
      type: double_array
      default_value: [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
    damping:
      type: double_array
      default_value: [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
  # ... other parameter definitions
```

**Runtime Equilibrium Configuration** (`config/equilibrium.yaml`):
```yaml
# Generated by init_robot, used by admittance_node
admittance_node:
  ros__parameters:
    equilibrium.position: [0.761, 0.133, 0.257]  # Cartesian position [m]
    equilibrium.orientation: [0.212, -0.674, 0.674, -0.212]  # Quaternion WXYZ
```

**Calibration Parameters** (`config/wrench_calibration.yaml`):
```yaml
# Generated by wrench_calibration_node
tool_mass_kg: 2.485
tool_center_of_mass: [0.0, 0.0, 0.125]  # p_CoM_P: center of mass in payload frame
gravity_in_base_frame: [0.0, 0.0, -24.38]  # includes base tilt
force_bias: [0.12, -0.34, 0.08]  # sensor bias
torque_bias: [0.002, 0.001, -0.003]
rotation_sensor_to_endeffector: [[1,0,0], [0,1,0], [0,0,1]]
```

## Parameters

### Wrench Node Parameters
```bash
# Compensation type (current: gravity_bias, future: dynamic)
ros2 param set /wrench_node compensation_type "gravity_bias"

# Calibration file location  
ros2 param set /wrench_node calibration_file "config/wrench_calibration.yaml"
```

### Dynamic Parameters (Runtime Tuning)
```bash
# Virtual mass [kg, kg·m²] - default: [2,2,2,2,2,2]
ros2 param set /admittance_node admittance.mass "[1.0,1.0,1.0,0.5,0.5,0.5]"

# Damping [Ns/m, Nms/rad] - default: [10,10,10,10,10,10]
ros2 param set /admittance_node admittance.damping "[20.0,20.0,20.0,15.0,15.0,15.0]"

# Stiffness [N/m, Nm/rad] - default: [10,10,10,10,10,10]
ros2 param set /admittance_node admittance.stiffness "[0,0,0,0,0,0]"  # Pure admittance
```

### Control Modes
- **Pure Admittance**: K=[0,0,0,0,0,0] - no position return
- **Impedance Control**: K>0 - virtual spring to equilibrium
- **Free-float**: M=small, D=small, K=0 - minimal resistance

## Running Individual Nodes

```bash
# Initialize robot to equilibrium position (required first)
ros2 run ur_admittance_controller init_robot
# This defines equilibrium in joint space and saves calculated Cartesian pose to config
# Gets robot_description from /robot_description topic published by robot_state_publisher

# Calibration (run once per tool change)
ros2 run ur_admittance_controller wrench_calibration_node
ros2 control switch_controllers --deactivate scaled_joint_trajectory_controller --activate forward_velocity_controller

# Wrench preprocessing
ros2 run ur_admittance_controller wrench_node

# Admittance control (reads equilibrium from config)
ros2 run ur_admittance_controller admittance_node --ros-args --params-file src/ur_admittance_controller/config/equilibrium.yaml

# With debug logging:
ros2 run ur_admittance_controller wrench_node --ros-args --log-level debug
```

## Safety Limits
- **Workspace**: X/Y: ±0.5m, Z: 0-0.7m (hard limits)
- **Velocity**: Translation: 1.5 m/s, Rotation: 3.0 rad/s  
- **Acceleration**: Translation: 1.0 m/s², Rotation: 2.0 rad/s²

## Troubleshooting

**Robot doesn't move**: 
1. Check calibration exists: `ls config/wrench_calibration.yaml`
2. Verify compensated wrench: `ros2 topic echo /wrench_tcp_base --once`
3. Check force threshold (1.5N minimum)

**Z-axis drift after applying downward force**:
- This indicates missing gravity compensation
- Run calibration: `ros2 run ur_admittance_controller wrench_calibration_node`

**Unstable motion**: Increase damping or mass:
```bash
ros2 param set /admittance_node admittance.damping "[25,25,25,20,20,20]"
ros2 param set /admittance_node admittance.mass "[5,5,5,2,2,2]"
```

**IK failures**: Robot stops immediately (safety feature). Check workspace limits.

## Code Examples

### Key Transforms and Velocities
```cpp
// Get current payload pose
get_X_BP_current();  // Updates X_BP_current from joint positions

// Compute velocity from admittance equation
Vector6d acceleration = M_inverse_diag.array() * 
    (F_P_B.array() - D_diag.array() * V_P_B_commanded.array() - 
     K_diag.array() * pose_error.array());

// Transform payload velocity to wrist3 for IK
KDL::Vector p_PW3_B = X_BP.p - X_BW3.p;  // Position offset
V_W3_B.vel = V_P_B.vel - V_P_B.rot * p_PW3_B;  // Velocity transform
```

## Future Extensions

The modular compensator architecture supports future dynamic compensation:

```cpp
// Future: Add inertial, Coriolis, centrifugal compensation
ros2 param set /wrench_node compensation_type "dynamic"
```

This would compensate for high-speed motion effects using joint velocities/accelerations.

## Design Philosophy

- **Trust the system**: 487Hz joint states don't need tracking flags
- **Fail fast**: IK failure = immediate stop (no decay)
- **Minimize defense**: Remove checks where guarantees exist
- **Simple > Complex**: Hard limits only, proven algorithms
- **Modular**: Each compensator handles one physics effect

## Force/Torque Sensor Comparison: UR5e Built-in vs ATI External

### Overview
The UR5e robot includes a built-in force/torque sensor, while many research applications use external ATI sensors. Understanding their differences is crucial for implementation decisions.

### Key Specifications

| Feature | UR5e Built-in | ATI Gamma | Difference |
|---------|---------------|-----------|------------|
| **Sensor Type** | Proprietary UR design | Silicon strain gauge | - |
| **Force Range** | ±50 N | ±32 to ±660 N | ATI configurable |
| **Force Accuracy** | ±4.0 N (8%) | ±0.03 N (0.1%) | ATI 133x more accurate |
| **Force Precision** | ±3.5 N | <0.05 N | ATI 70x more precise |
| **Sampling Rate** | ~500 Hz | Up to 8 kHz | ATI 16x faster |
| **Integration** | Built-in, plug & play | External mounting | - |
| **Cost** | Included | $5,000-$15,000 | - |

### Sign Conventions
- **UR5e Built-in**: Follows right-hand rule, no sign inversions needed
- **ATI Sensors**: May require sign inversions depending on mounting (e.g., ATI Gamma needs `-force.x` and `-torque.x`)

### Application Guidelines

**Use UR5e Built-in When:**
- Force changes >5N are sufficient
- Collaborative safety is primary goal
- Quick deployment is needed
- Budget is limited
- Basic force feedback suffices

**Use ATI External When:**
- Sub-Newton precision required
- High-frequency force control (>500Hz)
- Research-grade measurements
- Surface following/polishing tasks
- Precise assembly operations

### Implementation Notes
1. Our simulation models the UR5e built-in sensor behavior
2. Do NOT apply ATI-specific sign corrections unless using ATI hardware
3. Calibration procedures differ significantly between sensors
4. UR5e sensor data available via standard ROS drivers
5. ATI requires separate drivers and frame management

## References
- **Yu et al.**: "Decoupling of gravitational and inertial forces for robot force sensor calibration"
- **CLAUDE.md**: Architecture decisions and development notes
- **ur_simulation_gz**: Gazebo simulation package