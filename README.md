# ur_admittance_controller

> **6-DOF force-compliant motion control for Universal Robots - push the robot and it moves!**

## Overview
Standalone ROS2 node providing admittance control for Universal Robots. Converts external forces into smooth robot motions using the classic admittance equation: **M·ẍ + D·ẋ + K·x = F**

Key features:
- **100Hz real-time control loop** with timer-based architecture
- **Event-driven parameter updates** for immediate configuration changes
- **Dual-node architecture**: Separate wrench processing and admittance control
- **Supports all UR robots**: Tested on UR3, UR5e (primary), UR10, UR16, UR20

## Quick Start

### Installation
```bash
mkdir -p ~/ur_ws/src && cd ~/ur_ws/src
git clone https://github.com/ajaygunalan/ur_simulation_gz.git
git clone https://github.com/ajaygunalan/ur_admittance_controller.git

cd ~/ur_ws && rosdep install --from-paths src --ignore-src -r -y
```


```
colcon build --packages-select ur_simulation_gz
colcon build --packages-select ur_admittance_controller

source install/setup.bash
```


### Simulation Demo
```
ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur5e
```


```
ros2 run ur_admittance_controller init_robot
ros2 run ur_admittance_controller verify_poses
```


```
ros2 run ur_admittance_controller init_robot.py
```

# Terminal 2: Start wrench filter node (processes raw F/T sensor data)
ros2 run ur_admittance_controller wrench_node

# Terminal 3: Start admittance controller
ros2 run ur_admittance_controller admittance_node

# Terminal 4: Apply test force (robot will move)
ros2 topic pub /wrench_tcp_base_raw geometry_msgs/WrenchStamped \
  "{header: {frame_id: 'base_link'}, wrench: {force: {x: 10.0}}}" --once
```

#### Alternative: Use launch file
```bash
# Terminal 2: Launch both nodes with proper timing
ros2 launch ur_admittance_controller ur_admittance.launch.py
```

### Hardware Setup
```bash
# Terminal 1: Connect to real UR robot
ros2 launch ur_robot_driver ur_control.launch.py robot_ip:=192.168.1.100

# Terminal 2: Launch admittance control for hardware
ros2 launch ur_admittance_controller ur_admittance.launch.py use_sim_time:=false
```

## Architecture

### System Overview
```
[F/T Sensor] → /wrench_tcp_base_raw → [Wrench Node] → /wrench_tcp_base → [Admittance Node] → /forward_velocity_controller/commands → [Robot]
                                          ↓                                      ↓
                                   (Bias removal +                        (Admittance control)
                                    Low-pass filter)                    
```

### Core Components

#### 1. **Wrench Node** (`wrench_node.cpp`)
- **Purpose**: Pre-processes F/T sensor data
- **Features**:
  - Automatic bias compensation on startup
  - Exponential moving average (EMA) low-pass filtering
  - Per-axis deadband thresholding
  - Runtime bias reset service (`~/reset_bias`)
- **Parameters**:
  - `filter_coefficient`: EMA filter strength (0.1-0.95, default: 0.8)
  - `min_motion_threshold`: Force/torque deadband (default: 1.5 N/Nm)
  - `auto_bias_on_start`: Auto-calibrate on first reading (default: true)

#### 2. **Admittance Node** (`admittance_node.cpp`)
- **Purpose**: Implements 6-DOF admittance control
- **Control Law**: `M·ẍ + D·ẋ + K·x = F_external`
- **Features**:
  - 100Hz real-time control via ROS2 timer
  - KDL-based inverse kinematics (WDLS solver)
  - Workspace limiting and velocity saturation
  - Graceful degradation on IK failures
- **Key Methods**:
  - `compute_admittance()`: Core control algorithm
  - `computeForwardKinematics()`: Joint → Cartesian transform
  - `compute_joint_velocities()`: Cartesian → joint velocity IK

### Node Communication

#### Subscriptions
| Topic | Type | Node | Description |
|-------|------|------|-------------|
| `/wrench_tcp_base_raw` | `WrenchStamped` | wrench_node | Raw F/T sensor data |
| `/wrench_tcp_base` | `WrenchStamped` | admittance_node | Filtered forces/torques |
| `/joint_states` | `JointState` | admittance_node | Current joint positions |
| `/admittance_node/desired_pose` | `PoseStamped` | admittance_node | Target equilibrium pose |

#### Publications
| Topic | Type | Node | Description |
|-------|------|------|-------------|
| `/wrench_tcp_base` | `WrenchStamped` | wrench_node | Filtered F/T data |
| `/forward_velocity_controller/commands` | `Float64MultiArray` | admittance_node | Joint velocity commands |

## Configuration

### Parameter Structure (`admittance_config.yaml`)

```yaml
ur_admittance_controller:
  # Robot configuration
  joints: ["shoulder_pan_joint", "shoulder_lift_joint", ...]  # UR joint names
  base_link: "base_link"                                      # Robot base frame
  tip_link: "tool0"                                          # End-effector frame
  
  # Admittance parameters (6-DOF: [X,Y,Z,Rx,Ry,Rz])
  admittance:
    mass: [2.0, 2.0, 2.0, 2.0, 2.0, 2.0]                   # Virtual inertia [kg, kg·m²]
    damping: [12.0, 12.0, 12.0, 10.0, 10.0, 10.0]          # Damping [Ns/m, Nms/rad]
    stiffness: [10.0, 20.0, 10.0, 10.0, 10.0, 10.0]        # Stiffness [N/m, Nm/rad]
    enabled_axes: [true, true, true, true, true, true]      # Per-axis enable/disable
    min_motion_threshold: 1.5                               # Force threshold [N/Nm]
```

### Control Modes

#### 1. **Pure Admittance Mode** (Force → Motion)
```bash
# Zero stiffness = no position return
ros2 param set /admittance_node admittance.stiffness "[0,0,0,0,0,0]"
```
Best for: Direct force control, manual guidance, teaching

#### 2. **Impedance Control Mode** (Position + Compliance)
```bash
# Non-zero stiffness = virtual spring to equilibrium
ros2 param set /admittance_node admittance.stiffness "[100,100,100,10,10,10]"
```
Best for: Assembly tasks, contact operations, surface following

#### 3. **Selective Compliance**
```bash
# Enable only specific axes (e.g., XY translation + Z rotation)
ros2 param set /admittance_node admittance.enabled_axes "[true,true,false,false,false,true]"
```
Best for: Constrained tasks, planar operations

### Real-Time Tuning

All parameters update immediately without restarting:

```bash
# Increase damping for more stability
ros2 param set /admittance_node admittance.damping "[20.0,20.0,20.0,15.0,15.0,15.0]"

# Decrease mass for faster response
ros2 param set /admittance_node admittance.mass "[1.0,1.0,1.0,0.2,0.2,0.2]"

# Adjust force sensitivity
ros2 param set /wrench_node min_motion_threshold 0.5

# Change filter strength (0.95 = light filtering, 0.1 = heavy)
ros2 param set /wrench_node filter_coefficient 0.6
```

## Advanced Features

### Equilibrium Pose Setting

Set a desired equilibrium position (impedance mode only):
```bash
# Via ROS2 topic
ros2 topic pub /admittance_node/desired_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'base_link'}, 
    pose: {position: {x: 0.3, y: 0.2, z: 0.5}, 
           orientation: {w: 1.0}}}" --once

# Via launch parameter
ros2 param set /admittance_node equilibrium.position "[0.3, 0.2, 0.5]"
```

### Workspace Limits

The controller enforces Cartesian workspace boundaries:
- Default: X: [-0.5, 0.5], Y: [-0.5, 0.5], Z: [0.0, 0.7] meters
- Velocity limit: 1.5 m/s (configurable)
- Acceleration limit: 1.0 m/s² (configurable)

### Force Bias Reset

Reset F/T sensor bias during operation:
```bash
ros2 service call /wrench_node/reset_bias std_srvs/srv/Trigger
```

## Troubleshooting

### Robot doesn't move when pushed
1. **Check force threshold**:
   ```bash
   ros2 param get /wrench_node min_motion_threshold
   # Try lowering it: ros2 param set /wrench_node min_motion_threshold 0.5
   ```

2. **Verify sensor data flow**:
   ```bash
   # Check raw forces
   ros2 topic echo /wrench_tcp_base_raw --once
   # Check filtered forces
   ros2 topic echo /wrench_tcp_base --once
   ```

3. **Verify enabled axes**:
   ```bash
   ros2 param get /admittance_node admittance.enabled_axes
   ```

### Jerky or unstable motion
1. **Increase damping**:
   ```bash
   ros2 param set /admittance_node admittance.damping "[25.0,25.0,25.0,20.0,20.0,20.0]"
   ```

2. **Increase virtual mass**:
   ```bash
   ros2 param set /admittance_node admittance.mass "[10.0,10.0,10.0,2.0,2.0,2.0]"
   ```

3. **Adjust filter coefficient** (lower = more filtering):
   ```bash
   ros2 param set /wrench_node filter_coefficient 0.5
   ```

### High CPU usage
- Control loop is optimized for 100Hz (10ms period)
- Parameter updates are event-driven (no polling)
- Check for other nodes consuming resources

## Technical Details

### Implementation Highlights
- **Control frequency**: 100Hz via ROS2 timer (no manual while loops)
- **Kinematics**: KDL library with WDLS (Weighted Damped Least Squares) solver
- **Integration method**: Forward Euler for velocity integration
- **Thread safety**: Proper mutex protection for shared data
- **Error handling**: Graceful degradation on IK failures

### Performance Specifications
- **Latency**: <2ms control cycle
- **Parameter update**: <1ms (event-driven)
- **Memory**: Pre-allocated messages, no runtime allocations
- **CPU**: ~5-10% on modern processors

## Package Files

```
ur_admittance_controller/
├── src/
│   ├── admittance_node.cpp          # Main control node
│   ├── admittance_computations.cpp  # Core algorithms
│   └── wrench_node.cpp              # F/T preprocessing
├── include/
│   ├── admittance_node.hpp          # Node class definition
│   └── admittance_node_types.hpp    # Type definitions
├── config/
│   └── admittance_config.yaml       # Default parameters
├── launch/
│   └── ur_admittance.launch.py      # Main launch file
├── scripts/
│   ├── init_admittance.py           # Robot initialization
│   └── test_suite.py                # Testing utilities
└── CMakeLists.txt & package.xml     # Build configuration
```

## References & Documentation
- **CLAUDE.md**: Detailed development notes and architecture decisions
- **API Reference**: Method signatures and class interfaces
- **ROS2 Control**: Integration with ros2_control framework
- **UR Robot Driver**: Hardware interface documentation