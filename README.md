# UR Admittance Controller

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Documentation](https://img.shields.io/badge/Documentation-Architecture-blue)](ur_admittance_architecture.md)

ROS2 controller implementing cartesian (task-space) admittance control for Universal Robots, enabling compliant reaction to external forces/torques.

## Overview

This controller converts force/torque measurements into robot motion using the admittance control paradigm. The controller:
- Receives wrench data from the F/T sensor or simulation
- Transforms forces/torques into appropriate reference frames
- Generates smooth trajectories with velocity/acceleration limits
- Sends commands to the robot's joint trajectory controller

## Installation

### Dependencies

#### Runtime Dependencies
- **ROS2 Control**: `controller_interface`, `hardware_interface`, `realtime_tools`
- **Messages**: `control_msgs`, `geometry_msgs`, `trajectory_msgs`, `std_srvs`
- **Kinematics**: `kinematics_interface`, `kinematics_interface_kdl`, `kdl_parser`

#### Simulation Dependencies
- **UR Simulation**: `ur_simulation_gz` - Universal Robots Gazebo simulation package
  - Install: `git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git`

### Building

```bash
# Clone the repository into your workspace
cd ~/ros2_ws/src
git clone https://github.com/yourusername/ur_admittance_controller.git

# Build the package
cd ~/ros2_ws
colcon build --packages-select ur_admittance_controller

# Source the workspace
source install/setup.bash
```

## Usage

### Simulation

The recommended approach is to use the existing UR simulation environment and load our controller separately:

1. **Start the Gazebo simulation with the UR robot:**

```bash
ros2 launch ur_simulation_gz ur_sim_control.launch.py
```

2. **Load the admittance controller:**

```bash
ros2 launch ur_admittance_controller load_controller.launch.py
```

Or load it directly with the controller manager:

```bash
ros2 run controller_manager spawner ur_admittance_controller
```

3. **Run the simulated force/torque sensor:**

```bash
ros2 run ur_admittance_controller wrench_signal_generator
```

4. **Apply an external wrench for testing:**

```bash
ros2 service call /apply_link_wrench gazebo_msgs/srv/ApplyLinkWrench \
  "{link_name: 'wrist_3_link',
    reference_frame: 'world',
    reference_point: {x: 0.0, y: 0.0, z: 0.0},
    wrench: {
      force: {x: 0.0, y: 0.0, z: 40.0},
      torque: {x: 0.0, y: 0.0, z: 0.0}
    },
    start_time: {sec: 0, nanosec: 0},
    duration: {sec: 1, nanosec: 0}
  }"
```

### Real Hardware Usage

1. **Verify connection to the UR robot:**
   - Ensure the robot is powered on and in Remote Control mode
   - Verify network connection with the robot (ping robot IP)

2. **Start the UR robot driver:**

```bash
ros2 launch ur_robot_driver ur_control.launch.py robot_ip:=192.168.1.100
```

3. **Load the admittance controller:**

```bash
ros2 launch ur_admittance_controller load_controller.launch.py
```

4. **Verify force-torque sensor readings:**

```bash
ros2 topic echo /wrench
```

## Configuration

### Key Parameters

Parameters can be set in `config/ur_controllers.yaml`:

| Parameter Group | Parameters | Description |
|-----------------|------------|-------------|
| **Admittance** | `mass`, `damping_ratio`, `stiffness` | Control force response sensitivity |
| **Motion** | `trajectory_duration`, `velocity_scale_factor` | Adjust trajectory shape |
| **Tool** | `tool_mass`, `tool_center_of_mass` | Configure tool properties |

### Runtime Parameter Tuning

```bash
# Make the robot more compliant
ros2 param set /ur_admittance_controller mass "[5.0, 5.0, 5.0, 5.0, 5.0, 5.0]"

# Zero the force/torque sensor
ros2 service call /zero_ftsensor std_srvs/srv/Trigger
```

## Implementation Details

### Working Principle

The admittance controller follows this pipeline:

```
[FT Sensor] → [Admittance Control] → [Trajectory Generation] → [Robot]
```

Forces applied to the robot are converted to motion using the admittance control law:
```
M * a + D * v + S * x = F
```

Where:
- M: Mass matrix (6x6)
- D: Damping matrix (6x6)
- S: Stiffness matrix (6x6)
- a: Acceleration
- v: Velocity
- x: Position error
- F: External force/torque

see [Architecture Document](ur_admittance_architecture.md) for more details about the controller implementation

### UR5e Specifications

- 6 DOF, 5kg payload capacity, 850 mm maximum reach
- Sensing/actuation capabilities:
  - Joint position feedback
  - Joint velocity feedback
  - No direct joint torque control (only estimates via motor current)
  - 6-axis force/torque sensing at the TCP

### Controller Selection

We use the default `scaled_joint_trajectory_controller` for safety and reliability. For detailed information about UR robot controllers and why we stciked with `default` controller, see the [UR Controllers Reference](docs/ur_controllers.md).







