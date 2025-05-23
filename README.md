# UR Admittance Controller

A ROS2 controller for Universal Robots that enables force-compliant behavior in response to external forces.

## Overview

This plugin enables the UR robot to react compliantly to external forces by converting force/torque measurements into smooth joint trajectories.

## Key Features

- **Force Compliance**: Responds to external forces with adjustable mass, damping, and stiffness
- **Plugin Architecture**: Implements the ROS2 `ChainableControllerInterface` for direct integration
- **Gravity Compensation**: Uses the UR robot's built-in compensation for accurate force sensing
- **Real-time Performance**: Maintains deterministic control behavior for safety

## Components

### Core Controller

The `ur_admittance_controller` plugin:
- Reads wrench data from a force/torque sensor
- Converts external forces to Cartesian velocities via admittance control
- Generates and sends trajectories to the `scaled_joint_trajectory_controller`

### Utility Tools

- **wrench_signal_generator**: Simulates force/torque sensor signals for testing without hardware
- **joint_motion_example**: Demonstrates trajectory generation and execution (beginner-friendly)

## Quick Start

### Simulation Usage

1. **Start the Gazebo simulation:**

```bash
ros2 launch ur_simulation_gz ur_sim_control.launch.py
```

2. **Launch the admittance controller:**

```bash
ros2 launch ur_admittance_controller ur_admittance_controller.launch.py use_sim:=true
```

3. **Generate test forces (optional):**

```bash
ros2 run ur_admittance_controller wrench_signal_generator
```

### Real Hardware Usage

1. **Verify connection to the UR robot:**
   - Ensure the robot is powered on and in Remote Control mode
   - Verify network connection with the robot (ping robot IP)

2. **Launch the admittance controller with the robot IP:**

```bash
ros2 launch ur_admittance_controller ur_admittance_controller.launch.py use_sim:=false robot_ip:=192.168.1.100
```

3. **Verify force-torque sensor readings:**

```bash
ros2 topic echo /wrench
```

### Configuration

Key parameters in `config/ur_controllers.yaml`:

- **Admittance**: `mass`, `damping_ratio`, `stiffness` control force response
- **Motion**: `trajectory_duration`, `velocity_scale_factor` adjust trajectory shape
- **Tools**: The simulated tool has configurable `tool_mass` and center of mass offset

### Parameter Tuning

Adjust parameters dynamically:

```bash
# Make the robot more compliant
ros2 param set /ur_admittance_controller mass "[5.0, 5.0, 5.0, 5.0, 5.0, 5.0]"

# Zero the simulated force/torque sensor
ros2 service call /zero_ftsensor std_srvs/srv/Trigger
```

## How It Works

The admittance control follows a simple workflow:

1. Read forces from F/T sensor (or simulated signals)
2. Calculate motion using `M * a + D * v + S * x = F`
3. Generate smooth trajectories with velocity/acceleration limits
4. Send commands to the robot's joint trajectory controller

## Troubleshooting

### Common Issues

- **Trajectory Aborts**: Increase `position_tolerance` or `trajectory_duration` 
- **Jerky Motion**: Reduce `velocity_scale_factor` or increase `mass` values
- **No Force Response**: Verify wrench topic is being published and check TF frames
- **Simulation Issues**: Ensure Gazebo is running with proper UR5e model loaded
