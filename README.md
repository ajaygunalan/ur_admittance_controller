# ur_admittance_controller

> **6-DOF force-compliant motion control for Universal Robots - push the robot and it moves!**

## Overview
Standalone ROS2 node providing admittance control for Universal Robots. Converts external forces into smooth robot motions using: **M·ẍ + D·ẋ + K·x = F**

## Quick Start

### Installation
```bash
mkdir -p ~/ur_ws/src && cd ~/ur_ws/src
git clone https://github.com/ajaygunalan/ur_simulation_gz.git
git clone https://github.com/ajaygunalan/ur_admittance_controller.git

cd ~/ur_ws && rosdep install --from-paths src --ignore-src -r -y
colcon build && source install/setup.bash
```

### Simulation
```bash
# Terminal 1: Launch UR5e + F/T sensor in Gazebo
ros2 launch ur_simulation_gz ur_sim_control.launch.py

# Terminal 2: Launch admittance control  
ros2 launch ur_admittance_controller ur_admittance.launch.py

# Terminal 3: Apply force (robot moves)
ros2 topic pub /wrist_ft_sensor geometry_msgs/WrenchStamped \
  "{header: {frame_id: 'ft_sensor_link'}, wrench: {force: {x: 10.0}}}" --once
```

### Hardware
```bash
# Terminal 1: Connect to real UR robot
ros2 launch ur_robot_driver ur_control.launch.py robot_ip:=192.168.1.100

# Terminal 2: Launch admittance control
ros2 launch ur_admittance_controller ur_admittance.launch.py use_sim:=false
```

## Architecture

### Data Flow
```
[F/T Sensor] → [Admittance Node] → [scaled_joint_trajectory_controller] → [Robot]
```

### Package Structure
```
src/
├── admittance_node.cpp          # Main node + control loop
├── admittance_computations.cpp  # Core dynamics (M·ẍ + D·ẋ + K·x = F)
└── sensor_handling.cpp          # F/T processing + transforms

config/
└── admittance_config.yaml       # Parameter definitions

launch/
└── ur_admittance.launch.py      # Main launch file
```

## Configuration

### Control Modes
```bash
# Pure admittance (default) - force → motion only
ros2 param set /admittance_node admittance.stiffness "[0,0,0,0,0,0]"

# Impedance mode - position regulation + force compliance  
ros2 param set /admittance_node admittance.stiffness "[100,100,100,10,10,10]"

# Selective compliance - enable/disable specific DOF
ros2 param set /admittance_node admittance.enabled_axes "[true,true,false,false,false,true]"
```

### Real-Time Parameter Changes
```bash
# All parameter changes take effect immediately (no restart required)

# Increase damping for stability
ros2 param set /admittance_node admittance.damping_ratio "[1.0,1.0,1.0,1.0,1.0,1.0]"

# Adjust virtual mass for responsiveness
ros2 param set /admittance_node admittance.mass "[5.0,5.0,5.0,0.5,0.5,0.5]"

# Adjust force sensitivity (lower = more sensitive)
ros2 param set /admittance_node admittance.min_motion_threshold 0.5

# Set velocity limits
ros2 param set /admittance_node max_linear_velocity 0.3
```

## Key Features
- **Real-time**: 500Hz control loop with precise timing
- **Safety**: Velocity limits, deadband filtering, graceful fallbacks
- **Integration**: Works with ur_simulation_gz + scaled_joint_trajectory_controller  
- **Dynamic Parameters**: Immediate parameter updates via event-driven callback system
- **Simplified Architecture**: Clean, maintainable codebase with minimal overhead

## Troubleshooting

### Robot doesn't move
1. Check force threshold: `ros2 param get /admittance_node admittance.min_motion_threshold`
2. Verify F/T data: `ros2 topic echo /wrist_ft_sensor --once`
3. Check enabled axes: `ros2 param get /admittance_node admittance.enabled_axes`

### Jerky motion
1. Increase damping: `ros2 param set /admittance_node admittance.damping_ratio "[1.2,1.2,1.2,1.2,1.2,1.2]"`
2. Increase virtual mass: `ros2 param set /admittance_node admittance.mass "[12,12,12,1.5,1.5,1.5]"`

## Documentation
- **API Reference**: `docs/API_REFERENCE.md`
- **Architecture**: `docs/ARCHITECTURE.md`  
- **Setup Guide**: `docs/SETUP_GUIDE.md`
- **Notation Guide**: `docs/NOTATION_GUIDE.md`