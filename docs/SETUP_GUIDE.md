# Setup Guide

## Installation
```bash
# Create workspace and clone
mkdir -p ~/ur_ws/src && cd ~/ur_ws/src
git clone https://github.com/ajaygunalan/ur_simulation_gz.git
git clone https://github.com/ajaygunalan/ur_admittance_controller.git

# Install dependencies and build
cd ~/ur_ws && rosdep install --from-paths src --ignore-src -r -y
colcon build && source install/setup.bash
```

## Quick Start

### Simulation Mode
```bash
# Terminal 1: Launch UR5e + F/T sensor in Gazebo
ros2 launch ur_simulation_gz ur_sim_control.launch.py

# Terminal 2: Launch admittance control  
ros2 launch ur_admittance_controller ur_admittance.launch.py

# Terminal 3: Apply force (robot moves)
ros2 topic pub /wrist_ft_sensor geometry_msgs/WrenchStamped \
  "{header: {frame_id: 'ft_sensor_link'}, wrench: {force: {x: 10.0}}}" --once
```

### Hardware Mode
```bash
# Terminal 1: Connect to real UR robot
ros2 launch ur_robot_driver ur_control.launch.py robot_ip:=192.168.1.100

# Terminal 2: Launch admittance control for hardware
ros2 launch ur_admittance_controller ur_admittance.launch.py use_sim:=false
```

## Controller Selection

### Why scaled_joint_trajectory_controller?
- **Safety**: Built-in trajectory validation and smooth motion
- **Speed scaling**: Automatic adjustment via teach pendant  
- **Integration**: Works with existing UR safety systems
- **Reliability**: Well-tested default controller

### Why Cartesian Admittance?
- **Accurate sensing**: Built-in F/T sensor provides precise 6D measurements
- **Intuitive control**: Forces/motions naturally understood in Cartesian space
- **UR hardware**: No direct joint torque commands available

## Configuration

### Control Modes
```bash
# Pure admittance (default)
ros2 param set /admittance_node admittance.stiffness "[0,0,0,0,0,0]"

# Impedance mode
ros2 param set /admittance_node admittance.stiffness "[100,100,100,10,10,10]"

# Selective compliance (XY translation + Z rotation)
ros2 param set /admittance_node admittance.enabled_axes "[true,true,false,false,false,true]"
```

### Safety Tuning
```bash
# Increase damping for stability
ros2 param set /admittance_node admittance.damping_ratio "[1.2,1.2,1.2,1.2,1.2,1.2]"

# Adjust force sensitivity
ros2 param set /admittance_node admittance.min_motion_threshold 0.5

# Set velocity limits
ros2 param set /admittance_node max_linear_velocity 0.3
```

## Troubleshooting

### Robot doesn't move
1. Check force threshold: `ros2 param get /admittance_node admittance.min_motion_threshold`
2. Verify F/T data: `ros2 topic echo /wrist_ft_sensor --once`
3. Check enabled axes: `ros2 param get /admittance_node admittance.enabled_axes`

### Jerky motion
1. Increase damping: `ros2 param set /admittance_node admittance.damping_ratio "[1.0,1.0,1.0,1.0,1.0,1.0]"`
2. Increase virtual mass: `ros2 param set /admittance_node admittance.mass "[12,12,12,1.5,1.5,1.5]"`

### Transform errors
1. Check TF tree: `ros2 run tf2_tools view_frames`
2. Verify transforms: `ros2 topic echo /tf_static`