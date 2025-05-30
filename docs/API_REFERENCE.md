# API Reference

## Node Interface

### Topics
```yaml
# Inputs
/robot_description:    std_msgs/String           # Robot URDF
/joint_states:        sensor_msgs/JointState     # Current joints  
/wrist_ft_sensor:     geometry_msgs/WrenchStamped # F/T sensor

# Outputs
/scaled_joint_trajectory_controller/joint_trajectory: trajectory_msgs/JointTrajectory
/admittance_cartesian_velocity:  geometry_msgs/Twist
/admittance_pose_error:         geometry_msgs/Twist
```

## Parameters

### Admittance Control
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `mass` | double[6] | [8,8,8,0.8,0.8,0.8] | Virtual inertia [kg, kg⋅m²] |
| `stiffness` | double[6] | [0,0,0,0,0,0] | Spring constants [N/m, Nm/rad] |
| `damping_ratio` | double[6] | [0.8,0.8,0.8,0.8,0.8,0.8] | Damping coefficients |
| `min_motion_threshold` | double | 1.5 | Force deadband [N] |

### Safety Limits
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `max_linear_velocity` | double | 0.5 | Max TCP velocity [m/s] |
| `max_angular_velocity` | double | 1.0 | Max angular velocity [rad/s] |

## Usage Examples

### Basic Operation
```bash
# Launch admittance control
ros2 launch ur_admittance_controller ur_admittance.launch.py

# Apply force (robot moves)
ros2 topic pub /wrist_ft_sensor geometry_msgs/WrenchStamped \
  "{header: {frame_id: 'ft_sensor_link'}, wrench: {force: {x: 10.0}}}" --once
```

### Runtime Configuration
```bash
# Switch to impedance mode
ros2 param set /admittance_node admittance.stiffness "[100,100,100,10,10,10]"

# Adjust sensitivity
ros2 param set /admittance_node admittance.min_motion_threshold 0.5
```

## Performance
- **Control frequency**: 500Hz (2ms period)
- **Latency**: <2ms force-to-motion
- **Memory**: Pre-allocated, real-time safe