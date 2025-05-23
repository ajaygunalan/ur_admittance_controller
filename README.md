# UR Admittance Controller
ROS2 controller implementing cartesian (task-space) admittance control for Universal Robots. 


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

3. **Run the simulated force/torque sensor (optional):**

```bash
ros2 run ur_admittance_controller wrench_signal_generator
```

The wrench signal generator accurately simulates a UR5e force/torque sensor with:
- Tool frame to base frame transformations (like real UR sensors)
- Gravity compensation based on configured tool mass
- `zero_ftsensor` service (equivalent to "Calibrate FT Sensor" on the teach pendant)

4. Apply an external wrench to the robot's wrist link:

```bash
ros2 service call /gazebo/apply_body_wrench gazebo_msgs/srv/ApplyBodyWrench \
  "{body_name: 'robot::wrist_3_link',
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

## Documentation

### Architecture & Technical Details

For a comprehensive understanding of the controller's internal architecture, please refer to the [**UR Admittance Architecture**](ur_admittance_architecture.md) document, which covers:

- Detailed system component descriptions
- Complete execution pipeline
- Frame transformations and mathematical operations
- Controller parameters and configuration options
- Implementation details and error handling strategies

## How It Works

The admittance controller follows this simple pipeline:

```
[FT Sensor] → [Admittance Control] → [Trajectory Generation] → [Robot]
```

Forces applied to the robot are converted to motion using the admittance control law:
```
M * a + D * v + S * x = F
```

> **Technical Details**: For a comprehensive explanation of the pipeline, data types, and mathematical operations, see the [architecture document](ur_admittance_architecture.md#42-runtime-pipeline).

## Testing

### Testing with External Forces



## Robot Details

### UR5e Specifications

- 6 DOF, 5kg payload capacity, 850 mm maximum reach
- Sensing/actuation capabilities:
  - Joint position feedback
  - Joint velocity feedback
  - No direct joint torque control (only estimates via motor current)
  - 6-axis force/torque sensing at the TCP

### Controller Selection

We use the default `scaled_joint_trajectory_controller` for safety and reliability. For detailed information about UR robot controllers, see the [UR Controllers Reference](ur_admittance_architecture.md#626-trajectory-execution).

## Dependencies

### ROS2 Dependencies

- **Core**: `rclcpp`, `rclcpp_lifecycle`, `rclcpp_action`
- **Control**: `controller_interface`, `hardware_interface`, `realtime_tools`
- **Messages**: `control_msgs`, `geometry_msgs`, `trajectory_msgs`, `std_srvs`
- **Kinematics**: `kinematics_interface`, `kinematics_interface_kdl`, `kdl_parser`
- **TF**: `tf2`, `tf2_ros`, `tf2_geometry_msgs`
- **Other**: `pluginlib`, `Eigen3`

## Troubleshooting

### Common Issues

- **Trajectory Aborts**: Increase `position_tolerance` or `trajectory_duration` 
- **Jerky Motion**: Reduce `velocity_scale_factor` or increase `mass` values
- **No Force Response**: Verify wrench topic is being published and check TF frames
- **Simulation Issues**: Ensure Gazebo is running with proper UR5e model loaded
