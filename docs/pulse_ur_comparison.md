# Comparison: ur_admittance_controller vs pulse_force_estimation

## Frame Definitions

### ur_admittance_controller
**wrench_calibration_node.cpp:**
- `base_frame`: "base_link" (ROS2 parameter, configurable)
- `ee_frame`: "tool0" (ROS2 parameter, configurable)
- `sensor_frame`: "netft_link1" (hardcoded in wrench_node.cpp)

**wrench_node.cpp:**
- `BASE_FRAME`: "base_link" (hardcoded constant)
- `EE_FRAME`: "tool0" (hardcoded constant)
- `SENSOR_FRAME`: "netft_link1" (hardcoded constant)

### pulse_force_estimation
**yu_bias_comp_main.cpp:**
- `robot_base_name`: "base_link" (ROS1 parameter, configurable)
- `robot_tool_name`: "tool0" (ROS1 parameter, configurable)
- `sensor_base_name`: "netft_link1" (ROS1 parameter, configurable)
- `probe_frame_name`: "p42v_link1" (default, configurable)

**Key Difference**: pulse_force_estimation adds a 4th frame - the "probe frame" for their visual servoing system.

## Output Topics Comparison

### ur_admittance_controller (wrench_node)
1. `/netft/proc_sensor` - Compensated wrench in sensor frame
2. `/netft/proc_probe` - Compensated wrench in end-effector frame (tool0)
3. `/netft/proc_probe_base` - Compensated wrench in base frame

### pulse_force_estimation (yu_bias_comp_node)
1. `/netft/proc_sensor` - Compensated wrench in sensor frame
2. `/netft/proc_probe` - Compensated wrench in probe frame (p42v_link1)
3. `/netft/raw_sensor_force_norm` - Debug: force magnitude
4. `/netft/raw_sensor_torque_norm` - Debug: torque magnitude

**Key Differences**:
- ur_admittance_controller publishes base frame wrench, pulse doesn't
- pulse publishes debug norms, ur_admittance_controller doesn't
- "proc_probe" means different things: tool0 vs p42v_link1

## Algorithm Implementation Differences

### Calibration Process
Both use LROM (Limited Robot Orientation Method) from Yu et al. 2022:

**ur_admittance_controller:**
- 32 poses generated programmatically
- 10 samples per pose at 10Hz
- Saves to YAML with rotation matrix and quaternion
- Integrated into single executable

**pulse_force_estimation:**
- 32 poses (LROM_NUM_FT_ROBOT_POSES)
- Multiple samples per pose (NUM_FT_SAMPLES_PER_POSE)
- Saves to HDF5 file format
- Separate node for calibration

### Compensation Algorithm
Both implement the same mathematical approach:

**ur_admittance_controller:**
```cpp
// Centralized in algorithms::compensateWrench()
ft_proc_s = f_raw_s - f_grav_s - f_bias_s  // Force
            t_raw_s - (p_grav_s × f_grav_s) - t_bias_s  // Torque
```

**pulse_force_estimation:**
```cpp
// Direct implementation in callback
ft_proc_s[0:3] = f_raw_s - f_grav_s - f_bias_s
ft_proc_s[3:6] = t_raw_s - (p_grav_s_hat * f_grav_s) - t_bias_s
```

## Architecture Differences

### ur_admittance_controller
- ROS2 based
- Modern C++17 with error handling (tl::expected)
- Modular: algorithms separated from nodes
- YAML configuration
- Minimal dependencies

### pulse_force_estimation
- ROS1 based
- C++11 style
- Monolithic: algorithm mixed with ROS code
- HDF5 for data storage
- Depends on MoveIt for robot control

## Data Flow Differences

### ur_admittance_controller
```
/netft/raw_sensor → wrench_node → /netft/proc_sensor
                                → /netft/proc_probe (tool0)
                                → /netft/proc_probe_base
```

### pulse_force_estimation
```
/netft/raw_sensor → yu_bias_comp_node → /netft/proc_sensor
                                      → /netft/proc_probe (p42v_link1)
                                      → /netft/raw_sensor_force_norm
                                      → /netft/raw_sensor_torque_norm
```

## Key Architectural Insights

1. **Frame Philosophy**: pulse adds probe frame for task-specific hardware
2. **Output Philosophy**: ur_admittance_controller provides all coordinate representations, pulse focuses on sensor/probe frames
3. **Storage Format**: YAML (ur) vs HDF5 (pulse) reflects different ecosystems
4. **Modularity**: ur_admittance_controller separates concerns better
5. **Runtime Reconfiguration**: pulse allows bias updates via service, ur requires restart