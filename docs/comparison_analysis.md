# Comparison Analysis: ur_admittance_controller vs pulse_force_estimation

## Critical Differences Found

### 1. Transform Computation Issues

#### Sensor to Probe Transform Direction
**Our implementation:**
```cpp
auto transform_msg = tf_buffer_->lookupTransform(
    frames::SENSOR_FRAME, frames::PROBE_FRAME, tf2::TimePointZero);
Eigen::Isometry3d sensor_to_probe = tf2::transformToEigen(transform_msg);
Eigen::Matrix3d R_PS = sensor_to_probe.rotation().transpose();
```

**Pulse implementation:**
```cpp
this->tf_listener->lookupTransform(
    this->sensor_frame_name, this->probe_frame_name, ros::Time(0), st_s_p);
tf::transformTFToEigen(st_s_p, eig_s_p);
rot_p_s = mat_s_p.block(0, 0, 3, 3).transpose();
```

Both look up sensor→probe transform and transpose the rotation. ✓ SAME

#### Probe to Base Transform
**Our implementation:**
```cpp
Transform X_BP = X_EB_.inverse() * Transform(calibration_params_.R_SE) * 
                 tf2::transformToEigen(tf_buffer_->lookupTransform(
                     frames::PROBE_FRAME, frames::SENSOR_FRAME, tf2::TimePointZero));
```

**Issue**: This math is wrong! 
- X_EB_.inverse() gives X_BE (base to end-effector)
- R_SE is rotation from sensor to end-effector
- lookupTransform(PROBE, SENSOR) gives X_PS (probe to sensor)

The composition X_BE * R_SE * X_PS doesn't give X_BP!

### 2. Missing Topics

**Pulse publishes:**
- `/netft/proc_sensor` ✓
- `/netft/proc_probe` ✓
- `/netft/raw_sensor_force_norm` (debug)
- `/netft/raw_sensor_torque_norm` (debug)

**We publish:**
- `/netft/proc_sensor` ✓
- `/netft/proc_probe` ✓
- `/netft/proc_probe_base` (EXTRA - pulse doesn't have this)

### 3. Transform Availability Handling

**Our implementation:**
```cpp
if (!tf_buffer_->canTransform(..., std::chrono::seconds(10))) {
    RCLCPP_FATAL(...);
    throw std::runtime_error(...);
}
```

**Pulse implementation:**
```cpp
this->tf_listener->waitForTransform(..., ros::Duration(10.0));
// No throw on failure - continues with try/catch
```

### 4. Initialization Differences

**Our implementation:**
- Loads calibration first
- Sets up ROS interfaces
- Then computes adjoint (may fail if TF not ready)

**Pulse implementation:**
- Sets up TF listener
- Computes adjoint in constructor
- Loads calibration from file OR service

### 5. Calibration Loading

**Our implementation:**
- Only loads from YAML file
- No runtime updates

**Pulse implementation:**
- Loads from HDF5 file
- Has service `/netft/yu_set_biases` for runtime updates

### 6. Debug Features

**Pulse has:**
- Force/torque magnitude publishers
- Execution timer
- `/netft/cancel` publisher

**We don't have these**

### 7. Transform Caching

**Our implementation:**
```cpp
tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true))  // true enables caching
```

**Pulse implementation:**
```cpp
this->tf_listener = new tf::TransformListener();  // Default caching behavior
```

### 8. Error in Base Transform Calculation

The correct computation should be:
```cpp
// Get X_BP (base to probe)
// X_BP = X_BS * X_SP
// Where X_BS = X_BE * X_ES and X_ES = inverse(R_SE)
Transform X_ES(calibration_params_.R_SE.transpose()); 
Transform X_BS = X_EB_.inverse() * X_ES;
Transform X_SP = tf2::transformToEigen(tf_buffer_->lookupTransform(
    frames::SENSOR_FRAME, frames::PROBE_FRAME, tf2::TimePointZero));
Transform X_BP = X_BS * X_SP;
```

## Calibration Node Differences

### 1. Pose Generation Algorithm

**Our implementation:**
```cpp
void WrenchCalibrationNode::generateCalibrationPoses() {
    calibration_poses_.assign(CalibrationConstants::NUM_POSES, current_joint_positions_);
    for (int i = 1; i < CalibrationConstants::NUM_POSES; ++i) {
        const double idx = i - 1.0;
        calibration_poses_[i][3] = current_joint_positions_[3] + (idx/CalibrationConstants::NUM_POSES) * M_PI/3.0 - M_PI/6.0;
        calibration_poses_[i][4] = current_joint_positions_[4] + (std::fmod(idx, 8.0)/8.0) * M_PI/3.0 - M_PI/6.0;
        calibration_poses_[i][5] = current_joint_positions_[5] + idx * M_PI/CalibrationConstants::NUM_POSES - M_PI/2.0;
    }
}
```

**Pulse implementation:**
```cpp
this->joint_q_cmd_mat[i0][3] = ((((double)(i0 - 1)) / ((double)this->num_robot_poses)) * M_PI / 3.0) - (M_PI / 6.0) + wrist_1_q_val;
this->joint_q_cmd_mat[i0][4] = ((((double)((i0 - 1) % 8)) / 8.0) * M_PI / 3.0) - (M_PI / 6.0) + wrist_2_q_val;
this->joint_q_cmd_mat[i0][5] = (((double)(i0 - 1)) * M_PI / (this->num_robot_poses)) - (M_PI / 2.0) + wrist_3_q_val;
```

**Same algorithm!** ✓

### 2. Robot Movement

**Our implementation:**
- Uses ROS2 action client directly
- No MoveIt dependency

**Pulse implementation:**
- Uses MoveIt for initial lift
- Then uses trajectory controller
- Has probe_lift_displacement = 0.05m

### 3. Data Storage

**Our implementation:**
- Saves to YAML only
- Fixed location in config/

**Pulse implementation:**
- Saves to HDF5 file
- Configurable output location
- Includes metadata (timestamps, etc.)

### 4. FSM Architecture

**Our implementation:**
- Simple sequential execution
- No state machine

**Pulse implementation:**
- Full FSM with states:
  - INITIALIZE
  - MOVE_TO_POSE
  - COLLECTING_FT_DATA
  - etc.
- Timer-based execution

### 5. Debug Publishers

**Pulse has:**
- `/yu_bias_est/robot_poses`
- `/yu_bias_est/ft_readings`

**We don't publish debug data**

## Summary of Critical Issues

1. **Math Error**: Probe to base transform computation is incorrect
2. **Extra Topic**: We publish base frame wrench that pulse doesn't
3. **No Runtime Updates**: We can't update calibration without restart
4. **Missing Debug Features**: No force/torque norms or timing info
5. **Initialization Order**: May cause TF issues if adjoint computed too early
6. **No FSM**: Sequential execution vs pulse's robust state machine
7. **No MoveIt**: Direct trajectory control vs pulse's collision-aware planning