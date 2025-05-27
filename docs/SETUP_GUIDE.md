# UR Admittance Controller Setup Guide

## UR Robot Simulation System

When you launch `ur_sim_control.launch.py`, you're starting a comprehensive robotic simulation environment. Here's what becomes available:

### 🤖 Core Robot System

#### Active Controllers:
- **`joint_state_broadcaster`** - Continuously publishes real-time joint data (positions, velocities, efforts) from all 6 robot joints
- **`scaled_joint_trajectory_controller`** - Accepts trajectory commands to move the robot arm smoothly between waypoints

#### Key Robot Topics:
- **`/joint_states`** - Real-time feed of all joint positions, velocities, and efforts
- **`/robot_description`** - URDF model defining robot geometry, joints, and properties
- **`/scaled_joint_trajectory_controller/joint_trajectory`** - Send movement commands here
- **`/tf` & `/tf_static`** - Coordinate transformations between all robot links

### 🎮 Control & Monitoring Infrastructure

#### Controller Management:
The `/controller_manager/*` services let you:
- Load/unload new controllers dynamically
- Switch between different control modes
- Monitor controller health and performance
- List available hardware interfaces

#### Visualization & Interaction:
- **`/clicked_point` & `/goal_pose`** - RViz interaction points for setting targets
- **`/initialpose`** - Set robot's initial position in RViz
- **`/diagnostics`** - System health monitoring

## Available Controllers Reference

The following controllers are available for Universal Robots in ROS2. Each controller has specific use cases and limitations when implementing admittance control. Details on all commanding controllers can be found in the [official UR ROS2 documentation](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/usage/controllers.html).

### Default `scaled_joint_trajectory_controller`
`ScaledJointTrajectoryController` streams position set-points every control cycle and "warps" their timestamps with the robot's live speed-scaling value (teach-pendant slider, safeguard stop) so the path stays geometrically correct while it slows, pauses, or resumes; the velocity and acceleration fields in each point act only as feed-forward cues—no true velocity or torque commands ever reach the robot. 

Conversely, `PassthroughTrajectoryController` forwards the **entire** JointTrajectory (positions + velocities + accelerations) to the UR controller in a single call, letting the firmware handle spline interpolation; this frees the PC from real-time but requires that position, velocity, and acceleration command interfaces be enabled and prevents new goals from pre-empting an active trajectory until completion.

### `io_and_status_controller`

ROS interface for Universal-Robots-specific I/O and status: read/write digital & analog pins, adjust speed-slider, change payload, zero the FT sensor, and monitor robot/safety modes. Runs beside any motion controller and **does not generate motion itself**. Used for grippers.

### `forward_velocity_controller`
This controller streams raw joint velocity commands directly to the robot, making it ideal for applications like teleoperation, Cartesian admittance control, or MoveIt Servo that require low-latency responses. It does not include internal smoothing or safety checks, so users must manually limit speeds and accelerations. Additionally, ensure that any position controllers are deactivated before activating this velocity controller.

### `forward_position_controller`

Publishes immediate joint-position targets at high rate ("jogging"). Good for incremental nudges or UI sliders; interpolation is your responsibility, so large steps can jerk or trigger safety stops. Works concurrently with speed-scaling but, like the default controller, sends position only.

### `force_mode_controller`

Bridges ROS to the URScript `force_mode()` primitive, enabling firmware-level Cartesian force regulation. Use *only* together with the `passthrough_trajectory_controller` when you need constant-force tasks such as insertion, polishing, or pressing. Force commands override motion along compliant axes and cannot be time-varying at high rate. Not a full admittance loop and still offers no joint-torque interface.

### `freedrive_mode_controller`

Activates gravity-compensated **Freedrive** so an operator can hand-guide the arm. Requires a continuous "keep-alive" message on `/freedrive_mode_controller/enable_freedrive_mode`; stops automatically if the heartbeat ceases or the controller is deactivated. Perfect for teaching way-points or quick manual repositioning, but suspends all autonomous motion while active.

## Controller Selection for Admittance Control

### Why Admittance Control?

The UR5e robots cannot accept direct joint torque commands due to their hardware design. This limitation makes traditional impedance control impossible. Instead, we implement admittance control, which:
- Converts measured forces into desired motions
- Works with position-controlled robots
- Provides compliant behavior through velocity commands

### Joint Space vs Cartesian Space

We implement admittance control in **Cartesian space** rather than joint space for several reasons:

1. **Force Measurement Accuracy**
   - The built-in F/T sensor provides accurate 6D wrench measurements at the TCP
   - Joint torque estimates are unreliable due to high gear ratios (50:1 to 100:1)
   - Direct force measurement eliminates the need for complex joint torque estimation

2. **Intuitive Control**
   - Forces and motions are naturally understood in Cartesian space
   - Task-specific behaviors (e.g., maintain Z height, free XY motion) are straightforward
   - Easier to implement safety limits and constraints

### Controller Choice: `scaled_joint_trajectory_controller`

While several controllers could theoretically support admittance control, we use the `scaled_joint_trajectory_controller` for these reasons:

**Advantages:**
- **Safety**: Built-in trajectory validation and smooth motion profiles
- **Speed Scaling**: Automatic adjustment based on teach pendant safety settings
- **Reliability**: Well-tested default controller with extensive safety features
- **Integration**: Works seamlessly with existing UR safety systems

**Why not other controllers?**
- `forward_velocity_controller`: No trajectory validation or smoothing
- `force_mode_controller`: Limited to static force setpoints, not dynamic control
- `forward_position_controller`: Risk of jerky motion without proper interpolation

## Force/Torque Sensor Processing

### Changes Made

We have simplified the force/torque sensor processing in the UR Admittance Controller by removing the special case handling that was previously used when the force/torque sensor frame was identical to the base frame.

### Key Modifications

1. **Force Processing Logic**
   - **Before**: Conditionally applied transform only if `params_.ft_frame != params_.base_link`
   - **After**: Unconditionally applies force transformation if transform is valid

2. **Transform Caches**
   - **Before**: Skipped F/T transform update when sensor frame matched base frame
   - **After**: Always updates both tip and F/T transforms regardless of frame configuration

3. **Transform Validation**
   - **Before**: Special condition for when F/T frame matched base frame
   - **After**: Consistently checks F/T transform validity in all cases

4. **TF Lookup**
   - **Before**: Only checked for F/T transform if frames differed
   - **After**: Always checks for F/T transform availability

### Benefits

1. **Simplified Logic**
   - Removed special case branching, making code flow more consistent
   - Improved readability by using a unified approach

2. **Consistent Transform Management**
   - All transforms are treated equally regardless of frame configuration
   - Reduces potential for bugs due to inconsistent handling

3. **Standardized Force Processing**
   - Forces are always explicitly transformed using the adjoint matrix
   - Creates a more predictable behavior model

### Implementation Details

The simplified approach follows our updated frame notation consistently:

```cpp
// Always apply transform from F/T sensor to base frame
if (transform_base_ft_.isValid()) {
  const auto& transform_data = transform_base_ft_.getTransform();
  F_sensor_base_ = transform_data.adjoint * raw_wrench;
} else {
  // No valid transform yet - use raw data as fallback
  F_sensor_base_ = raw_wrench;
}
```

This consistent handling ensures that all spatial quantities are properly transformed between frames, maintaining our notation principle that clearly indicates what is being measured (`F`), from where (`sensor`), and in which frame it's expressed (`base`).

The controller now always treats the force/torque sensor frame as potentially different from the base frame, applying the appropriate transformation matrix in all cases. This approach is both more explicit and more robust to configuration changes.