# Universal Robots ROS 2 Controller Guide

## Choosing and Driving Controllers for Admittance-Based Applications

### 1. Purpose

This guide catalogs the controllers shipped with the Universal Robots ROS 2 driver and explains how to select both a motion controller and a ROS interface when implementing a high-frequency Cartesian admittance loop on UR manipulators (e.g. UR5e). It also documents our standalone admittance node implementation as an alternative to ROS2 Control-based approaches.

### 2. Controller Catalog (Quick Reference)


List of all the commanding controllers is found [here](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/usage/controllers.html)

#### Default `scaled_joint_trajectory_controller`
`ScaledJointTrajectoryController` streams position set-points every control cycle and "warps" their timestamps with the robot's live speed-scaling value (teach-pendant slider, safeguard stop) so the path stays geometrically correct while it slows, pauses, or resumes; the velocity and acceleration fields in each point act only as feed-forward cues—no true velocity or torque commands ever reach the robot. Conversely, `PassthroughTrajectoryController` forwards the **entire** JointTrajectory (positions + velocities + accelerations) to the UR controller in a single call, letting the firmware handle spline interpolation; this frees the PC from real-time but requires that position, velocity, and acceleration command interfaces be enabled and prevents new goals from pre-empting an active trajectory until completion.

#### `io_and_status_controller`

ROS interface for Universal-Robots-specific I/O and status: read/write digital & analog pins, adjust speed-slider, change payload, zero the FT sensor, and monitor robot/safety modes. Runs beside any motion controller and **does not generate motion itself**. Used for grippers.

#### `forward_velocity_controller`
This controller streams raw joint velocity commands directly to the robot, making it ideal for applications like teleoperation, Cartesian admittance control, or MoveIt Servo that require low-latency responses. It does not include internal smoothing or safety checks, so users must manually limit speeds and accelerations. Additionally, ensure that any position controllers are deactivated before activating this velocity controller.

#### `forward_position_controller`

Publishes immediate joint-position targets at high rate ("jogging"). Good for incremental nudges or UI sliders; interpolation is your responsibility, so large steps can jerk or trigger safety stops. Works concurrently with speed-scaling but, like the default controller, sends position only.

#### `force_mode_controller`

Bridges ROS to the URScript `force_mode()` primitive, enabling firmware-level Cartesian force regulation. Use *only* together with the `passthrough_trajectory_controller` when you need constant-force tasks such as insertion, polishing, or pressing. Force commands override motion along compliant axes and cannot be time-varying at high rate. Not a full admittance loop and still offers no joint-torque interface.

#### `freedrive_mode_controller`

Activates gravity-compensated **Freedrive** so an operator can hand-guide the arm. Requires a continuous "keep-alive" message on `/freedrive_mode_controller/enable_freedrive_mode`; stops automatically if the heartbeat ceases or the controller is deactivated. Perfect for teaching way-points or quick manual repositioning, but suspends all autonomous motion while active.

### 3. Why Cartesian Admittance?

For the UR5e, we implement admittance control instead of impedance control since the robot cannot accept joint torque commands. We have two choices to implement the admittance: in joint space or task space (cartesian-space). 

We choose cartesian space over joint space because:
- We only get indirect estimates of joint torques which are not reliable due to high gear ratios
- We get accurate and precise 6D wrench at the TCP from the force sensor

While `forward_velocity_controller` might seem like a good choice for cartesian-based admittance control compared to the robot's proprietary `force_mode_controller` (which offers coarser control and only one set-point wrench at a time), we use the default `scaled_joint_trajectory_controller` because it offers better safety features, which is important for our applications.

### 3.1 Standalone Admittance Node Architecture

Instead of implementing admittance control as a ROS2 Control ChainableControllerInterface, we developed a **standalone ROS2 node** that:
- **Subscribes to**: Force/torque sensor data (`/wrist_ft_sensor`) and joint states (`/joint_states`)
- **Publishes to**: `scaled_joint_trajectory_controller` via topic interface (`/scaled_joint_trajectory_controller/joint_trajectory`)
- **Operates at**: 200 Hz by default (configurable from 1-1000 Hz) with pre-allocated messages for performance

**Why Standalone Node over ROS2 Control?**
1. **Reduced Complexity**: Eliminated ~50% of code dealing with controller lifecycle, hardware interfaces, and chaining mechanics
2. **Better Debugging**: Standard ROS2 node patterns are easier to trace than controller state machines
3. **Improved Stability**: No controller manager lifecycle issues or interface claiming conflicts
4. **Flexible Integration**: Direct topic-based communication works seamlessly in both simulation and hardware
5. **Maintainable**: Clear separation between admittance algorithm and ROS infrastructure
6. **Configurable Frequency**: Unlike ROS2 Control's fixed controller manager rate, our node supports configurable control frequencies (default 200 Hz, up to 1000 Hz)

**Frame References:**
- **Input**: F/T sensor data in `ft_sensor_link` frame (automatically transformed to `base_link`)
- **Computation**: Admittance control law (M·ẍ + D·ẋ + K·x = F) computed in `base_link` frame
- **Output**: Cartesian velocities at `tool0` converted to joint velocities via pluggable kinematics



### 4. Implementation Flow: Admittance Node → Trajectory Controller

Our admittance node implements the following control flow:
```
F/T Sensor → Admittance Node → Joint Trajectory → Scaled Joint Trajectory Controller → Robot
     (Topic)      (200 Hz)         (Topic)                  (Real-time)              (UR Hardware)
```

The node publishes single-point trajectories with:
- **Positions**: Current joint positions from `/joint_states`
- **Velocities**: Computed from admittance control law
- **Time-from-start**: Fixed at 0.1s for smooth streaming
- **Joint names**: Matching robot configuration

### 5. Driving scaled_joint_trajectory_controller: Action vs Topic

| Aspect | Action (`/follow_joint_trajectory`) | Topic (`/joint_trajectory`) |
|--------|-------------------------------------|------------------------------|
| **Designed for** | Pre-planned paths that need monitoring and a result code. | Continuous "fire-and-forget" updates. |
| **Overhead at 10–100 Hz** | High – handshake, feedback, result each tick. | Minimal – one publish call. |
| **Pre-emption** | Explicit cancel_goal required. | Implicit – new msg replaces old trajectory. |
| **Feedback** | Built-in feedback & result. | Must read `/joint_states` or controller `/state`. |
| **Suitability for admittance loop** | Poor – cancels & goals flood server. | Ideal – low latency, implicit override. |

**Recommendation:** Use the topic interface for high-rate (<100 Hz) admittance control. Reserve the action interface for lower-rate, fully planned motions that need explicit success/failure reporting.

### 6. Best-Practice Checklist for Topic Streaming

- Include velocity (and, if needed, acceleration) with every waypoint to avoid stop-and-go jerks.
- Keep a short but non-zero time-from-start (e.g. 0.05–0.1 s). Extremely short horizons cause large accelerations.
- Monitor execution via `/joint_states` and the controller's `/state` topic – not via action feedback.
- Deactivate any other motion controller before enabling `forward_velocity_controller` or similar.


### 7. Using the Admittance Node

**Launch in Simulation:**
```bash
ros2 launch ur_admittance_controller ur_admittance.launch.py
```

**Launch with Real Robot:**
```bash
ros2 launch ur_admittance_controller ur_admittance.launch.py use_sim:=false
```

**Key Parameters:**
- `admittance.mass`: Virtual mass matrix (default: [8,8,8,0.8,0.8,0.8])
- `admittance.damping_ratio`: Damping ratios (default: [0.8,0.8,0.8,0.8,0.8,0.8])
- `admittance.stiffness`: Spring constants for impedance mode (default: [0,0,0,0,0,0])
- `admittance.min_motion_threshold`: Force deadband in Newtons (default: 1.5)

**Safety Features:**
- Deadband filtering prevents noise-induced motion
- Velocity limits in both Cartesian and joint space
- Gradual stiffness engagement prevents sudden movements
- Automatic safe stop on transform failures or kinematics errors

### 8. Key Takeaway

For high-frequency admittance control on UR robots:
1. Use our **standalone admittance node** for simplified integration and better maintainability
2. Interface with `scaled_joint_trajectory_controller` via **topic streaming** for low-latency control
3. Leverage the **25 Hz control rate** which provides sufficient responsiveness while maintaining stability
4. Take advantage of **pre-allocated messages** and **cached transforms** for consistent performance

This architecture provides a practical balance between performance, safety, and ease of development compared to implementing admittance control within the ROS2 Control framework.