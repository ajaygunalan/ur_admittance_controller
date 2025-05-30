# Universal Robots Controllers Reference

This document provides a reference for the controllers available in the Universal Robots ROS2 Driver.

## Available Controllers

List of all the commanding controllers is found [here](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/usage/controllers.html)

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

For the UR5e, we implement admittance control instead of impedance control since the robot cannot accept joint torque commands. We have two choices to implement the admittance: in joint space or task space (cartesian-space). 


We choose cartesian space over joint space because:
- We only get indirect estimates of joint torques which are not reliable due to high gear ratios
- We get accurate and precise 6D wrench at the TCP from the force sensor

While `forward_velocity_controller` might seem like a good choice for cartesian-based admittance control compared to the robot's proprietary `force_mode_controller` (which offers coarser control and only one set-point wrench at a time), we use the default `scaled_joint_trajectory_controller` because it offers better safety features, which is important for our applications.


## Choosing the Right Interface for Admittance Loop Control

When driving `scaled_joint_trajectory_controller` with an admittance loop, selecting the appropriate ROS interface is crucial for performance and responsiveness.

## Goal-Type Actions (`/follow_joint_trajectory`)

### Design Purpose
Designed for pre-planned motions that you want to:
- Monitor execution progress
- Pre-empt occasionally 
- Receive result codes upon completion

### Why It's a Mismatch for Admittance Control

**High Overhead at Control Frequencies**
- Each new goal triggers a complete handshake, feedback, and result cycle
- This creates excessive overhead when running at ≥ 10 Hz control rates

**Streaming Inefficiency**
- Streaming would require cancelling and re-sending goals every control tick
- This approach stresses the action server significantly
- Introduces unnecessary latency into the control loop

## Topic Streaming (`/joint_trajectory`)

### Design Purpose
Designed for "fire-and-forget" trajectory updates where the latest message automatically replaces the previous one.

### Why It Fits Admittance Control

**Low Latency Operation**
- Simply publish a `JointTrajectory` message each control cycle
- No complex handshake protocols or acknowledgment overhead

**Implicit Pre-emption**
- No explicit cancel calls required
- All earlier waypoints beyond the current time are automatically dropped

**Community-Tested Reliability**
- Widely used for 10–100 Hz control loops
- Issues with jerkiness are well-understood and solvable
- Include velocities (and possibly accelerations) in each trajectory point

## Key Caveats When Using Topic Streaming

### Smoothness Requirements
- **Always send velocity** (and acceleration if needed) with each waypoint
- This prevents stop-and-go behavior between trajectory points

### Feedback Handling
- No built-in action feedback available
- Monitor execution status through:
  - `/joint_states` topic
  - Controller's `/state` topic

### Controller Limitations
- Very short time horizons (e.g., Δt = 0.01 s) can still cause large accelerations
- Maintain a small buffer or use overlapping trajectories to mitigate this

## Practical Guideline

### For High-Frequency Cartesian Admittance Loops
- **Use**: Publish updated trajectories on `/scaled_joint_trajectory_controller/joint_trajectory`
- **Monitor**: Manage feedback through sensor and joint-state topics

### For Low-Rate Planned Motions
- **Use**: Reserve the action interface (`/follow_joint_trajectory`)
- **When**: You need explicit completion status for fully planned paths

## Critical Takeaway

**Topic streaming** provides the responsiveness an admittance controller needs, while the **action interface** shines only for slower, monitored moves that require explicit completion feedback.