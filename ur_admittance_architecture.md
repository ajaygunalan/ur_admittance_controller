# UR Admittance Controller - Software Architecture Document

## 1. Overview

### 1.1 Purpose and Scope
The UR Admittance Controller is a ROS2 controller that implements cartesian admittance control for Universal Robots manipulators. It enables compliant motion in response to external forces, transforming force/torque measurements into smooth robot motion commands.

This document describes the software architecture, component design, data flow, and interfaces of the UR Admittance Controller system.

### 1.2 Key Features
- Real-time admittance control implementation
- Force/torque sensor integration with filtering
- Smooth trajectory generation with safety limits
- Chainable controller architecture (ROS2 control framework)
- Configurable compliance parameters (mass, damping, stiffness)
- Robust error handling and retry mechanisms

### 1.3 High-Level Architecture

```
┌───────────────┐    ┌───────────────────────┐    ┌────────────────────┐    ┌───────────┐
│ Force/Torque  │    │                       │    │                    │    │           │
│ Sensor        ├───►│  Admittance Control   ├───►│ Joint Trajectory   ├───►│   Robot   │
│ (Real/Sim)    │    │  Controller           │    │ Controller         │    │           │
└───────────────┘    └───────────────────────┘    └────────────────────┘    └───────────┘
     WrenchMsg             CartesianVelocity          JointTrajectory          Position
  (force/torque)           (twist command)         (positions/velocities)      Commands
```

## 2. System Architecture

### 2.1 Components

| Component | Type | Description |
|-----------|------|-------------|
| `ur_admittance_controller` | ROS2 Controller Plugin | Core controller implementing the admittance control algorithm |
| `wrench_signal_generator` | ROS2 Node | Simulates F/T sensor data with gravity compensation |
| `scaled_joint_trajectory_controller` | ROS2 Controller | Executes joint trajectories sent by admittance controller |

### 2.2 Controller Hierarchy

The controller implements the `ChainableControllerInterface` to integrate within the ROS2 control framework:

```
┌───────────────────────────────┐
│     ros2_control Framework     │
│  ┌──────────────────────────┐  │
│  │    Controller Manager    │  │
│  └──────────┬──────────────┘  │
│             │                 │
│  ┌──────────▼──────────────┐  │
│  │ ur_admittance_controller │  │
│  └──────────┬──────────────┘  │
│             │                 │
│  ┌──────────▼──────────────┐  │
│  │joint_trajectory_controller│  │
│  └──────────┬──────────────┘  │
│             │                 │
│  ┌──────────▼──────────────┐  │
│  │   Hardware Interface     │  │
│  └──────────┬──────────────┘  │
└─────────────┼────────────────┘
              │
      ┌───────▼───────┐
      │  UR5e Robot   │
      └───────────────┘
```

### 2.3 Key Dependencies

| Dependency | Type | Purpose |
|------------|------|---------|
| `controller_interface` | ROS2 Package | Provides controller base classes and interfaces |
| `hardware_interface` | ROS2 Package | Access to robot state and command interfaces |
| `kinematics_interface` | ROS2 Package | Forward/inverse kinematics calculations |
| `tf2` | ROS2 Package | Frame transformations for wrench data |
| `realtime_tools` | ROS2 Package | Real-time safe data structures |
| `Eigen` | Library | Linear algebra operations |
| `KDL` | Library | Kinematics and dynamics calculations |

## 3. Control Architecture

### 3.1 Component Interfaces

| Component | Inputs | Outputs | State Reading |
|-----------|--------|---------|---------------|
| F/T Sensor | N/A | `WrenchStamped` (forces and torques) | No |
| Admittance Controller | `WrenchStamped`, Current joint positions/velocities | `JointTrajectory` | Yes - reads position and velocity interfaces |
| Trajectory Controller | `JointTrajectory` | Position/velocity commands | Yes - reads position and velocity for feedback |

### 3.2 Core Admittance Control Algorithm

The controller implements admittance control based on the second-order dynamic system equation:

```
M * a + D * v + K * x = F_ext
```

Where:
- M: Mass matrix (6×6, diagonal)
- D: Damping matrix (6×6, diagonal)
- K: Stiffness matrix (6×6, diagonal)
- a: Cartesian acceleration 
- v: Cartesian velocity
- x: Position error
- F_ext: External force/torque

**Implementation:**

```cpp
// In calculateAdmittance():
// Solve for acceleration from the admittance equation
desired_acceleration_ = mass_matrix_.inverse() * 
  (wrench_external_ - damping_matrix_ * desired_velocity_ - stiffness_matrix_ * pose_error_);

// Integrate acceleration to get velocity
desired_velocity_ += desired_acceleration_ * period.seconds();

// Apply selective admittance control based on enabled axes
for (size_t i = 0; i < 6; ++i) {
  if (!params_.admittance_enabled_axes[i]) {
    desired_velocity_(i) = 0.0;
  }
}

// Store as Cartesian velocity command
cart_vel_cmd_ = desired_velocity_;
```

### 3.3 Detailed Data Flow Pipeline

```
                 ┌────────────────────────────────────┐                 
                 │                                    │                 
                 │  Current Robot State               │                 
                 │  (Joint Positions/Velocities)      │                 
                 │                                    │                 
                 └────────────┬───────────────────────┘                 
                              │                                          
                              │ Reads                                    
                              ▼                                          
┌───────────────┐     ┌───────────────┐     ┌───────────────┐     ┌───────────────┐
│  F/T Sensor   │     │    Frame      │     │  Admittance   │     │    Inverse    │
│     Data      │────►│ Transformation│────►│  Equations    │────►│  Kinematics   │
│ (Tool Frame)  │     │ (to Base)     │     │  M·a+D·v+K·x=F│     │  (Jacobian)   │
└───────────────┘     └───────────────┘     └───────────────┘     └───────────────┘
   WrenchStamped         BaseWrench           CartesianVelocity        JointVelocities
                                                                             │
                                                                             ▼
┌───────────────┐     ┌───────────────┐     ┌───────────────┐     ┌───────────────┐
│    Robot      │     │   Joint       │     │  Trajectory   │     │    Path       │
│    Motion     │◄────│  Trajectory   │◄────│  Generation   │◄────│  Planning     │
│  (Positions)  │     │  Controller   │     │               │     │               │
└───────────────┘     └───────────────┘     └───────────────┘     └───────────────┘
  Position Commands      JointTrajectory         TimeParameterized        WaypointPath
                      (positions/velocities)         Trajectory
```

## 4. Implementation Details

### 4.1 Lifecycle Management

The controller follows a standard ROS2 control lifecycle pattern:

```
┌────────────────────┐
│                      │
│     Unconfigured     │
│                      │
└─────────┬──────────┘
              │  on_configure()
              ▼
┌────────────────────┐
│                      │
│       Inactive       │
│                      │
└─────────┬──────────┘
              │  on_activate()
              ▼
┌────────────────────┐
│                      │
│        Active        │
│                      │
└────────────────────┘
```

| Lifecycle Phase | Key Operations |
|-----------------|----------------|
| **Configuration** | Load parameters, set up TF buffer, initialize kinematics |
| **Activation** | Allocate memory, set up subscribers, connect interfaces |
| **Execution** | Run control loop, process wrench data, generate trajectories |
| **Deactivation** | Safely stop robot motion, release resources |

### 4.2 Robot State Acquisition and Admittance Control

#### 4.2.1 Reading Robot State

The controller dynamically reads the robot's current state at each control cycle - it does **not** assume a static robot pose:

```cpp
// In update_and_write_commands():
// Acquire current joint positions and velocities from hardware interfaces
for (size_t i = 0; i < params_.joints.size(); ++i) {
  const auto position_interface_index = i;
  const auto velocity_interface_index = i + params_.joints.size();
  
  // Read current position from robot hardware interface
  joint_positions_[i] = state_interfaces_[position_interface_index].get_optional().value();
  // Read current velocity from robot hardware interface
  joint_velocities_[i] = state_interfaces_[velocity_interface_index].get_optional().value();
}
```

This design allows the controller to:
1. Start from **any robot position** without requiring a specific initial pose
2. Adapt to changing robot configurations in real-time
3. Compensate for external disturbances by always using fresh state data

#### 4.2.2 Cartesian-Joint Space Mapping

Unlike some implementations that work directly in joint space, this controller operates in Cartesian (task) space, offering these advantages:

- **Intuitive Force Response**: Forces in Cartesian space directly correspond to physical interactions
- **Simpler Parameter Tuning**: Mass, damping, and stiffness have clearer physical meaning
- **Tool-Centric Control**: Enables compliant behavior relative to the end-effector frame

The Cartesian-to-joint space transformation occurs through these steps:

```cpp
// 1. Calculate current end-effector pose from joint positions
kinematics_->calculate_fk(joint_positions_, current_pose_);

// 2. Calculate Jacobian at current configuration
kinematics_->calculate_jacobian(joint_positions_, jacobian_);

// 3. Convert Cartesian velocity to joint velocities using the Jacobian
joint_velocities_cmd_ = jacobian_.transpose() * cart_vel_cmd_;
```

### 4.3 Wrench Processing Pipeline

The controller implements a sophisticated wrench processing pipeline:

1. **Wrench Signal Reception**
   ```cpp
   // In wrenchCallback():
   void AdmittanceController::wrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
   ```

2. **Signal Filtering**
   ```cpp
   // Apply exponential filter to reduce noise
   msg->wrench.force.x = params_.filter_coefficient * msg->wrench.force.x + 
     (1.0 - params_.filter_coefficient) * last_wrench->wrench.force.x;
   // Similar filtering for all 6 components (fx, fy, fz, tx, ty, tz)
   ```

3. **Thread-Safe Storage**
   ```cpp
   // Store in real-time safe buffer
   wrench_buffer_.writeFromNonRT(*msg);
   ```

4. **Frame Transformation**
   ```cpp
   // Transform wrench from tool frame to base frame
   geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
     base_frame_id_, tool_frame_id_, tf2::TimePointZero);
   ```

5. **Deadband Application**
   ```cpp
   // Apply deadband to filter out sensor noise
   if (std::abs(wrench_external_(i)) < params_.wrench_deadband[i]) {
     wrench_external_(i) = 0.0;
   }
   ```

### 4.4 Trajectory Generation

The controller generates joint trajectories from Cartesian velocities:

1. **Read Current Robot State**
   ```cpp
   // Read current joint positions and velocities from interfaces
   for (size_t i = 0; i < params_.joints.size(); ++i) {
     current_positions[i] = state_interfaces_[position_interface_index].get_optional().value();
   }
   ```

2. **Convert Cartesian to Joint Velocities**
   ```cpp
   // Use KDL Jacobian to convert Cartesian velocity to joint velocities
   KDL::Twist kdl_cart_vel;
   // ... populate kdl_cart_vel ...
   
   // Convert to joint velocities
   kinematics_->convert(cart_vel_cmd_, joint_velocities_cmd_);
   ```

3. **Generate Trajectory Points**
   ```cpp
   // In sendTrajectory():
   trajectory_msgs::msg::JointTrajectory trajectory;
   trajectory.joint_names = params_.joints;
   
   // Create waypoints for trajectory
   trajectory.points.resize(1);  // Single point trajectory
   auto & point = trajectory.points[0];
   
   // Calculate target position using current position + velocity*duration
   for (size_t i = 0; i < params_.joints.size(); ++i) {
     point.positions[i] = current_positions[i] + 
       velocity_commands[i] * params_.trajectory_duration;
   }
   ```

4. **Send to Joint Trajectory Controller**
   ```cpp
   // Create action goal and send to trajectory controller
   auto goal_msg = FollowJointTrajectory::Goal();
   goal_msg.trajectory = trajectory;
   action_client_->async_send_goal(goal_msg);
   ```

### 4.4 Critical Data Structures

| Data Structure | Type | Purpose |
|----------------|------|----------|
| `wrench_buffer_` | `realtime_tools::RealtimeBuffer<WrenchStamped>` | Thread-safe wrench data storage |
| `mass_matrix_` | `Eigen::Matrix<double, 6, 6>` | Inertial parameters |
| `damping_matrix_` | `Eigen::Matrix<double, 6, 6>` | Damping coefficients |
| `joint_positions_` | `Eigen::VectorXd` | Current joint positions |
| `joint_velocities_cmd_` | `Eigen::VectorXd` | Target joint velocities |
| `desired_velocity_` | `Eigen::Matrix<double, 6, 1>` | Cartesian velocity command |

### 4.5 ROS2 Interfaces

#### Subscribers
```cpp
// Wrench input from F/T sensor
rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_subscriber_;
```

#### Publishers
```cpp
// Cartesian velocity output for monitoring
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cart_vel_pub_;
```

#### Action Clients
```cpp
// Joint trajectory execution
rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
```

#### Services
```cpp
// Zero the F/T sensor
rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zero_ft_sensor_service_;
```

## 5. Configuration Parameters

### 5.1 Admittance Parameters

| Parameter | Type | Description | Effect |
|-----------|------|-------------|--------|
| `mass` | Vector(6) | Virtual mass matrix diagonals | Higher values increase inertia |
| `damping_ratio` | Vector(6) | Damping ratio for each DOF | Higher values increase stability |
| `stiffness` | Vector(6) | Virtual spring stiffness | Higher values increase position-holding |
| `deadband` | Vector(6) | Minimum force threshold | Filters sensor noise |

### 5.2 Motion Parameters

| Parameter | Type | Description | Default |
|-----------|------|-------------|---------|
| `trajectory_duration` | double | Duration of generated trajectories | 0.5s |
| `velocity_scale_factor` | double | Safety scaling for velocities | 0.8 |
| `position_limits` | Array | Joint position min/max values | Robot-specific |
| `velocity_limits` | Array | Joint velocity min/max values | Robot-specific |

### 5.3 Error Handling

| Error Condition | Handling Strategy |
|-----------------|-------------------|
| TF Lookup Failure | Retry with exponential backoff |
| Trajectory Execution Failure | Retry with new trajectory |
| Kinematics Error | Use fallback mode (Cartesian control) |

## 6. Real-Time Considerations

### 6.1 Real-Time Constraints
- Designed for real-time control loops (typically 1kHz)
- Bounded execution time guarantees
- Minimal dynamic allocation in control paths

### 6.2 Memory Management
- Pre-allocated matrices and buffers
- Real-time safe communication patterns
- State data protection mechanisms

### 6.3 Thread Safety
- Non-blocking communication patterns
- Real-time safe parameter access
- Protected critical sections

## 7. Extensibility

### 7.1 Customization Points
- Configurable admittance parameters
- Replaceable kinematics plugins
- Custom trajectory generation
- Filter coefficient tuning

### 7.2 Future Enhancements
- Adaptive admittance parameters
- Multi-sensor fusion
- Enhanced safety monitoring
- Dynamic reconfiguration

## 8. Conclusion

The UR Admittance Controller provides a complete implementation of cartesian admittance control for Universal Robots manipulators. Its modular design follows ROS2 control best practices, with clear separation of concerns between:

- Force/torque sensing and processing
- Reference frame transformations
- Admittance control calculations
- Inverse kinematics and motion planning
- Trajectory generation and execution

The controller's compliance behavior can be finely tuned through the mass, damping, and stiffness parameters, making it suitable for various interaction tasks such as assembly, polishing, or human-robot collaboration.
