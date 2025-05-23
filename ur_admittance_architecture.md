# UR Admittance Controller: Architecture Documentation

## 1. Overview

The UR Admittance Controller is a ROS2 controller that implements cartesian admittance control for Universal Robots manipulators. It enables compliant motion in response to external forces, transforming force/torque measurements into robot motion.

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

### 2.2 Key Dependencies

| Dependency | Type | Purpose |
|------------|------|--------|
| `controller_interface` | ROS2 Package | Provides controller base classes and interfaces |
| `hardware_interface` | ROS2 Package | Access to robot state and command interfaces |
| `kinematics_interface` | ROS2 Package | Forward/inverse kinematics calculations |
| `tf2` | ROS2 Package | Frame transformations for wrench data |
| `realtime_tools` | ROS2 Package | Real-time safe data structures |

### 2.3 Integration with ROS2 Control

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

## 3. Control Architecture

### 3.1 Data Flow Pipeline with Inputs/Outputs

The controller reads the current robot state and processes external force data through a sequential pipeline:

```
┌───────────────┐     ┌───────────────┐     ┌───────────────┐     ┌───────────────┐
│  F/T Sensor   │     │    Frame      │     │  Admittance   │     │    Inverse    │
│     Data      │────►│ Transformation│────►│  Equations    │────►│  Kinematics   │
│ (Tool Frame)  │     │ (to Base)     │     │  M·a+D·v+K·x=F│     │  (Jacobian)   │
└───────────────┘     └───────────────┘     └───────────────┘     └───────────────┘
   WrenchStamped         BaseWrench              CartesianVelocity         JointVelocities
                                                                           │
                     ┌────────────────────────────────────┐    ▼
                     │ Current Robot State (positions/velocities) │    
                     └────────────────────────────────────┘    
                                                                           
┌───────────────┐     ┌───────────────┐     ┌───────────────┐     ┌───────────────┐
│    Robot      │     │   Joint       │     │  Trajectory   │     │    Path       │
│    Motion     │◄────│  Trajectory   │◄────│  Generation   │◄────│  Planning    │
│  (Positions)  │     │  Controller   │     │               │     │               │
└───────────────┘     └───────────────┘     └───────────────┘     └───────────────┘
  Position Commands      JointTrajectory         TimeParameterized         WaypointPath
                      (positions/velocities)         Trajectory
```

### 3.2 Component Interfaces

| Component | Inputs | Outputs | State Reading |
|-----------|--------|---------|---------------|
| F/T Sensor | N/A | `WrenchStamped` (forces and torques) | No |
| Admittance Controller | `WrenchStamped`, Current joint positions/velocities | `JointTrajectory` | Yes - reads position and velocity interfaces |
| Trajectory Controller | `JointTrajectory` | Position/velocity commands | Yes - reads position and velocity for feedback |

### 3.3 Core Admittance Control Algorithm

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

**Implementation (from the code):**

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

## 4. Implementation Details

### 4.1 Initialization Phase

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

### 4.2 Wrench Processing Pipeline

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

### 4.3 Trajectory Generation

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

## 6. Conclusion

The UR Admittance Controller provides a complete implementation of cartesian admittance control for Universal Robots manipulators. Its modular design follows ROS2 control best practices, with clear separation of concerns between:

- Force/torque sensing and processing
- Reference frame transformations
- Admittance control calculations
- Inverse kinematics and motion planning
- Trajectory generation and execution

The controller's compliance behavior can be finely tuned through the mass, damping, and stiffness parameters, making it suitable for various interaction tasks such as assembly, polishing, or human-robot collaboration.
