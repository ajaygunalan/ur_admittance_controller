# UR Admittance Controller: Architecture & Execution Pipeline

## 1. System Overview

The UR Admittance Controller is a ROS2 controller plugin that enables force-compliant motion control for Universal Robots manipulators. It follows the admittance control paradigm, where external forces drive robot motion according to a second-order mechanical system model.

```
                                                    ROS2 Control Framework
                                                  ┌───────────────────────┐
                                                  │  Controller Manager   │
                                                  └───────────┬───────────┘
                                                              │
                                       ┌──────────────────────┴─────────────────────┐
                                       │                                              │
                              ┌────────▼───────┐                          ┌──────────▼─────────┐
                              │ Controllers     │                          │ Hardware Interface  │
┌──────────────────┐   msg   │ ┌──────────────┐│   cmd   ┌─────────────┐ │ ┌─────────────────┐│
│  External Input  │────────►│ │ ur_admittance││────────►│  Chainable  │ │ │ Hardware Access ││
│  (F/T Sensor)    │         │ │ controller   ││         │ Controllers │ │ │ Abstraction     ││
└──────────────────┘         │ └──────────────┘│         └─────────────┘ │ └─────────────────┘│
                              └────────────────┘                          └────────────────────┘
                                                                                     │
                                                                                     ▼
                                                                          ┌────────────────────┐
                                                                          │ Physical Hardware  │
                                                                          │ or Simulation     │
                                                                          └────────────────────┘
```

## 2. System Components

| Component | Type | Purpose |
|-----------|------|---------|
| `ur_admittance_controller` | ROS2 Controller Plugin | Main admittance control implementation |
| `wrench_signal_generator` | ROS2 Node | Simulates F/T sensor data for testing |
| `joint_motion_example` | ROS2 Node | Demonstrates trajectory execution |
| `scaled_joint_trajectory_controller` | ROS2 Controller Plugin | Downstream controller that executes joint trajectories |

## 3. Control Architecture

### 3.1 Controller Plugin Integration

The admittance controller implements the `controller_interface::ChainableControllerInterface` to integrate with the ROS2 Control framework:

```
                                  ┌─────────────────────────┐
┌──────────────────┐  WrenchMsg  │                         │  JointTrajectory  ┌───────────────────────────┐
│ F/T Sensor or    │───────────▶│ ur_admittance_controller │───────────────▶  │scaled_joint_trajectory_ctrl│───▶ Robot
│ Signal Generator │             │                         │                   │                           │
└──────────────────┘             └─────────────────────────┘                   └───────────────────────────┘
```

### 3.2 Data Flow Pipeline

```
[FT Sensor] → WrenchStamped → [Frame Transform] → Base Frame Wrench → 
[Admittance Control] → Cartesian Velocity → [Inverse Kinematics] → 
Joint Velocity → [Trajectory Generator] → JointTrajectory → 
[Trajectory Controller] → Hardware Commands → [Robot]
```

## 4. Execution Pipeline

### 4.1 Initialization Phase

| Step | Component | Process | Output |
|------|-----------|---------|--------|
| 1 | Launch System | Parse parameters, load URDF, configure controllers | Loaded controller manager |
| 2 | Controller Manager | Load plugins via pluginlib, initialize controllers | Configured controller chain |
| 3 | Admittance Controller | Configure interfaces, TF, kinematics | Initialized controller |

**Key Code:**
```cpp
// In on_configure():
// 1. Load parameters
param_listener_ = std::make_shared<ur_admittance_controller::ParamListener>(get_node());
// 2. Set up TF
tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_node()->get_clock());
// 3. Initialize kinematics
loadKinematics();
```

### 4.2 Runtime Pipeline

#### 4.2.1 Sensor Data Acquisition

| Aspect | Specification |
|--------|---------------|
| **Source** | F/T sensor (real) or `wrench_signal_generator` (simulation) |
| **Topic** | `/wrench` |
| **Message Type** | `geometry_msgs::msg::WrenchStamped` |
| **Frame** | Tool frame (e.g., `tool0`) |
| **Update Rate** | ~125-500 Hz (hardware dependent) |
| **Data Format** | 6D vector (3D force, 3D torque) |
| **Units** | Force [N], Torque [Nm] |

**Key Code:**
```cpp
// In wrenchCallback():
wrench_buffer_.writeFromNonRT(std::move(*msg));
```

#### 4.2.2 Reference Frame Transformation

| Aspect | Specification |
|--------|---------------|
| **Process** | Transform wrench from tool frame to base frame |
| **Library** | TF2 |
| **Input Frame** | Tool frame (`tool0`) |
| **Output Frame** | Base frame (`base_link`) |
| **Mathematical Operation** | Force-Torque transformation |

**Mathematical Operations:**
```
force_base = R * force_tool
torque_base = R * torque_tool + p × (R * force_tool)
```
where:
- R = Rotation matrix from tool to base
- p = Position vector from base to tool
- × = Cross product

**Key Code:**
```cpp
// In transformWrench():
geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
  base_frame_id_, tool_frame_id_, tf2::TimePointZero);
```

#### 4.2.3 Admittance Control Calculation

| Aspect | Specification |
|--------|---------------|
| **Process** | Convert external forces to motion via admittance law |
| **Update Rate** | Controller frequency (~100-500 Hz) |
| **Input** | 6D wrench in base frame |
| **Output** | 6D Cartesian velocity |
| **Parameters** | Mass matrix (M), Damping matrix (D), Stiffness matrix (K) |
| **Algorithm** | Second-order mechanical system solver |

**Admittance Equation:**
```
M * a + D * v + K * x = F_ext
```

**Calculation Steps:**
1. Read external wrench (F_ext)
2. Calculate desired acceleration: a = M⁻¹ * (F_ext - D*v - K*x)
3. Integrate to get velocity: v = v + a * dt
4. Apply deadband filter to remove noise

**Key Code:**
```cpp
// In calculateAdmittance():
desired_acceleration_ = mass_matrix_.inverse() * 
  (wrench_external_ - damping_matrix_ * desired_velocity_ - 
   stiffness_matrix_ * pose_error_);
   
// Integration
desired_velocity_ += desired_acceleration_ * period.seconds();
```

#### 4.2.4 Inverse Kinematics

| Aspect | Specification |
|--------|---------------|
| **Process** | Convert Cartesian velocity to joint velocity |
| **Library** | KDL (Kinematics and Dynamics Library) |
| **Input** | 6D Cartesian velocity |
| **Output** | 6D joint velocity (one per joint) |
| **Algorithm** | Jacobian pseudo-inverse |

**Mathematical Operation:**
```
q̇ = J⁻¹(q) * v
```
where:
- q̇ = Joint velocity vector
- J⁻¹ = Pseudo-inverse of the Jacobian matrix
- v = Cartesian velocity vector

**Key Code:**
```cpp
// In update_and_write_commands():
kinematics_->convert(cart_vel_cmd_, joint_velocities_cmd_);
```

#### 4.2.5 Trajectory Generation

| Aspect | Specification |
|--------|---------------|
| **Process** | Create smooth joint trajectory from velocities |
| **Input** | Joint velocities |
| **Output** | Joint trajectory |
| **Message Type** | `trajectory_msgs::msg::JointTrajectory` |
| **Parameters** | Time horizon, position/velocity limits |

**Trajectory Generation Steps:**
1. Start from current joint positions
2. Forward integrate velocities: q_future = q_current + q̇ * dt
3. Create waypoints with timestamps
4. Apply velocity and acceleration limits
5. Package into trajectory message

**Key Code:**
```cpp
// In sendTrajectory():
trajectory_msgs::msg::JointTrajectory trajectory;
trajectory.joint_names = params_.joints;

// Add points at different time steps
for (size_t i = 0; i < num_points; ++i) {
  trajectory_msgs::msg::JointTrajectoryPoint point;
  // Calculate positions and velocities for each point
  trajectory.points.push_back(point);
}
```

#### 4.2.6 Trajectory Execution

| Aspect | Specification |
|--------|---------------|
| **Process** | Send trajectory to joint trajectory controller |
| **Interface** | ROS2 Action |
| **Action Type** | `control_msgs::action::FollowJointTrajectory` |
| **Client** | `ur_admittance_controller` |
| **Server** | `scaled_joint_trajectory_controller` |

**Key Code:**
```cpp
// In sendTrajectory():
auto goal_msg = FollowJointTrajectory::Goal();
goal_msg.trajectory = trajectory;
goal_future_ = action_client_->async_send_goal(goal_msg);
```

#### 4.2.7 Hardware Command Execution

| Aspect | Specification |
|--------|---------------|
| **Process** | Convert trajectory into hardware commands |
| **Component** | `scaled_joint_trajectory_controller` |
| **Hardware Interface** | Position or velocity command interfaces |
| **Target** | UR hardware (real) or Gazebo (simulation) |

## 5. Controller Parameters

| Parameter | Type | Description | Default |
|-----------|------|-------------|---------|
| `joints` | string[] | Joint names to control | `["shoulder_pan_joint", ...]` |
| `mass_matrix` | double[36] | 6x6 mass matrix for admittance | Identity |
| `damping_matrix` | double[36] | 6x6 damping matrix | Diagonal matrix |
| `stiffness_matrix` | double[36] | 6x6 stiffness matrix | Zero matrix |
| `wrench_deadband` | double[6] | Minimum wrench to respond to | `[0.1, 0.1, 0.1, 0.1, 0.1, 0.1]` |
| `controller_name` | string | Name of trajectory controller | `scaled_joint_trajectory_controller` |

## 6. Implementation Details

### 6.1 Controller State Machine

The controller follows the ROS2 Controller lifecycle:

```
      on_configure()
Unconfigured ──────────────► Configured
      │                        │
      │                        │ on_activate()
      │                        ▼
      │                      Active
      │                        │
      │                        │ on_deactivate()
      │                        ▼
      └───────────────── Inactive
```

### 6.2 Error Handling

| Error Condition | Handling Strategy |
|-----------------|-------------------|
| TF Lookup Failure | Retry with exponential backoff |
| Trajectory Execution Failure | Retry with new trajectory |
| Kinematics Error | Use fallback mode (Cartesian control) |

## 7. Testing & Validation

### 7.1 Simulation Testing

Use the `wrench_signal_generator` to publish simulated force/torque data:

```bash
ros2 run ur_admittance_controller wrench_signal_generator
```

### 7.2 Hardware Testing

For real robot testing, ensure proper configuration of the F/T sensor:

```bash
ros2 launch ur_admittance_controller ur_admittance_controller.launch.py use_sim:=false robot_ip:=<ROBOT_IP>
```

## 8. References

1. [ROS2 Control Framework](https://control.ros.org/)
2. [Admittance Control Theory](https://doi.org/10.1109/ROBOT.2000.844146)
3. [Universal Robots ROS2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
