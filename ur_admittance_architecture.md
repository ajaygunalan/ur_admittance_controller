# UR Admittance Controller: Architecture Documentation

## 1. Overview

The UR Admittance Controller is a ROS2 controller that implements cartesian admittance control for Universal Robots manipulators. It enables compliant motion in response to external forces, transforming force/torque measurements into robot motion.

```
┌───────────────┐    ┌───────────────────────┐    ┌────────────────────┐    ┌───────────┐
│ Force/Torque  │    │                       │    │                    │    │           │
│ Sensor        ├───►│  Admittance Control   ├───►│ Joint Trajectory   ├───►│   Robot   │
│ (Real/Sim)    │    │  Controller           │    │ Controller         │    │           │
└───────────────┘    └───────────────────────┘    └────────────────────┘    └───────────┘
      Wrench                Cartesian                Joint Commands             Motion
                            Velocity
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

### 3.1 Data Flow Pipeline

The controller processes data through the following sequential pipeline:

```
┌───────────────┐     ┌───────────────┐     ┌───────────────┐     ┌───────────────┐
│  F/T Sensor   │     │    Frame      │     │  Admittance   │     │    Inverse    │
│     Data      │────►│ Transformation│────►│  Equations    │────►│  Kinematics   │
│ (Tool Frame)  │     │ (to Base)     │     │  M·a+D·v+K·x=F│     │  (Jacobian)   │
└───────────────┘     └───────────────┘     └───────────────┘     └───────────────┘
                                                                           │
                                                                           ▼
┌───────────────┐     ┌───────────────┐     ┌───────────────┐     ┌───────────────┐
│    Robot      │     │   Joint       │     │  Trajectory   │     │    Joint      │
│   Motion      │◄────│  Trajectory   │◄────│  Generation   │◄────│  Velocities   │
│  Execution    │     │  Controller   │     │               │     │               │
└───────────────┘     └───────────────┘     └───────────────┘     └───────────────┘
```

### 3.2 Data Types and Transformations

| Stage | Input | Output | Library/API |
|-------|-------|--------|-------------|
| Frame Transformation | `WrenchStamped` (tool) | `Wrench` (base) | TF2 |
| Admittance Control | `Wrench` | `Twist` (Cartesian velocity) | Eigen |
| Inverse Kinematics | `Twist` | `JointVelocities` | KDL |
| Trajectory Gen | `JointVelocities` | `JointTrajectory` | realtime_tools |
| Control | `JointTrajectory` | Joint commands | controller_interface |

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

### 4.2 Admittance Control Implementation

The core algorithm implements a second-order dynamic system:

```
M·a + D·v + K·x = F_ext
```

Where:
- M: Mass matrix (6×6, diagonal)
- D: Damping matrix (6×6, diagonal)
- K: Stiffness matrix (6×6, diagonal)
- a: Cartesian acceleration
- v: Cartesian velocity
- x: Position error
- F_ext: External force/torque

#### Implementation Steps:

1. **Acquire Force Data**: Read wrench data from sensor or simulation
2. **Frame Transformation**: Convert from tool to base frame
3. **Solve Admittance Equation**: `a = M⁻¹·(F_ext - D·v - K·x)`
4. **Velocity Integration**: `v = v + a·dt`
5. **Apply Limits**: Enforce velocity and acceleration limits
6. **Inverse Kinematics**: Convert to joint velocities
7. **Trajectory Generation**: Create smooth joint trajectories

### 4.3 Critical Data Structures

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
