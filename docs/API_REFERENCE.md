# UR Admittance Controller API Reference

## Controller Interface

### AdmittanceController Class

The main controller class that implements force-based compliant motion control for Universal Robots.

```cpp
class AdmittanceController : public controller_interface::ChainableControllerInterface
```

#### Key Methods

##### Lifecycle Methods
- `on_init()` - Initialize controller name and parameters
- `on_configure()` - Set up kinematics, allocate memory, initialize interfaces  
- `on_activate()` - Start real-time control, reset integrators
- `on_deactivate()` - Stop control gracefully
- `on_cleanup()` - Release resources

##### Control Methods
- `update_reference_from_subscribers()` - Process external pose/force commands
- `update_and_write_commands()` - Main control loop (called at control rate)
- `reference_interfaces_()` - Export joint position references to downstream controller

##### Service Callbacks
- `resetDesiredPoseCallback()` - Set desired pose to current pose
- `moveToStartPoseCallback()` - Enable safe impedance mode transition

## Parameter Interface

Parameters are defined in `config/admittance_config.yaml` and accessed through the generated parameter library.

### Admittance Parameters
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `admittance.mass` | double[6] | [10,10,10,1,1,1] | Virtual mass [kg, kg⋅m²] |
| `admittance.damping_ratio` | double[6] | [0.8,0.8,0.8,0.8,0.8,0.8] | Damping ratio (ζ) |
| `admittance.stiffness` | double[6] | [0,0,0,0,0,0] | Spring stiffness [N/m, Nm/rad] |
| `admittance.min_motion_threshold` | double | 1.5 | Force threshold to start motion [N] |

### Kinematics Parameters
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `kinematics.plugin` | string | "kinematics_interface_kdl/KDLKinematics" | Kinematics solver plugin |
| `kinematics.base_link` | string | "base_link" | Robot base frame |
| `kinematics.tip_link` | string | "tool0" | End-effector frame |

### Safety Parameters
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `safety.max_linear_velocity` | double | 0.5 | Max TCP velocity [m/s] |
| `safety.max_angular_velocity` | double | 1.0 | Max TCP angular velocity [rad/s] |
| `safety.max_linear_acceleration` | double | 2.0 | Max TCP acceleration [m/s²] |
| `safety.max_angular_acceleration` | double | 5.0 | Max angular acceleration [rad/s²] |

## Topic Interface

### Published Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/ur_admittance_controller/cartesian_velocity_command` | `geometry_msgs/TwistStamped` | Current TCP velocity |
| `/ur_admittance_controller/current_pose` | `geometry_msgs/PoseStamped` | Current TCP pose |
| `/ur_admittance_controller/desired_pose` | `geometry_msgs/PoseStamped` | Target pose (impedance mode) |
| `/ur_admittance_controller/pose_error` | `geometry_msgs/TwistStamped` | Position/orientation error |

### Subscribed Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/ft_sensor_readings` | `geometry_msgs/WrenchStamped` | Force/torque sensor data |
| `/ur_admittance_controller/set_desired_pose` | `geometry_msgs/PoseStamped` | Set target pose |

## Service Interface

| Service | Type | Description |
|---------|------|-------------|
| `/ur_admittance_controller/reset_desired_pose` | `std_srvs/Trigger` | Set desired = current pose |
| `/ur_admittance_controller/move_to_start_pose` | `std_srvs/Trigger` | Safe impedance activation |

## Data Types

### AdmittanceState
Internal state structure for real-time computations:
```cpp
struct AdmittanceState {
    Eigen::Isometry3d pose;          // Current TCP pose
    Vector6d velocity;               // Current TCP velocity  
    Vector6d acceleration;           // Current TCP acceleration
    Vector6d force;                  // External force/torque
    std::array<double, 6> joint_positions;
    std::array<double, 6> joint_velocities;
};
```

### Vector6d
6D spatial vector for forces, velocities, etc:
```cpp
using Vector6d = Eigen::Matrix<double, 6, 1>;
// Format: [rotation_x, rotation_y, rotation_z, translation_x, translation_y, translation_z]
```

## Transform Utilities

### Matrix Operations (matrix_utilities.hpp)
- `skewSymmetric(Vector3d)` - Create skew-symmetric matrix from vector
- `computeAdjoint(Isometry3d)` - Compute 6x6 adjoint matrix for force/velocity transform
- `poseTwistFromPoseError(Isometry3d, Isometry3d)` - Compute pose error as twist

### Safety Functions  
- `clampVelocity(Vector6d, limits)` - Apply velocity limits
- `clampAcceleration(Vector6d, Vector6d, dt, limits)` - Apply acceleration limits
- `checkNaN(Vector6d)` - Validate numerical stability

## Example Usage

### Python Test Script
```python
import rclpy
from geometry_msgs.msg import WrenchStamped

# Apply 10N force in X direction
wrench_pub = node.create_publisher(WrenchStamped, '/ft_sensor_readings', 10)
msg = WrenchStamped()
msg.header.frame_id = 'tool0'
msg.wrench.force.x = 10.0
wrench_pub.publish(msg)
```

### Parameter Configuration
```yaml
# Enable impedance mode with moderate stiffness
ros2 param set /ur_admittance_controller admittance.stiffness [100,100,100,10,10,10]

# Increase damping for stability
ros2 param set /ur_admittance_controller admittance.damping_ratio [1.0,1.0,1.0,1.0,1.0,1.0]

# Lower force threshold for sensitive tasks
ros2 param set /ur_admittance_controller admittance.min_motion_threshold 0.5
```

## Thread Safety

The controller uses several mechanisms to ensure real-time safety:
- Pre-allocated memory buffers
- Lock-free realtime publishers  
- Atomic transform caches
- No dynamic memory allocation in control loop

## Performance Metrics

Typical performance on RT-patched kernel:
- Control loop: <1ms @ 500Hz
- Force transform: ~0.1ms
- Kinematics computation: ~0.5ms  
- Total latency: <2ms from force to motion