# Notation Guide

## Frame Convention
All spatial quantities specify: **what**, **from where**, **in which frame**

### Coordinate Frames
| Frame | Symbol | Description |
|-------|--------|-------------|
| Base | `base` | Robot base (`base_link`) |
| Tip | `tip` | End-effector (`tool0`) |
| Sensor | `ft` | Force/torque sensor |

### Variable Naming
```cpp
// Position: what_from_to_expressedIn
Vector3d p_base_tip_base;        // Tip position from base, in base frame

// Rotation: from_to  
Matrix3d R_base_tip;             // Tip orientation relative to base

// Transform: from_to
Isometry3d X_base_tip;           // Tip pose in base frame

// Velocity: frame_body_expressedIn
Vector6d V_base_tip_base;        // Tip velocity in base, expressed in base

// Force: measured_expressedIn
Vector6d F_sensor_base;          // Sensor force expressed in base frame
```

## Transform Rules
- **Valid**: `X_W_tip = X_W_base * X_base_tip` (adjacent frames match)
- **Invalid**: `X_W_tip = X_W_base * X_tip_ft` (base ≠ tip)

## Method Naming
```cpp
void computePoseError_tip_base();      // Error of tip in base frame
void updateTransforms_base();          // Update transforms to base
void computeAdmittance_base();         // Admittance in base frame
```

## Admittance Equation
```
M * A_tip_desired_base + D * V_tip_base + K * error_tip_base = F_sensor_base
```

## Common Variables
| Current | Proper | Description |
|---------|--------|-------------|
| `current_pose_` | `X_base_tip_current_` | Current tip pose |
| `wrench_` | `F_sensor_base_` | Force in base frame |
| `cart_twist_` | `V_base_tip_base_` | Tip velocity |
| `pose_error_` | `error_tip_base_` | Pose error |