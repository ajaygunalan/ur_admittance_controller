# UR Admittance Controller - Notation Guide

This document defines the coordinate frame notation used throughout the UR Admittance Controller, following Drake's systematic notation principles to prevent coordinate frame errors.

## Core Principles

All spatial quantities (positions, rotations, forces, velocities) must specify:
1. **What** is being measured
2. **From where** it's measured  
3. **In which frame** it's expressed

## Coordinate Frames

### Primary Frames Used

| Frame | Symbol | Description | Fixed/Moving |
|-------|--------|-------------|--------------|
| World | `W` | Inertial world frame (optional, often same as base) | Fixed |
| Base | `base` | Robot base frame (`base_link` in URDF) | Fixed |
| Tip | `tip` | End-effector frame (`tool0` in URDF) | Moving |
| Sensor | `ft` | Force/torque sensor frame | Moving |

### Frame Relationships

```
World (W) [optional]
    |
    └── Base (base) [base_link]
            |
            └── Robot Kinematic Chain
                    |
                    └── Tip (tip) [tool0]
                            |
                            └── F/T Sensor (ft) [if different from tip]
```

## Notation Reference

### Position Vectors
- **Mathematical**: $^{base}p_{tip}$ = position of tip from base
- **Code**: `p_base_tip`
- **With expression frame**: `p_base_tip_base` = position of tip from base, expressed in base frame

### Rotation Matrices  
- **Mathematical**: $^{base}R_{tip}$ = orientation of tip frame relative to base frame
- **Code**: `R_base_tip`
- **Property**: `v_base = R_base_tip * v_tip` (transforms vector from tip to base frame)

### Rigid Transforms (Poses)
- **Mathematical**: $^{base}X_{tip}$ = pose of tip frame in base frame
- **Code**: `X_base_tip`
- **Combines**: rotation `R_base_tip` and position `p_base_tip`

### Spatial Vectors (6D)
Format: `[rotational (0-2), translational (3-5)]`
- **Velocity**: `V_base_tip_base` = spatial velocity of tip in base, expressed in base
- **Force**: `F_tip_base` = wrench applied at tip, expressed in base frame

## Transform Table

### Required Transforms in Controller

| Transform | Variable Name | Description | Update Frequency |
|-----------|--------------|-------------|------------------|
| `X_base_tip` | `transform_base_tip_` | End-effector pose in base frame | RT loop (500Hz) |
| `X_base_ft` | `transform_base_ft_` | F/T sensor pose in base frame | RT loop (500Hz) |
| `X_W_base` | `transform_world_base_` | Base in world (if world ≠ base) | Once at startup |

### Transform Composition Rules

Valid compositions (adjacent frames must match):
- ✅ `X_W_tip = X_W_base * X_base_tip`
- ✅ `X_base_ft = X_base_tip * X_tip_ft`
- ❌ `X_W_tip = X_W_base * X_tip_ft` (Invalid: base ≠ tip)

## Variable Naming Convention

### Current Controller Variables → Proper Names

| Current Name | Proper Name | Type | Description |
|--------------|-------------|------|-------------|
| `ee_transform_cache_` | `transform_base_tip_` | `Eigen::Isometry3d` | Tip pose in base frame |
| `ft_transform_cache_` | `transform_base_ft_` | `Eigen::Isometry3d` | F/T sensor pose in base frame |
| `current_pose_` | `X_base_tip_current_` | `Eigen::Isometry3d` | Current tip pose in base |
| `desired_pose_` | `X_base_tip_desired_` | `Eigen::Isometry3d` | Desired tip pose in base |
| `pose_error_` | `error_tip_base_` | `Vector6d` | Pose error [rot, trans] in base |
| `wrench_` | `F_sensor_base_` | `Vector6d` | Force from sensor in base frame |
| `wrench_filtered_` | `F_sensor_base_filtered_` | `Vector6d` | Filtered force in base frame |
| `cart_twist_` | `V_base_tip_base_` | `Vector6d` | Tip velocity in base frame |
| `desired_vel_` | `V_base_tip_desired_base_` | `Vector6d` | Desired tip velocity in base |
| `velocity_error_` | `V_error_tip_base_` | `Vector6d` | Velocity error in base frame |

### Method Naming Convention

Methods should indicate the frames they operate in:

| Current Name | Proper Name | Description |
|--------------|-------------|-------------|
| `computePoseError()` | `computePoseError_tip_base()` | Computes error of tip in base frame |
| `updateTransformCaches()` | `updateTransforms_base()` | Updates transforms to base frame |
| `computeAdmittanceControl()` | `computeAdmittanceControl_base()` | Computes control in base frame |

## Force/Torque Sensor Special Case

Since the F/T sensor outputs forces already measured with respect to base frame:

```cpp
// Special case: sensor already outputs in base frame
if (params_.ft_frame == params_.base_link) {
    F_sensor_base_ = raw_wrench;  // No transformation needed
} else {
    // Transform from sensor frame to base frame
    F_sensor_base_ = transform_base_ft_.adjoint * F_sensor_ft_;
}
```

## Admittance Control Equations in Notation

The admittance control law with proper notation:

```
M * A_base_tip_desired_base + D * V_base_tip_base + K * error_tip_base = F_sensor_base
```

Where:
- `M`: Mass matrix (6x6)
- `D`: Damping matrix (6x6)  
- `K`: Stiffness matrix (6x6)
- `A_base_tip_desired_base`: Desired acceleration of tip in base frame
- `V_base_tip_base`: Current velocity of tip in base frame
- `error_tip_base`: Position/orientation error in base frame
- `F_sensor_base`: External force measured at sensor, expressed in base frame

## Common Pitfalls to Avoid

1. **Missing frame information**: Never use generic names like `pose` or `velocity`
2. **Frame mismatches**: Always verify adjacent frames match in multiplications
3. **Expression frame ambiguity**: Always specify which frame vectors are expressed in
4. **Assuming world = base**: Be explicit even if they're the same frame

## Validation Checklist

Before committing code changes:
- [ ] All spatial variables have frame suffix (e.g., `_base`, `_tip`)
- [ ] All transforms specify both frames (e.g., `X_base_tip`)
- [ ] Method names indicate operating frame when relevant
- [ ] Transform compositions have matching adjacent frames
- [ ] Comments clarify frame relationships for complex operations