# Comprehensive Parameter Comparison: ur_admittance_controller vs ur3_admittance_controller

## Executive Summary

This document provides an exhaustive comparison of all parameters between our ROS2 `ur_admittance_controller` and the ROS1 `ur3_admittance_controller`, including M (mass), D (damping), K (stiffness), workspace limits, and all other control parameters.

## 1. Mass Matrix (M) Comparison

| Parameter | ur_admittance_controller | ur3_admittance_controller | Discrepancy |
|-----------|-------------------------|---------------------------|-------------|
| **Translation (X,Y,Z)** | 8.0, 8.0, 8.0 kg | 2.0, 2.0, 2.0 kg | **4x HIGHER** |
| **Rotation (Rx,Ry,Rz)** | 0.8, 0.8, 0.8 kg·m² | 2.0, 2.0, 2.0 kg·m² | **2.5x LOWER** |
| **Matrix Type** | Diagonal only | Full 36-element matrix (diagonal) | Same approach |

### Analysis:
- **Major discrepancy**: Our translational masses are 4x higher (8kg vs 2kg)
- **Major discrepancy**: Our rotational inertias are 2.5x lower (0.8 vs 2.0)
- ur3 has alternative config with M=[6,6,6,1,1,0.5] for real robot

## 2. Damping Matrix (D) Comparison

| Component | ur_admittance_controller | ur3_admittance_controller | Discrepancy |
|-----------|-------------------------|---------------------------|-------------|
| **Damping Calculation** | D = 2ζ√(MK) | Direct specification | **Different approach** |
| **Damping Ratio (ζ)** | 0.8 for all axes | N/A | - |
| **D_coupling** | Not implemented | [0,0,0,0,0,0] (all zeros) | Missing feature |
| **D_arm (computed)** | ~0 (when K=0) | [12,12,12,10,10,10] | **MAJOR DIFFERENCE** |

### Analysis:
- **Critical difference**: ur3 has explicit damping values while ours are computed
- When K=0 (pure admittance), our D≈0, but ur3 has D=[12,12,12,10,10,10]
- ur3 separates coupling damping (D_coupling) from arm damping (D_arm)

## 3. Stiffness Matrix (K) Comparison

| Parameter | ur_admittance_controller | ur3_admittance_controller | Discrepancy |
|-----------|-------------------------|---------------------------|-------------|
| **Default K** | [0,0,0,0,0,0] | [10,20,10,10,10,10] | **MAJOR DIFFERENCE** |
| **Mode** | Pure admittance | Impedance control | Different default |
| **Range** | 0-10000 N/m | Hardcoded values | More flexible |

### Analysis:
- **Major discrepancy**: We default to pure admittance (K=0), they default to impedance
- ur3 uses different stiffness for Y-axis (20 vs 10)
- We allow runtime changes, theirs is fixed in config

## 4. Workspace Limits Comparison

| Axis | ur_admittance_controller | ur3_admittance_controller | Discrepancy |
|------|-------------------------|---------------------------|-------------|
| **X limits** | [-1.0, 1.0] m | [-0.5, 0.5] m | **2x LARGER** |
| **Y limits** | [-1.0, 1.0] m | [-0.5, 0.5] m | **2x LARGER** |
| **Z limits** | [0.0, 1.5] m | [0.0, 0.7] m | **2.1x HIGHER** |

### Analysis:
- **Major discrepancy**: Our workspace is significantly larger in all dimensions
- ur3 has commented conservative option: [-0.3,0.3, 0.35,0.55, 0.2,0.75]

## 5. Velocity/Acceleration Limits Comparison

| Parameter | ur_admittance_controller | ur3_admittance_controller | Discrepancy |
|-----------|-------------------------|---------------------------|-------------|
| **Max Linear Velocity** | 0.5 m/s | 1.5 m/s | **3x LOWER** |
| **Max Acceleration** | 1.0 m/s² | 1.0 m/s² | Same |
| **Angular Velocity Limit** | Not implemented | Not implemented | Same |

### Analysis:
- **Major discrepancy**: Our velocity limit is 3x more conservative
- Both use same acceleration limit

## 6. Equilibrium Position Comparison

| Component | ur_admittance_controller | ur3_admittance_controller | Discrepancy |
|-----------|-------------------------|---------------------------|-------------|
| **Position** | [0.1, 0.4, 0.5] m | [0.1, 0.4, 0.5] m | **IDENTICAL** |
| **Orientation** | [0,0,0,1] (identity) | [-0.594,-0.567,-0.369,0.436] | **DIFFERENT** |

### Analysis:
- Position equilibrium is identical
- ur3 uses non-identity orientation (corresponds to specific joint config)
- ur3 real robot config: [0.111,0.494,0.587] with different orientation

## 7. Force/Torque Sensor Parameters

| Parameter | ur_admittance_controller | ur3_admittance_controller | Discrepancy |
|-----------|-------------------------|---------------------------|-------------|
| **Filter Coefficient** | 0.8 | 1.0 (no filtering) | More filtering |
| **Force Dead Zone** | Not implemented | 0.0 (configurable) | **Missing feature** |
| **Torque Dead Zone** | Not implemented | 0.0 (configurable) | **Missing feature** |
| **Min Motion Threshold** | 1.5 N/Nm | Not implemented | Extra feature |

## 8. Additional Parameters

| Parameter | ur_admittance_controller | ur3_admittance_controller | Discrepancy |
|-----------|-------------------------|---------------------------|-------------|
| **Admittance Ratio** | 1.0 (hardcoded) | Topic-based (0-1) | **Less flexible** |
| **Control Wrench Input** | Not implemented | Supported | **Missing feature** |
| **DS Velocity Input** | Not implemented | Supported | **Missing feature** |
| **Drift Reset** | 0.001 m/s threshold | Not implemented | Extra feature |

## 9. Summary of Major Discrepancies

### Critical Differences:
1. **Mass**: 4x higher translational, 2.5x lower rotational
2. **Damping**: Computed (often ~0) vs explicit values [12,12,12,10,10,10]
3. **Stiffness**: Pure admittance (0) vs impedance mode (10-20)
4. **Velocity Limit**: 3x more conservative (0.5 vs 1.5 m/s)
5. **Workspace**: 2x larger in XY, 2.1x in Z

### Missing Features in ur_admittance_controller:
- Force/torque dead zones
- Control wrench input
- DS velocity integration
- Separate coupling dynamics
- Runtime admittance ratio adjustment

### Better in ur_admittance_controller:
- Configurable parameters via ROS2 params
- Force filtering implementation
- Drift reset capability
- Min motion threshold

## 10. Recommendations

### High Priority Fixes:
1. **Damping Issue**: When K=0, our D≈0 which may cause instability. Consider:
   ```yaml
   damping_absolute: [12.0, 12.0, 12.0, 10.0, 10.0, 10.0]  # Add absolute damping option
   ```

2. **Mass Discrepancy**: Our masses seem too high for UR robots:
   ```yaml
   mass: [2.0, 2.0, 2.0, 2.0, 2.0, 2.0]  # More realistic for UR5e
   ```

3. **Velocity Limit**: Consider increasing to match ur3:
   ```cpp
   arm_max_vel_ = 1.5;  // Instead of 0.5
   ```

### Parameter Alignment Suggestions:
```yaml
# Suggested aligned parameters
ur_admittance_controller:
  admittance:
    mass: [2.0, 2.0, 2.0, 2.0, 2.0, 2.0]
    stiffness: [10.0, 20.0, 10.0, 10.0, 10.0, 10.0]  # For impedance mode
    damping_absolute: [12.0, 12.0, 12.0, 10.0, 10.0, 10.0]  # New parameter
    
  workspace_limits: [-0.5, 0.5, -0.5, 0.5, 0.0, 0.7]  # Match ur3
  max_velocity: 1.5  # Match ur3
```

## Conclusion

There are significant discrepancies in fundamental parameters between the two controllers. The most critical issues are:
1. Different mass values (4x for translation)
2. Damping computation leading to potential instability
3. Different default modes (admittance vs impedance)
4. Conservative velocity limits in our implementation

These differences could lead to very different robot behavior and should be carefully reviewed based on your specific application requirements.