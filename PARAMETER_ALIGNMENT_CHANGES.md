# Parameter Alignment Changes

## Summary
Aligned parameters with ur3_admittance_controller to ensure consistent behavior.

## Changes Made

### 1. Mass Matrix (M)
- **Before**: [8.0, 8.0, 8.0, 0.8, 0.8, 0.8]
- **After**: [2.0, 2.0, 2.0, 2.0, 2.0, 2.0]
- **Rationale**: 4x reduction in translational mass, 2.5x increase in rotational inertia

### 2. Stiffness Matrix (K)
- **Before**: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] (pure admittance)
- **After**: [10.0, 20.0, 10.0, 10.0, 10.0, 10.0] (impedance mode)
- **Rationale**: Match ur3's impedance control behavior

### 3. Workspace Limits
- **Before**: [-1.0, 1.0, -1.0, 1.0, 0.0, 1.5] m
- **After**: [-0.5, 0.5, -0.5, 0.5, 0.0, 0.7] m
- **Rationale**: More realistic workspace for UR robots

### 4. Maximum Linear Velocity
- **Before**: 0.5 m/s
- **After**: 1.5 m/s
- **Rationale**: 3x increase to match ur3's capabilities

### 5. Maximum Acceleration
- **Before**: 1.0 m/s²
- **After**: 1.0 m/s² (unchanged)
- **Rationale**: Already matched

## Impact on Behavior

1. **Faster Response**: Lower mass = quicker acceleration for same force
2. **Spring Behavior**: Non-zero stiffness creates position-keeping behavior
3. **Smaller Workspace**: More conservative safety limits
4. **Higher Speed**: Allows faster movements (3x)

## Damping Consideration

With new parameters:
- M = 2.0, K = 10.0, ζ = 0.8
- Computed damping: D = 2 × 0.8 × √(2.0 × 10.0) ≈ 7.16

This is closer to ur3's explicit damping of 12.0, but still lower.

## Testing Notes

After these changes:
1. Robot will feel "lighter" (lower mass)
2. Robot will try to return to equilibrium position (spring effect)
3. Movement restricted to smaller workspace
4. Can command faster velocities