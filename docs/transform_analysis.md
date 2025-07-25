# Transform Chain Analysis - Critical Finding

## The Issue
Our implementation computes probe-to-base transform as:
```cpp
Transform X_BP = X_EB_.inverse() * Transform(calibration_params_.R_SE) * 
                 tf2::transformToEigen(tf_buffer_->lookupTransform(
                     frames::PROBE_FRAME, frames::SENSOR_FRAME, tf2::TimePointZero));
```

## Analysis Result from Perplexity

**Our transform chain is MATHEMATICALLY CORRECT!**

The chain `X_BP = X_EB.inverse() * Transform(R_SE) * X_PS` correctly computes:
- `X_EB.inverse()` = T_BE (base to end-effector)
- `Transform(R_SE)` = T_SE (sensor to end-effector)  
- `lookupTransform(PROBE, SENSOR)` = T_PS (probe to sensor)

Chain: T_BP = T_BE ∘ T_SE ∘ T_PS (base → end-effector → sensor → probe)

## BUT - Critical Implementation Issue

While the transform chain is correct, we're NOT using the adjoint transformation!

**Current (WRONG):**
```cpp
ft_proc_b_ = algorithms::transformWrenchToBase(wrench_probe, X_BP);
```

**Should be:**
```cpp
// Wrench transformation requires adjoint!
Eigen::Matrix<double,6,6> Ad = computeAdjoint(X_BP.inverse());
ft_proc_b_ = Ad.transpose() * wrench_probe;
```

## Why This Matters

For wrench transformation:
- Force: `f_B = R * f_P` (rotation only)
- Torque: `τ_B = R * τ_P + p × (R * f_P)` (includes cross-term!)

Without adjoint, we're missing the critical `p × f` term that accounts for:
- Lever arm effects
- Force-induced torques at different locations

## Example
Force of 10N in Z at probe (1m from base):
- Without adjoint: τ_base = 0 (WRONG)
- With adjoint: τ_base = 10 Nm (CORRECT)

## The Fix

Check what `algorithms::transformWrenchToBase()` does:
- If it uses adjoint internally → We're OK
- If it just rotates → We have a MAJOR BUG

## Comparison with Pulse

Pulse doesn't publish base frame wrench, so they avoid this issue entirely. They only do:
```cpp
ft_proc_p = adjoint_p_s * ft_proc_s;  // Sensor to probe (with adjoint)
```

## Action Items

1. **URGENT**: Verify `algorithms::transformWrenchToBase()` implementation
2. If it doesn't use adjoint, fix immediately
3. Add unit tests for wrench transformation with non-zero translations

## Conclusion

Our transform chain math is correct, but we may be applying it wrong for wrenches. This could cause serious control issues if forces are applied far from the base.