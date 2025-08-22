# Calibration Algorithm Fix Plan

## Summary
**2 bugs** to fix in `src/wrench_calibration_algorithm.cpp`

---

## Bug Fixes Required

### Fix 1: Correlation Matrix Order (Line 68)

**Current (WRONG):**
```cpp
D += rotation_centered * gravity_in_base.N * force_centered.transpose();
```

**Fixed (CORRECT):**
```cpp
D += force_centered * (rotation_centered * gravity_in_base.N).transpose();
```

**Why:** Pseudocode requires `D = Σ (F_sensor - mean) × ((R - mean) × F_b)^T`

---

### Fix 2: SVD Determinant (Lines 73-77)

**Current (WRONG):**
```cpp
Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
const double det = (svd.matrixU() * svd.matrixV().transpose()).determinant();
if (det < 0) {
    correction(2, 2) = -1;
}
```

**Fixed (CORRECT):**
```cpp
Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
correction(2, 2) = svd.matrixU().determinant() * svd.matrixV().determinant();
```

**Why:** Pseudocode specifies `diag(1, 1, det(U)×det(V))`

---

## Implementation Steps

1. Open `src/wrench_calibration_algorithm.cpp`
2. Go to line 68 - fix correlation matrix order
3. Go to lines 73-77 - fix SVD determinant calculation
4. Build and test

---

## No Changes Needed

✅ LROM scaling (line 29) - Correct  
✅ Rotation centering (line 67) - Correct  
✅ Force bias (line 82) - Correct  
✅ Matrix A construction - Correct  

---

## Testing

```bash
# Build
cd ~/ros2_ws
colcon build --packages-select ur_admittance_controller

# Run calibration
ros2 run ur_admittance_controller wrench_calibration_node

# Verify:
# - Rotation matrix is orthogonal
# - Determinant = 1
# - Zero force with no contact
```

---

*Plan Date: 2025-08-22*  
*Status: Ready to implement*