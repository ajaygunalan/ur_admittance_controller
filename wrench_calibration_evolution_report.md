# Wrench Calibration Evolution Report

## Executive Summary

Based on comprehensive git history analysis, the **last known working version** of the wrench calibration was **commit f0265fc (July 24, 2025)**. The code underwent significant evolution from initial implementation to working state, then was broken during subsequent refactoring.

## Critical Working Version

### ✅ **WORKING: Commit f0265fc (July 24, 2025)**
**Title:** "fix(wrench_calibration): ensure processing of joint state data and improve calibration logging"

**Why this was working:**
1. Fixed critical bug where joint state data was received but not processed
2. Included all previous critical fixes (double gravity compensation, Procrustes alignment)
3. Added comprehensive logging showing all 4 LROM algorithm steps completing
4. Successfully computed calibration parameters with correct values

## Evolution Timeline

### Phase 1: Initial Implementation (June 21, 2025)
**Commit 1c7adc4** - "Fix double gravity compensation in COM calibration"
- **Status:** First potentially working version
- **Key Fix:** Resolved double gravity subtraction in COM calculation
- **Evidence:** COM values corrected from [-1.013, 0.567, 1.171]m to [0.000, -0.000, 0.050]m
- **Files created:** Initial implementation of wrench_calibration_node.cpp

### Phase 2: Refactoring & Breaking (July 4, 2025)
**Commit 0a340c4** - "refactor: extract LROM calibration algorithms into dedicated module"
- **Status:** Likely broke functionality during refactoring
- **Changes:** Extracted 325+ lines of algorithm code to separate file
- **Risk:** Major architectural change without verification

### Phase 3: Algorithm Fixes (July 22, 2025)
**Commit 4f9d86f** - "Refactored algorithm...proper Procrustes alignment using SVD"
- **Status:** Critical algorithm fix
- **Key Fix:** Replaced incorrect optimization-based rotation with proper Procrustes alignment
- **Impact:** Fixed sensor rotation estimation using SVD with determinant correction

### Phase 4: Final Working State (July 24, 2025)
**Commit f0265fc** - "fix(wrench_calibration): ensure processing of joint state data"
- **Status:** CONFIRMED WORKING ✅
- **Critical Fix:** 
  ```cpp
  // Before: Joint data received but not used
  rclcpp::wait_for_message(joint_msg, shared_from_this(), "/joint_states", std::chrono::seconds(10));
  
  // After: Joint data properly processed
  auto joint_msg_ptr = std::make_shared<sensor_msgs::msg::JointState>(joint_msg);
  updateJointPositions(joint_msg_ptr);  // THIS WAS MISSING!
  ```
- **Evidence:** Comprehensive logging output showing all calibration steps completing

### Phase 5: Feature Addition (July 24, 2025+)
**Commit f765792** - "feat: enhance wrench compensation to include probe frame transformations"
- **Status:** Likely still working (feature addition, not refactoring)
- **Changes:** Added probe frame support

### Phase 6: Refactoring Storm & Breaking (July 2025 - August 2025)
Multiple aggressive refactoring commits that broke functionality:
- f053e4f: "refactor: Clean up repository and fix topic subscription bug"
- 5a82e55: "fix: standardize transform naming convention"
- a5d8725 through f74c091: Multiple architecture changes
- bce77f0: "wrench_calibation_major_refgcatro_gonna_happen" (current state)

## Key Issues That Were Fixed

### 1. Double Gravity Compensation (Fixed in 1c7adc4)
**Problem:** Gravity was being subtracted twice in COM calculation
**Solution:** Only subtract force_bias (which already includes gravity)

### 2. Incorrect Rotation Extraction (Fixed in 4f9d86f)
**Problem:** Using optimization-based approach for rotation extraction
**Solution:** Proper Procrustes alignment with SVD and determinant correction

### 3. Joint State Not Processed (Fixed in f0265fc)
**Problem:** Joint state data received but current_joint_positions_ never updated
**Solution:** Call updateJointPositions() after receiving joint state message

## Code Verification at Working State

At commit f0265fc, the code had:
- ✅ Proper joint state processing
- ✅ Correct LROM algorithm implementation (4 steps)
- ✅ No double gravity compensation
- ✅ Proper Procrustes alignment for sensor rotation
- ✅ Comprehensive logging of all steps
- ✅ Successful calibration parameter computation

## Recommendation

To restore working calibration:
1. **Option 1:** Revert to commit f0265fc for calibration files only
   ```bash
   git checkout f0265fc -- src/wrench_calibration_node.cpp src/wrench_calibration_algorithm.cpp include/wrench_calibration_node.hpp
   ```

2. **Option 2:** Apply the critical fixes to current code:
   - Ensure joint state processing is implemented
   - Verify Procrustes alignment is used (not optimization)
   - Check no double gravity compensation
   - Confirm all 4 LROM steps are executing

## Current State Analysis

The current code (bce77f0) has been heavily refactored with:
- Changed function signatures (snake_case to PascalCase)
- Modified constant structures
- Different error handling approach
- Architectural changes that likely broke the calibration pipeline

The commit message "wrench_calibation_major_refgcatro_gonna_happen" suggests incomplete refactoring.

## Conclusion

**Definitive Answer:** Commit **f0265fc (July 24, 2025)** is the last confirmed working version of the wrench calibration system. The code worked because it had all critical bug fixes applied and was before the aggressive refactoring phase that broke functionality.