# Force Sensor and Control Issues Analysis Report

## Executive Summary
The UR Admittance Controller is now successfully launching and connecting to the robot system. However, two critical issues prevent proper operation:
1. **Force sensor bias** - Constant 9.8N force reading (gravity compensation issue)
2. **Position error safety stop** - 1.104m error exceeds 0.5m safety limit

## Progress Made ✅
- Controller successfully receives robot_description from topic
- Kinematics initialization works perfectly (6 joints, 9 segments)
- All ROS2 connections established properly
- Force sensor data is being received

## Issue 1: Force Sensor Bias (Critical)

### Current Sensor Reading
```yaml
force:
  x: 0.0156 N
  y: 9.8 N    # ⚠️ Constant gravity force!
  z: ~0 N
torque:
  x: -0.49 Nm
  y: 0.0008 Nm
  z: ~0 Nm
```

### Root Cause Analysis
The force sensor is measuring **internal forces** between joints, not external interaction forces:

1. **9.8N in Y-axis** = Exactly 1kg × 9.8m/s² (gravity)
   - This suggests the sensor is measuring the weight of the tool/gripper
   - The sensor frame orientation has Y pointing down

2. **Frame ID**: `ur/ft_sensor_joint/tcp_fts_sensor`
   - Sensor is between wrist and tool
   - Measures ALL forces, including tool weight

3. **Problem**: The controller interprets this constant 9.8N as an external push force!

### Why This Matters
- Controller thinks someone is constantly pushing with 9.8N force
- This creates unwanted motion commands
- The robot will try to "comply" with gravity forever

## Issue 2: Position Error on Startup

### Error Message
```
SAFETY: Position error 1.104 m > 0.500 m limit
```

### Root Cause
Looking at the initialization sequence:
```
1. Reference pose initialized at [0.000, 0.000, 0.000] m  # ❌ Wrong!
2. Transform lookup failed (base->tip)                     # TF not ready yet
3. InitializeDesiredPose() used identity transform         # Defaults to origin
```

The controller initialized the desired pose to the origin [0,0,0] because TF wasn't ready, but the actual robot TCP is likely at ~[1.1, 0, Z] meters from base.

## Technical Deep Dive

### Force Sensor Processing Flow
```
Physical Forces → F/T Sensor → ROS Topic → Our Callback → Control Algorithm
      ↓
Tool Weight + External Forces
      ↓
   9.8N + 0N = 9.8N (current reading)
```

### What Should Happen
```
1. Sensor Calibration/Taring
   - Measure baseline forces (gravity compensation)
   - Subtract baseline from all readings
   - Result: Only external forces remain

2. Proper Initialization
   - Wait for valid TF before setting desired pose
   - Initialize desired pose = current pose
   - Result: Zero initial error
```

## Proposed Solutions

### Solution 1: Force Sensor Bias Compensation
```cpp
// In WrenchCallback, after receiving data:
if (!force_bias_initialized_) {
  force_bias_ = raw_wrench;  // Store first reading as bias
  force_bias_initialized_ = true;
  RCLCPP_INFO("Force sensor bias recorded: [%.2f, %.2f, %.2f] N", 
              force_bias_(0), force_bias_(1), force_bias_(2));
}

// Subtract bias from all readings
Vector6d compensated_wrench = raw_wrench - force_bias_;
```

### Solution 2: Robust Pose Initialization
```cpp
bool AdmittanceNode::InitializeDesiredPose() {
  // Keep trying until we get a valid transform
  Eigen::Isometry3d current_pose = Eigen::Isometry3d::Identity();
  
  try {
    auto transform = tf_buffer_->lookupTransform(
      params_.base_link, params_.tip_link, 
      tf2::TimePointZero, std::chrono::seconds(1));  // Wait up to 1 second
    current_pose = tf2::transformToEigen(transform);
  } catch (...) {
    return false;  // Try again later
  }
  
  X_tcp_base_desired_ = current_pose;
  desired_pose_initialized_ = true;
  return true;
}
```

### Solution 3: Add Gravity Compensation Parameter
```yaml
# In config file
admittance:
  gravity_compensation:
    enabled: true
    force_bias: [0.0, 9.8, 0.0]    # Manual bias
    torque_bias: [0.0, 0.0, 0.0]
    auto_calibrate: true            # Auto-detect on startup
```

## Current Status Assessment

### What's Working ✅
- ROS2 infrastructure (topics, parameters, timers)
- Kinematics initialization from robot_description
- Force sensor data reception
- Safety checks (preventing dangerous motion)

### What's Not Working ❌
- Force sensor shows constant gravity bias
- Desired pose initializes to wrong position
- Control loop fails safety checks

### Are We Making Progress?
**YES!** Significant progress:
1. ✅ Solved robot_description access issue
2. ✅ Controller launches and connects properly
3. ✅ Identified root causes of remaining issues
4. ✅ Have clear path to solutions

## Next Steps (Priority Order)

1. **Implement force bias compensation** (Quick fix)
   - Add bias storage variables
   - Record baseline on startup
   - Subtract from all readings

2. **Fix pose initialization timing** (Quick fix)
   - Wait for valid TF before initializing
   - Retry if TF not ready

3. **Add configuration options** (Better solution)
   - Gravity compensation parameters
   - Calibration time parameter
   - Manual bias override option

4. **Consider advanced features** (Future)
   - Dynamic gravity compensation (orientation-aware)
   - Tool mass parameter
   - Automatic re-calibration triggers

## Conclusion

The admittance controller is very close to working. The issues are well-understood:
- **Force sensor bias**: Common issue with F/T sensors, easily fixed with calibration
- **Position initialization**: Timing issue with TF, needs proper synchronization

Both issues have straightforward solutions that are standard practice in force control applications.