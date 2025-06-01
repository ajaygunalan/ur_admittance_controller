# UR Admittance Controller - Complete Algorithm Pseudo Code

## **System Overview**
This document provides a complete step-by-step pseudo code analysis of the UR admittance controller, including all mathematical operations, inputs/outputs, and timing control cycles.

---

## **1. Initialization Phase**

### **1.1 Constructor Setup**
```
FUNCTION AdmittanceNode::AdmittanceNode()
  INPUT: ROS2 NodeOptions
  
  // Parameter initialization
  param_listener_ ← generate_parameter_library::ParamListener()
  params_ ← param_listener_.get_params()
  
  // State vector initialization
  joint_positions_[6] ← [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  joint_velocities_[6] ← [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  current_pos_[6] ← [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  
  // Control matrices initialization (6x6)
  mass_ ← diag([8.0, 8.0, 8.0, 0.8, 0.8, 0.8])
  damping_ ← Identity(6x6)
  stiffness_ ← Zero(6x6)
  
  // Control variables initialization
  F_sensor_base_[6] ← [0, 0, 0, 0, 0, 0]
  V_base_tip_base_[6] ← [0, 0, 0, 0, 0, 0]
  desired_vel_[6] ← [0, 0, 0, 0, 0, 0]
  
  // Transform infrastructure
  tf_buffer_ ← tf2_ros::Buffer()
  tf_listener_ ← tf2_ros::TransformListener()
  
  // ROS2 subscriptions
  CREATE_SUBSCRIPTION("/wrist_ft_sensor", wrenchCallback)
  CREATE_SUBSCRIPTION("/joint_states", jointStateCallback)
  CREATE_SUBSCRIPTION("/robot_description", robotDescriptionCallback)
  
  // ROS2 publishers
  CREATE_PUBLISHER("/scaled_joint_trajectory_controller/joint_trajectory")
  CREATE_PUBLISHER("/admittance_cartesian_velocity")
  CREATE_PUBLISHER("/admittance_pose_error")
  
  // Pre-allocate trajectory message
  trajectory_msg_.joint_names ← ["shoulder_pan_joint", "shoulder_lift_joint", 
                                 "elbow_joint", "wrist_1_joint", 
                                 "wrist_2_joint", "wrist_3_joint"]
  trajectory_msg_.points[0].positions.resize(6)
  trajectory_msg_.points[0].velocities.resize(6)
  trajectory_msg_.points[0].time_from_start ← 0.002 seconds  // 500Hz timing
  
  // Start control thread
  CONTROL_RATE_HZ ← 500.0
  running_ ← true
  control_thread_ ← THREAD(controlThreadFunction)
  
  OUTPUT: Initialized admittance node
END FUNCTION
```

---

## **2. Real-Time Control Thread**

### **2.1 Control Thread Function**
```
FUNCTION controlThreadFunction()
  CONTROL_RATE_HZ ← 500.0
  period_ns ← 1e9 / CONTROL_RATE_HZ  // 2,000,000 nanoseconds = 2ms
  
  next_time ← current_steady_clock_time()
  
  WHILE (ros_ok AND running_)
    CALL controlLoop()
    
    // Maintain precise 500Hz timing
    next_time ← next_time + period_ns
    SLEEP_UNTIL(next_time)
  END WHILE
END FUNCTION
```

### **2.2 Main Control Loop**
```
FUNCTION controlLoop()
  // Timing management
  current_time ← now()
  period ← current_time - last_time
  
  IF (period ≤ 0.0 OR period > 0.1)
    RETURN  // Skip invalid periods
  END IF
  
  last_time ← current_time
  
  // Core admittance computation
  success ← computeAdmittanceStep(period)
  
  IF (NOT success)
    CALL safeStop()
    RETURN
  END IF
  
  // Prepare trajectory message
  trajectory_msg_.header.stamp ← current_time
  trajectory_msg_.points[0].positions ← joint_positions_[6]
  trajectory_msg_.points[0].velocities ← joint_velocities_[6]
  
  // Publish to scaled_joint_trajectory_controller
  PUBLISH(trajectory_msg_)
  
  // Publish monitoring data
  CALL publishMonitoringData()
END FUNCTION
```

---

## **3. Sensor Data Processing**

### **3.1 Force/Torque Sensor Callback**
```
FUNCTION wrenchCallback(msg)
  INPUT: geometry_msgs::WrenchStamped
  
  LOCK(wrench_mutex_)
  
  // Extract raw wrench data
  raw_wrench[6] ← [msg.wrench.force.x,    msg.wrench.force.y,    msg.wrench.force.z,
                   msg.wrench.torque.x,   msg.wrench.torque.y,   msg.wrench.torque.z]
  
  // Transform to base frame
  F_sensor_base_ ← transformWrench(raw_wrench)
  
  // Apply exponential moving average filter
  α ← params_.admittance.filter_coefficient  // 0.8 (default)
  wrench_filtered_ ← α * F_sensor_base_ + (1-α) * wrench_filtered_
  
  UNLOCK(wrench_mutex_)
  
  OUTPUT: F_sensor_base_[6] [N, N, N, Nm, Nm, Nm]
END FUNCTION
```

### **3.2 Wrench Transformation**
```
FUNCTION transformWrench(wrench_sensor_frame)
  INPUT: wrench_sensor_frame[6] [N, N, N, Nm, Nm, Nm]
  
  IF (ft_frame == base_link)
    RETURN wrench_sensor_frame
  END IF
  
  TRY
    // TF2 transform lookup
    T ← tf_buffer_.lookupTransform(base_link, ft_frame, latest_time)
    
    // Extract rotation and translation
    R ← T.rotation()     // 3x3 rotation matrix
    t ← T.translation()  // 3x1 translation vector
    
    // Compute 6x6 adjoint matrix for wrench transformation
    adjoint ← [[R,      0₃ₓ₃],
               [t×R,    R   ]]
    
    RETURN adjoint * wrench_sensor_frame
    
  CATCH (tf2::TransformException)
    RETURN wrench_sensor_frame  // Fallback to sensor frame
  END TRY
  
  OUTPUT: wrench_base_frame[6] [N, N, N, Nm, Nm, Nm]
END FUNCTION
```

### **3.3 Joint State Callback**
```
FUNCTION jointStateCallback(msg)
  INPUT: sensor_msgs::JointState
  
  LOCK(joint_state_mutex_)
  
  FOR i = 0 TO 5
    joint_name ← params_.joints[i]
    index ← FIND_INDEX(msg.name, joint_name)
    IF (index exists)
      joint_positions_[i] ← msg.position[index]  // [rad]
    END IF
  END FOR
  
  UNLOCK(joint_state_mutex_)
  
  OUTPUT: joint_positions_[6] [rad, rad, rad, rad, rad, rad]
END FUNCTION
```

---

## **4. Core Admittance Algorithm**

### **4.1 Main Admittance Step**
```
FUNCTION computeAdmittanceStep(period)
  INPUT: period [seconds] (≈0.002s for 500Hz)
  
  // Lazy initialization check
  IF (NOT kinematics_ready_)
    IF (robot_description_received_)
      CALL loadKinematics()
    END IF
    IF (NOT kinematics_ready_)
      RETURN false
    END IF
  END IF
  
  // Parameter updates (every 100ms for performance)
  CALL checkParameterUpdates()
  
  // Transform updates
  success ← updateTransforms()
  IF (NOT success)
    RETURN false
  END IF
  
  // Deadband check
  IF (NOT checkDeadband())
    V_base_tip_base_ ← [0, 0, 0, 0, 0, 0]
    desired_vel_ ← [0, 0, 0, 0, 0, 0]
    RETURN true
  END IF
  
  // Core admittance control
  cmd_vel ← computeAdmittanceControl(period)
  IF (cmd_vel is invalid)
    RETURN false
  END IF
  
  V_base_tip_base_ ← cmd_vel
  
  // Convert to joint space
  success ← convertToJointSpace(cmd_vel, period)
  IF (NOT success)
    RETURN false
  END IF
  
  // Apply joint limits
  success ← applyJointLimits(period)
  IF (NOT success)
    RETURN false
  END IF
  
  RETURN true
  
  OUTPUT: joint_positions_[6], joint_velocities_[6]
END FUNCTION
```

### **4.2 Admittance Control Computation**
```
FUNCTION computeAdmittanceControl(period)
  INPUT: period [seconds]
  
  // Compute pose error
  error_tip_base_ ← computePoseError_tip_base()
  
  // Update stiffness engagement factor (for safe startup)
  CALL updateStiffnessEngagement(period)
  
  dt ← period.seconds()
  IF (dt ≤ 0.0 OR dt > 0.1)
    RETURN invalid
  END IF
  
  // Runge-Kutta 4th order integration for: M⁻¹(F_ext - D*v - K*x) = dv/dt
  v₀ ← desired_vel_
  
  // k₁ = f(tₙ, vₙ)
  stiffness_force ← stiffness_engagement_factor_ * (stiffness_ * error_tip_base_)
  k₁ ← mass_inverse_ * (wrench_filtered_ - damping_ * v₀ - stiffness_force)
  
  // k₂ = f(tₙ + dt/2, vₙ + k₁*dt/2)
  v₁ ← v₀ + k₁ * (dt/2)
  k₂ ← mass_inverse_ * (wrench_filtered_ - damping_ * v₁ - stiffness_force)
  
  // k₃ = f(tₙ + dt/2, vₙ + k₂*dt/2)
  v₂ ← v₀ + k₂ * (dt/2)
  k₃ ← mass_inverse_ * (wrench_filtered_ - damping_ * v₂ - stiffness_force)
  
  // k₄ = f(tₙ + dt, vₙ + k₃*dt)
  v₃ ← v₀ + k₃ * dt
  k₄ ← mass_inverse_ * (wrench_filtered_ - damping_ * v₃ - stiffness_force)
  
  // RK4 integration step
  desired_vel_ ← v₀ + (dt/6) * (k₁ + 2*k₂ + 2*k₃ + k₄)
  
  // Apply axis enable/disable mask
  FOR i = 0 TO 5
    IF (NOT params_.admittance.enabled_axes[i])
      desired_vel_[i] ← 0.0
    END IF
  END FOR
  
  // Apply Cartesian velocity limits
  CALL applyCartesianVelocityLimits()
  
  // Drift reset check
  IF (||desired_vel_|| < params_.admittance.drift_reset_threshold)
    CALL handleDriftReset()
    RETURN [0, 0, 0, 0, 0, 0]
  END IF
  
  RETURN desired_vel_
  
  OUTPUT: cmd_vel[6] [m/s, m/s, m/s, rad/s, rad/s, rad/s]
END FUNCTION
```

### **4.3 Mathematical Formulation**

The core admittance equation being solved:

$$M \ddot{x} + D \dot{x} + K x = F_{ext}$$

Where:
- $M \in \mathbb{R}^{6 \times 6}$: Virtual mass matrix (diagonal)
- $D \in \mathbb{R}^{6 \times 6}$: Damping matrix (computed from mass, stiffness, damping ratios)
- $K \in \mathbb{R}^{6 \times 6}$: Stiffness matrix (diagonal, typically zero for pure admittance)
- $x \in \mathbb{R}^6$: Pose error [position, orientation] 
- $F_{ext} \in \mathbb{R}^6$: External wrench [forces, torques]

Rearranged for velocity integration:
$$\dot{v} = M^{-1}(F_{ext} - D v - K x)$$

Damping computation:
$$D_{ii} = 2 \zeta_i \sqrt{M_{ii} K_{ii}}$$ for stiffness control
$$D_{ii} = 2 \zeta_i \sqrt{M_{ii} K_{virtual}}$$ for pure admittance

---

## **5. Cartesian to Joint Space Conversion**

### **5.1 KDL Inverse Kinematics**
```
FUNCTION convertToJointSpace(cart_vel, period)
  INPUT: cart_vel[6] [m/s, m/s, m/s, rad/s, rad/s, rad/s]
  INPUT: period [seconds]
  
  // Validation
  IF (period ≤ 0 OR cart_vel has NaN OR NOT kinematics_ready_)
    RETURN false
  END IF
  
  // Get current joint positions
  LOCK(joint_state_mutex_)
  current_pos_ ← joint_positions_
  UNLOCK(joint_state_mutex_)
  
  // Prepare KDL data structures
  q_current ← KDL::JntArray(6)
  FOR i = 0 TO 5
    q_current[i] ← current_pos_[i]  // [rad]
  END FOR
  
  // Create KDL Twist (CORRECT: velocity → velocity)
  cart_twist ← KDL::Twist()
  cart_twist.vel.x ← cart_vel[0]    // Linear velocity [m/s]
  cart_twist.vel.y ← cart_vel[1]
  cart_twist.vel.z ← cart_vel[2]
  cart_twist.rot.x ← cart_vel[3]    // Angular velocity [rad/s]
  cart_twist.rot.y ← cart_vel[4]
  cart_twist.rot.z ← cart_vel[5]
  
  // Solve inverse kinematics: J(q)·q̇ = v
  q_dot ← KDL::JntArray(6)
  result ← ik_vel_solver_.CartToJnt(q_current, cart_twist, q_dot)
  
  IF (result < 0)
    RETURN false  // KDL solver failed
  END IF
  
  // Store joint velocities directly (CORRECT: velocity → velocity)
  FOR i = 0 TO 5
    joint_velocities_[i] ← q_dot[i]  // [rad/s]
  END FOR
  
  // Integrate velocities to get positions (CORRECT: q = q₀ + v×Δt)
  FOR i = 0 TO 5
    joint_positions_[i] ← current_pos_[i] + joint_velocities_[i] * period
  END FOR
  
  RETURN true
  
  OUTPUT: joint_velocities_[6] [rad/s], joint_positions_[6] [rad]
END FUNCTION
```

### **5.2 Mathematical Relationship**

The KDL solver solves:
$$J(q) \dot{q} = v$$

Where:
- $J(q) \in \mathbb{R}^{6 \times 6}$: Robot Jacobian at configuration $q$
- $\dot{q} \in \mathbb{R}^6$: Joint velocities [rad/s]
- $v \in \mathbb{R}^6$: Cartesian velocity [m/s, rad/s]

Integration step:
$$q_{k+1} = q_k + \dot{q} \Delta t$$

---

## **6. Joint Limiting and Safety**

### **6.1 Joint Limits Application**
```
FUNCTION applyJointLimits(period)
  INPUT: period [seconds]
  
  FOR i = 0 TO 5
    // Velocity limiting
    v_max ← joint_limits_[i].max_velocity  // [rad/s]
    IF (|joint_velocities_[i]| > v_max)
      scale ← v_max / |joint_velocities_[i]|
      joint_velocities_[i] ← joint_velocities_[i] * scale
    END IF
    
    // Position limiting
    q_min ← joint_limits_[i].min_position  // [rad]
    q_max ← joint_limits_[i].max_position  // [rad]
    joint_positions_[i] ← CLAMP(joint_positions_[i], q_min, q_max)
    
    // NaN check
    IF (joint_positions_[i] is NaN OR joint_velocities_[i] is NaN)
      RETURN false
    END IF
  END FOR
  
  RETURN true
  
  OUTPUT: joint_velocities_[6] [rad/s], joint_positions_[6] [rad] (limited)
END FUNCTION
```

### **6.2 Emergency Safe Stop**
```
FUNCTION safeStop()
  // Zero all velocities
  V_base_tip_base_ ← [0, 0, 0, 0, 0, 0]
  desired_vel_ ← [0, 0, 0, 0, 0, 0]
  
  FOR i = 0 TO 5
    joint_velocities_[i] ← 0.0
  END FOR
  
  RETURN true
END FUNCTION
```

---

## **7. Trajectory Message Construction**

### **7.1 ROS2 Trajectory Publishing**
```
FUNCTION publishTrajectoryMessage()
  // Pre-allocated message (500Hz performance)
  trajectory_msg_.header.stamp ← current_time
  
  // Both position and velocity required by scaled_joint_trajectory_controller
  trajectory_msg_.points[0].positions ← joint_positions_[6]   // [rad]
  trajectory_msg_.points[0].velocities ← joint_velocities_[6] // [rad/s]
  trajectory_msg_.points[0].time_from_start ← 0.002 seconds   // 500Hz timing
  
  // Publish to UR controller
  PUBLISH("/scaled_joint_trajectory_controller/joint_trajectory", trajectory_msg_)
  
  OUTPUT: trajectory_msgs::JointTrajectory
END FUNCTION
```

---

## **8. Timing and Control Cycle Summary**

### **8.1 Complete Control Cycle Flow**
```
CONTROL_CYCLE (every 2ms @ 500Hz):
  
  1. TIMING_CHECK (0.001ms)
     └── Validate period, skip if invalid
  
  2. LAZY_INITIALIZATION (0.01ms, first cycle only)
     └── Load kinematics when robot_description available
  
  3. PARAMETER_UPDATES (0.02ms, every 100ms)
     └── Check for dynamic parameter changes
  
  4. TRANSFORM_UPDATE (0.05ms)
     └── tf2 lookup: base_link ↔ tool0
  
  5. DEADBAND_CHECK (0.01ms)
     └── Check if ||F_ext|| > threshold
  
  6. ADMITTANCE_COMPUTATION (0.3ms)
     ├── Pose error computation
     ├── RK4 integration (4 steps)
     ├── Axis masking
     └── Cartesian velocity limiting
  
  7. INVERSE_KINEMATICS (0.1ms)
     ├── KDL velocity solver: J(q)·q̇ = v
     └── Position integration: q = q₀ + v×Δt
  
  8. JOINT_LIMITING (0.02ms)
     ├── Velocity limits: |q̇| ≤ q̇_max
     └── Position limits: q_min ≤ q ≤ q_max
  
  9. TRAJECTORY_PUBLISHING (0.05ms)
     └── ROS2 message to scaled_joint_trajectory_controller
  
  10. MONITORING_DATA (0.01ms)
      └── Publish Cartesian velocity for debugging
  
  TOTAL: ~0.6ms per cycle (leaving 1.4ms margin for 500Hz)
```

### **8.2 Data Flow Summary**
```
EXTERNAL_INPUTS:
  /wrist_ft_sensor     → F_ext[6] [N, Nm]        @ variable rate
  /joint_states        → q_current[6] [rad]      @ 1000Hz
  /robot_description   → URDF string             @ once

INTERNAL_PROCESSING:
  F_ext → wrench_filtered_ → V_cartesian → q̇_joint → q_joint
                            [m/s,rad/s]   [rad/s]   [rad]

OUTPUTS:
  /scaled_joint_trajectory_controller/joint_trajectory → q[6], q̇[6] @ 500Hz
  /admittance_cartesian_velocity                       → v[6]       @ 500Hz  
  /admittance_pose_error                              → error[6]   @ 500Hz
```

---

## **9. Mathematical Constants and Parameters**

### **9.1 Default Parameter Values**
```
ADMITTANCE_PARAMETERS:
  mass: [8.0, 8.0, 8.0, 0.8, 0.8, 0.8] [kg, kg, kg, kg·m², kg·m², kg·m²]
  stiffness: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] [N/m, N/m, N/m, Nm/rad, Nm/rad, Nm/rad]
  damping_ratio: [0.8, 0.8, 0.8, 0.8, 0.8, 0.8] [dimensionless]
  filter_coefficient: 0.8 [dimensionless]
  min_motion_threshold: 1.5 [N, Nm]
  drift_reset_threshold: 0.001 [m/s]

VELOCITY_LIMITS:
  max_linear_velocity: 0.5 [m/s]
  max_angular_velocity: 1.0 [rad/s]
  
JOINT_LIMITS (UR5e):
  position: [-6.283, 6.283] [rad] (±360°)
  velocity: [3.142] [rad/s] (180°/s)
  
CONTROL_TIMING:
  control_rate: 500 [Hz]
  period: 0.002 [seconds]
  trajectory_time_from_start: 0.002 [seconds]
```

This pseudo code represents the complete mathematical and algorithmic implementation of the UR admittance controller, with proper unit consistency and timing control throughout the entire system.