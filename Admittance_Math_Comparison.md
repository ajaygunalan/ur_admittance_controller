# Detailed Mathematical Comparison: ROS1 vs ROS2 Admittance Control

## Important Correction

After analyzing the ROS1 code more carefully, here's what the matrices actually represent:

**ROS1 Matrices:**
- **M_a**: Arm mass matrix (how much inertia the arm appears to have)
- **K**: Stiffness matrix (spring constant for position restoring)
- **D**: Coupling damping (usually zero - damping in the spring-damper coupling)
- **D_a**: Arm damping (how much the arm resists velocity)

**There is NO separate "M" matrix in ROS1 - only M_a!**

## 1. Classical Admittance Control (Our ROS2 Implementation)

### Mathematical Formulation

Our ROS2 implementation uses the standard admittance equation:

$$M\ddot{x} + D\dot{x} + Kx = F_{external}$$

Rearranging to solve for acceleration:

$$\ddot{x} = M^{-1}(F_{external} - D\dot{x} - Kx)$$

### Our ROS2 Code Implementation

```cpp
// From admittance_computations.cpp
bool AdmittanceNode::compute_admittance() {
  // Calculate error (x)
  error_tcp_base_ = compute_pose_error();
  
  // Standard admittance equation: a = M^(-1) * (F - Dv - Kx)
  const Vector6d acceleration = M_inverse_diag_.array() *
      (Wrench_tcp_base_.array() - D_diag_.array() * V_tcp_base_desired_.array() -
       K_diag_.array() * error_tcp_base_.array());
  
  // Integration: v_new = v_old + a * dt
  V_tcp_base_desired_ = V_tcp_base_desired_ + acceleration * dt;
}
```

### What This Means

- **Single dynamics equation**: All forces (external, damping, spring) are lumped together
- **Direct coupling**: Position error directly affects acceleration through K
- **Fixed compliance**: The response to external forces is always the same

## 2. Advanced Admittance Control (ROS1 Implementation)

### Mathematical Formulation

The ROS1 implementation uses a different formulation with coupling dynamics:

#### Step 1: Coupling Wrench (Spring-Damper Forces)
$$F_{coupling} = D\dot{x}_{desired} + Kx_{error}$$

Where:
- **K**: Spring stiffness (pulls back to desired position)
- **D**: Coupling damping (usually zero in practice)

#### Step 2: Modified Admittance Equation
$$M_a\ddot{x} = -F_{coupling} - D_a\dot{x}_{desired} + \alpha F_{external} + F_{control}$$

Where:
- **M_a**: Arm mass matrix (virtual inertia)
- **D_a**: Arm damping matrix (velocity resistance)
- **α**: Admittance ratio (0 to 1)
- **F_control**: Additional control wrench input

Expanding the coupling wrench:
$$M_a\ddot{x} = -(D\dot{x} + Kx) - D_a\dot{x} + \alpha F_{external} + F_{control}$$

Combining damping terms:
$$M_a\ddot{x} = -(D + D_a)\dot{x} - Kx + \alpha F_{external} + F_{control}$$

Rearranging:
$$\ddot{x} = M_a^{-1}(-(D + D_a)\dot{x} - Kx + \alpha F_{external} + F_{control})$$

### ROS1 Code Implementation

```cpp
// From ROS1 AdmittanceController.cpp
// Matrix declarations:
Matrix6d M_a_, D_, D_a_, K_;  // Note: NO separate M matrix!

void AdmittanceController::compute_admittance() {
  // Step 1: Compute coupling wrench (spring + damper forces)
  // D_ is usually zero, K_ provides spring restoring force
  Vector6d coupling_wrench_arm = D_ * arm_desired_twist_adm_ + K_ * error;
  
  // Step 2: Compute acceleration with multiple terms
  arm_desired_accelaration = M_a_.inverse() * 
      (-coupling_wrench_arm           // Spring-damper coupling
       - D_a_ * arm_desired_twist_adm_ // Arm velocity damping
       + admittance_ratio_ * wrench_external_  // Scaled external force
       + wrench_control_);             // Control overlay
  
  // Integration
  arm_desired_twist_adm_ += arm_desired_accelaration * dt;
}

// Typical parameter values from ROS1:
// M_a = diag([2, 2, 2, 2, 2, 2])      // 2kg virtual mass
// K = diag([10, 20, 10, 10, 10, 10])  // Spring stiffness
// D = diag([0, 0, 0, 0, 0, 0])        // No coupling damping
// D_a = diag([12, 12, 12, 10, 10, 10]) // Arm damping
```

## 3. Key Differences Explained

### 3.1 Matrix Philosophy

**ROS1 approach:**
- **M_a**: Virtual mass (inertia) of the arm
- **K**: Spring stiffness to pull back to desired position
- **D**: Coupling damping (usually zero)
- **D_a**: Arm damping (resists velocity)
- Total damping = D + D_a (but D is usually 0, so just D_a)

**Our ROS2 approach:**
- **M**: Virtual mass (same role as M_a)
- **D**: Total damping (equivalent to ROS1's D + D_a)
- **K**: Spring stiffness (same role)

**Key insight**: ROS1 separates damping into two parts, but in practice D=0, so it's really just using D_a!

### 3.2 The Coupling Wrench Term

The coupling wrench represents the "internal forces" trying to restore equilibrium:

$$F_{coupling} = D\dot{x} + Kx$$

This is computed separately and then subtracted in the main equation, which means:
- The coupling dynamics can be tuned independently
- The arm can have different damping (D_a) than the coupling (D)

### 3.3 Variable Admittance Ratio (α)

**ROS1**: 
$$\alpha F_{external}$$

- When α = 0: No response to external forces (position control)
- When α = 1: Full admittance response
- When 0 < α < 1: Partial compliance

**Our ROS2**: Always uses the full external force (effectively α = 1)

### 3.4 Control Wrench Overlay

**ROS1**: Adds $F_{control}$ term
- Allows injecting additional forces
- Enables hybrid force/position control
- Useful for trajectory tracking with compliance

**Our ROS2**: No control overlay capability

## 4. Physical Interpretation

### Our ROS2 System (Simple Spring-Mass-Damper)

```
External Force → [M, D, K System] → Motion
```

- Acts like a single spring-mass-damper system
- All parameters affect the response equally

### ROS1 System (Dual-Layer Control)

```
External Force → [Scaling α] → [Arm Dynamics M_a, D_a] → Motion
                                          ↑
                                    [Coupling D, K]
                                          ↑
                                    Position Error
```

- **Layer 1**: Coupling dynamics (D, K) create restoring forces
- **Layer 2**: Arm dynamics (M_a, D_a) determine how the arm responds to ALL forces

## 5. Practical Example - Corrected

Let's see how both systems respond to a 10N push in the X direction:

### Our ROS2 Response

```cpp
// Given: F_external = [10, 0, 0, 0, 0, 0]
// With M=2kg, D=12Ns/m, K=10N/m, x_error=0.1m, v=0 (starting from rest)

acceleration = M^(-1) * (F_external - D*v - K*x)
acceleration = (1/2) * (10 - 12*0 - 10*0.1)
acceleration = 0.5 * (10 - 0 - 1)
acceleration = 0.5 * 9 = 4.5 m/s²
```

### ROS1 Response

```cpp
// Given: F_external = [10, 0, 0, 0, 0, 0]
// With M_a=2kg, D=0 (coupling damping), D_a=12Ns/m, K=10N/m, x_error=0.1m, v=0
// α=1.0 (full admittance)

coupling_wrench = D*v + K*x = 0*0 + 10*0.1 = 1
acceleration = M_a^(-1) * (-coupling_wrench - D_a*v + α*F_external + F_control)
acceleration = (1/2) * (-1 - 12*0 + 1.0*10 + 0)
acceleration = 0.5 * (-1 + 10)
acceleration = 0.5 * 9 = 4.5 m/s²
```

**Result**: Both systems produce the same acceleration! This is because:
- ROS2: Uses D=12 for total damping
- ROS1: Uses D=0, D_a=12, so total damping = 0+12 = 12

The main difference is ROS1 adds the admittance ratio α for scaling compliance.

## 6. Real Advantages of ROS1's Approach

After careful analysis, the main advantages are:

### 1. **Variable Admittance Ratio (α)**
This is the KEY difference! ROS1 can scale the external force response:
- α = 0: No compliance (pure position control)
- α = 0.5: Half compliance (partially stiff)
- α = 1.0: Full compliance (like our ROS2)

This allows dynamic compliance adjustment during operation.

### 2. **Control Wrench Overlay (F_control)**
- Inject additional forces for trajectory tracking
- Implement virtual fixtures or constraints
- Blend admittance with other control strategies

### 3. **Conceptual Clarity (Minor)**
- Separating D and D_a clarifies intent (even if D=0)
- Makes it clear what's "coupling" vs "arm" damping

### 4. **Not Really Different Math**
When D=0 (as typical), the equations become nearly identical:
- ROS2: `M*a = F - D*v - K*x`
- ROS1: `M_a*a = α*F - D_a*v - K*x` (when D=0)

The only real differences are α and F_control!

## 7. Simplified Implementation Recommendation for ROS2

After understanding the ROS1 code better, here's a minimal upgrade to get the key benefits:

```cpp
// Enhanced ROS2 admittance computation (minimal changes)
bool AdmittanceNode::compute_admittance() {
  // Compute error
  error_tcp_base_ = compute_pose_error();
  
  // Standard admittance with two new features:
  // 1. Admittance ratio for variable compliance
  // 2. Control wrench for trajectory overlay
  const Vector6d acceleration = M_inverse_diag_.array() *
      (admittance_ratio_ * Wrench_tcp_base_.array()  // Scaled external force
       + Wrench_control_.array()                      // Control overlay
       - D_diag_.array() * V_tcp_base_desired_.array()
       - K_diag_.array() * error_tcp_base_.array());
  
  // Integration
  V_tcp_base_desired_ = V_tcp_base_desired_ + acceleration * dt;
}
```

This minimal change adds:
1. **admittance_ratio_** parameter (0-1) for variable compliance
2. **Wrench_control_** input topic for force overlay

No need to split D into D and D_a since D=0 in practice!

## Summary: The Truth About ROS1 vs ROS2 Formulations

**The mathematical difference is much smaller than it first appears!**

1. **ROS1 uses 4 matrices**: M_a, K, D (=0), D_a
2. **ROS2 uses 3 matrices**: M, K, D
3. **When D=0 in ROS1** (typical case), the equations are nearly identical

**The real innovations in ROS1 are:**
- **Admittance ratio α**: Scale compliance dynamically
- **Control wrench overlay**: Add trajectory forces
- **Conceptual separation**: Clearer parameter naming

The "coupling dynamics" terminology is somewhat misleading since D=0 in practice. The system is really just using K for spring forces and D_a for damping, which is exactly what our ROS2 does with K and D!