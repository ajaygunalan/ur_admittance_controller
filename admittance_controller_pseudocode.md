## Admittance Control for Robot Manipulators

### Problem Statement

A robot manipulator needs to interact compliantly with its environment by modulating its motion in response to external forces. The admittance controller treats the robot as a virtual mass-spring-damper system that responds to external wrenches (forces and torques) by generating appropriate velocities.

### Model

The admittance control law models the robot's end-effector as a second-order dynamic system:

$$\mathbf{M} \cdot \ddot{\mathbf{x}} + \mathbf{D} \cdot \dot{\mathbf{x}} + \mathbf{K} \cdot \mathbf{x} = \mathbf{F}_{\text{ext}}$$

Where:
- $\mathbf{M} \in \mathbb{R}^{6 \times 6}$: Virtual inertia matrix (diagonal)
- $\mathbf{D} \in \mathbb{R}^{6 \times 6}$: Virtual damping matrix (diagonal)
- $\mathbf{K} \in \mathbb{R}^{6 \times 6}$: Virtual stiffness matrix (diagonal)
- $\mathbf{x} \in \mathbb{R}^6$: Pose error from equilibrium $[\Delta\mathbf{p}^T, \Delta\boldsymbol{\theta}^T]^T$
- $\dot{\mathbf{x}} \in \mathbb{R}^6$: Cartesian velocity $[\mathbf{v}^T, \boldsymbol{\omega}^T]^T$
- $\ddot{\mathbf{x}} \in \mathbb{R}^6$: Cartesian acceleration
- $\mathbf{F}_{\text{ext}} \in \mathbb{R}^6$: External wrench $[\mathbf{f}^T, \boldsymbol{\tau}^T]^T$

### Given

- Joint positions: $\mathbf{q} \in \mathbb{R}^n$ from joint state feedback
- External wrench: $^P\mathbf{F}_B \in \mathbb{R}^6$ measured at probe point P in base frame B
- Desired equilibrium pose: $^B\mathbf{X}_{P,\text{des}} \in SE(3)$
- Admittance parameters: $\mathbf{M}$, $\mathbf{D}$, $\mathbf{K}$ (diagonal matrices)
- Robot kinematics: Forward kinematics $f_{FK}(\mathbf{q})$ and Jacobian $\mathbf{J}(\mathbf{q})$
- Control frequency: $f_c = 100$ Hz (period $\Delta t = 0.01$ s)

### To Find

- Joint velocities: $\dot{\mathbf{q}} \in \mathbb{R}^n$ that realize the compliant behavior

---

### 1. Control Loop Architecture

The admittance controller executes a control cycle at 100 Hz:

```
for each control cycle at time t:
    1. GetCurrentPose()        // Forward kinematics
    2. ComputePoseError()       // Error from equilibrium  
    3. ComputeAdmittance()      // Apply control law
    4. LimitToWorkspace()       // Safety constraints
    5. ComputeJointVelocities() // Inverse kinematics
    6. PublishCommands()        // Send to robot
```

---

### 2. Forward Kinematics - Get Current Pose

#### 2.1 Compute End-Effector Pose

Given joint positions $\mathbf{q}$, compute the current pose of the probe point P in base frame B:

$$^B\mathbf{X}_{P,\text{curr}} = f_{FK}(\mathbf{q}) = ^B\mathbf{X}_{W3}(\mathbf{q}) \cdot ^{W3}\mathbf{X}_P$$

Where:
- $^B\mathbf{X}_{W3}(\mathbf{q})$: Wrist pose from forward kinematics
- $^{W3}\mathbf{X}_P$: Fixed transform from wrist to probe point

The pose is represented as an isometry:
$$^B\mathbf{X}_P = \begin{bmatrix} ^B\mathbf{R}_P & ^B\mathbf{p}_P \\ \mathbf{0}^T & 1 \end{bmatrix} \in SE(3)$$

---

### 3. Compute Pose Error

#### 3.1 Position Error

The position error is simply the difference:
$$\Delta\mathbf{p} = ^B\mathbf{p}_{P,\text{curr}} - ^B\mathbf{p}_{P,\text{des}} \in \mathbb{R}^3$$

#### 3.2 Orientation Error

The orientation error requires special handling on SO(3):

1. Extract quaternions from rotation matrices:
   $$\mathbf{q}_{\text{curr}} = \text{quat}(^B\mathbf{R}_{P,\text{curr}})$$
   $$\mathbf{q}_{\text{des}} = \text{quat}(^B\mathbf{R}_{P,\text{des}})$$

2. Ensure shortest rotation path:
   $$\text{if } \mathbf{q}_{\text{curr}} \cdot \mathbf{q}_{\text{des}} < 0 \text{ then } \mathbf{q}_{\text{des}} \leftarrow -\mathbf{q}_{\text{des}}$$

3. Compute error quaternion:
   $$\mathbf{q}_{\text{err}} = \mathbf{q}_{\text{curr}} \otimes \mathbf{q}_{\text{des}}^{-1}$$

4. Convert to axis-angle:
   $$\Delta\boldsymbol{\theta} = \text{angle}(\mathbf{q}_{\text{err}}) \cdot \text{axis}(\mathbf{q}_{\text{err}}) \in \mathbb{R}^3$$

#### 3.3 Combined Pose Error

$$\mathbf{x}_{\text{err}} = \begin{bmatrix} \Delta\mathbf{p} \\ \Delta\boldsymbol{\theta} \end{bmatrix} \in \mathbb{R}^6$$

---

### 4. Compute Admittance

#### 4.1 Solve for Acceleration

Rearranging the admittance equation for acceleration:

$$\ddot{\mathbf{x}} = \mathbf{M}^{-1} \cdot (\mathbf{F}_{\text{ext}} - \mathbf{D} \cdot \dot{\mathbf{x}} - \mathbf{K} \cdot \mathbf{x}_{\text{err}})$$

For diagonal matrices, this simplifies element-wise:
$$\ddot{x}_i = \frac{1}{m_i} \cdot (f_{ext,i} - d_i \cdot \dot{x}_i - k_i \cdot x_{err,i})$$

#### 4.2 Apply Acceleration Limits

To ensure safety, limit linear and angular accelerations:

$$\ddot{\mathbf{x}}_{\text{limited}} = \begin{cases}
\ddot{\mathbf{x}}_{\text{lin}} \cdot \frac{a_{\text{max,lin}}}{||\ddot{\mathbf{x}}_{\text{lin}}||} & \text{if } ||\ddot{\mathbf{x}}_{\text{lin}}|| > a_{\text{max,lin}} \\
\ddot{\mathbf{x}}_{\text{ang}} \cdot \frac{a_{\text{max,ang}}}{||\ddot{\mathbf{x}}_{\text{ang}}||} & \text{if } ||\ddot{\mathbf{x}}_{\text{ang}}|| > a_{\text{max,ang}}
\end{cases}$$

#### 4.3 Integrate Velocity

Using Euler integration with time step $\Delta t$:

$$\dot{\mathbf{x}}_{t+1} = \dot{\mathbf{x}}_t + \ddot{\mathbf{x}}_{\text{limited}} \cdot \Delta t$$

This gives the commanded Cartesian velocity:
$$^P\mathbf{V}_{B,\text{cmd}} = \begin{bmatrix} \mathbf{v}_{\text{cmd}} \\ \boldsymbol{\omega}_{\text{cmd}} \end{bmatrix}$$

---

### 5. Apply Safety Constraints

#### 5.1 Workspace Limits

Prevent the end-effector from leaving the safe workspace:

For each axis $i \in \{x, y, z\}$:
$$v_{cmd,i} = \begin{cases}
\max(0, v_{cmd,i}) & \text{if } p_i \leq p_{i,\text{min}} \\
\min(0, v_{cmd,i}) & \text{if } p_i \geq p_{i,\text{max}} \\
v_{cmd,i} & \text{otherwise}
\end{cases}$$

#### 5.2 Velocity Magnitude Limits

Limit maximum linear and angular velocities:

$$\mathbf{v}_{\text{limited}} = \begin{cases}
\mathbf{v}_{\text{cmd}} \cdot \frac{v_{\text{max,lin}}}{||\mathbf{v}_{\text{cmd}}||} & \text{if } ||\mathbf{v}_{\text{cmd}}|| > v_{\text{max,lin}} \\
\boldsymbol{\omega}_{\text{cmd}} \cdot \frac{v_{\text{max,ang}}}{||\boldsymbol{\omega}_{\text{cmd}}||} & \text{if } ||\boldsymbol{\omega}_{\text{cmd}}|| > v_{\text{max,ang}}
\end{cases}$$

---

### 6. Inverse Kinematics - Compute Joint Velocities

#### 6.1 Transform Velocity to Wrist Frame

Since the Jacobian is defined for the wrist (W3), transform the probe velocity:

$$^{W3}\mathbf{V}_B = \text{Ad}_{P \rightarrow W3} \cdot ^P\mathbf{V}_B$$

Where the adjoint matrix accounts for the lever arm:
$$\text{Ad}_{P \rightarrow W3} = \begin{bmatrix} \mathbf{I} & -[\mathbf{p}_{PW3}]_\times \\ \mathbf{0} & \mathbf{I} \end{bmatrix}$$

#### 6.2 Solve for Joint Velocities

Using the weighted damped least squares (WDLS) method:

$$\dot{\mathbf{q}} = \mathbf{J}^{\#}_{\lambda}(\mathbf{q}) \cdot ^{W3}\mathbf{V}_B$$

Where the damped pseudo-inverse is:
$$\mathbf{J}^{\#}_{\lambda} = \mathbf{J}^T(\mathbf{J}\mathbf{J}^T + \lambda^2 \mathbf{I})^{-1}$$

With damping factor $\lambda = 0.1$ for numerical stability.

---

### 7. Command Publishing

#### 7.1 Safety Check

Before publishing, verify all joint velocities are valid:
$$\text{if any } \dot{q}_i = \text{NaN or } |\dot{q}_i| > \dot{q}_{\text{max}} \text{ then } \dot{\mathbf{q}} \leftarrow \mathbf{0}$$

#### 7.2 Publish to Velocity Controller

Send the joint velocity vector $\dot{\mathbf{q}}$ to the robot's velocity controller at 100 Hz.

---

### Key Implementation Details

1. **Diagonal Matrices**: All admittance matrices (M, D, K) are diagonal for computational efficiency and decoupled behavior.

2. **Quaternion Handling**: The orientation error uses quaternions to avoid singularities and ensure shortest-path rotations.

3. **Damped IK**: The WDLS method with λ=0.1 provides robust inverse kinematics even near singularities.

4. **Safety Layers**: Multiple safety constraints (acceleration, velocity, workspace) ensure safe operation.

5. **Fixed Control Rate**: The 100 Hz control loop ensures stable and predictable dynamic behavior.

---

### Parameter Tuning Guidelines

The admittance parameters define the robot's compliance behavior:

- **Mass (M)**: Higher values create more "inertia" - slower response to forces
  - Typical: 5-20 kg for translation, 0.5-2 kg·m² for rotation

- **Damping (D)**: Controls oscillation damping - higher values reduce overshoot
  - Critical damping: $d_i = 2\sqrt{k_i \cdot m_i}$
  - Typical: 50-500 N·s/m for translation, 5-20 N·m·s/rad for rotation

- **Stiffness (K)**: Defines the "spring" pulling back to equilibrium
  - Zero stiffness = pure admittance (no position preference)
  - Typical: 100-1000 N/m for translation, 10-50 N·m/rad for rotation

---

### Stability Analysis

The system is stable if all eigenvalues of the characteristic equation have negative real parts:

$$\text{det}(\mathbf{M}s^2 + \mathbf{D}s + \mathbf{K}) = 0$$

For diagonal matrices, each DOF is independently stable if:
$$d_i > 0 \text{ and } k_i \geq 0 \text{ and } m_i > 0$$

The damping ratio for each DOF:
$$\zeta_i = \frac{d_i}{2\sqrt{k_i \cdot m_i}}$$

Where:
- $\zeta < 1$: Underdamped (oscillatory)
- $\zeta = 1$: Critically damped (fastest non-oscillatory)
- $\zeta > 1$: Overdamped (slow return)