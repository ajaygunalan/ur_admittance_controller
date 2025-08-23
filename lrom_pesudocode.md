## Force & Torque Sensor Calibration

### Problem Statement

A force/torque (F/T) sensor mounted on a robot wrist measures forces and torques that include both contact components and non-contact components (gravity, sensor bias). We need to identify a complete set of calibration parameters to extract the pure contact wrench (force and torque).

### Model

Each force and torque measurement satisfies the following relationships when the robot is static:

$$^s\mathbf{F} = ^e_s\mathbf{R} \cdot ^b_e\mathbf{R} \cdot \mathbf{F}^b + ^s\mathbf{F}_0$$

$$^s\mathbf{T} = ^g_s\mathbf{P}^\wedge \cdot (^s\mathbf{F} - ^s\mathbf{F}_0) + ^s\mathbf{T}_0$$

Where:

- $^s\mathbf{F}_i, ^s\mathbf{T}_i$: Measured force and torque in the sensor frame {S}.
- $^e_s\mathbf{R}$: Unknown rotation from the end-effector frame {E} to the sensor frame {S}.
- $^b_e\mathbf{R}_i$: Known rotation from the base frame {B} to the end-effector frame {E}.
- $\mathbf{F}^b$: Unknown gravitational force vector of the tool, expressed in the robot base frame {B}.
- $^s\mathbf{F}_0, ^s\mathbf{T}_0$: Unknown force and torque bias (offset) in the sensor frame {S}.
- $^g_s\mathbf{P}$: Unknown position vector of the tool's center of gravity, expressed in the sensor frame {S}.
- $(\cdot)^\wedge$: The skew-symmetric matrix operator.

### Given

- `n` force/torque measurements: $^s\mathbf{F}_i, ^s\mathbf{T}_i \in \mathbb{R}^3$.
- `n` corresponding robot orientations: $^b_e\mathbf{R}_i \in SO(3)$.
- Robot installation angle: $\alpha \in [0°, 360°]$ (resolves gravity direction ambiguity).

### To Find

- $\mathbf{F}^b \in \mathbb{R}^3$: Gravitational force in the robot base frame.
- $^e_s\mathbf{R} \in SO(3)$: Rotation from end-effector to sensor.
- $^s\mathbf{F}_0 \in \mathbb{R}^3$: Force bias in the sensor frame.
- $^s\mathbf{T}_0 \in \mathbb{R}^3$: Torque bias in the sensor frame.
- $^g_s\mathbf{P} \in \mathbb{R}^3$: Tool's center of gravity in the sensor frame.
- $mg$: Mass of the tool.

---

### 1. LROM - Estimate Gravitational Force Vector ($\mathbf{F}^b$)

This section uses the **Limited Robot Orientation Method (LROM)** to find an initial estimate for the gravity vector $\mathbf{F}^b$ by solving a constrained linear least-squares problem.

#### 1.1 Reformulate Force Model

Pre-multiply the force equation by $^e_s\mathbf{R}^T$ to linearize the unknown parameters:

$$^e_s\mathbf{R}^T \cdot ^s\mathbf{F} - ^b_e\mathbf{R} \cdot \mathbf{F}^b - ^e_s\mathbf{R}^T \cdot ^s\mathbf{F}_0 = \mathbf{0}$$

#### 1.2 Build Unknown Vector ($\mathbf{x}$)

Group all unknown terms into a single vector $\mathbf{x} \in \mathbb{R}^{15}$:

$$\mathbf{x} = \begin{bmatrix} \mathbf{x}_9 \\ \mathbf{x}_6 \end{bmatrix} \text{ where } \mathbf{x}_9 = \text{vec}(^e_s\mathbf{R}^T) = \begin{bmatrix} \mathbf{r}_1^T \\ \mathbf{r}_2^T \\ \mathbf{r}_3^T \end{bmatrix} \text{ and } \mathbf{x}_6 = \begin{bmatrix} \mathbf{F}^b \\ ^e_s\mathbf{R}^T \cdot ^s\mathbf{F}_0 \end{bmatrix}$$

#### 1.3 Construct Coefficient Matrix ($\mathbf{A}$)

For `n` measurements, build the system $\mathbf{A}\mathbf{x} = \mathbf{0}$:

$$\mathbf{A} = \begin{bmatrix} \mathbf{A}_1 \\ \vdots \\ \mathbf{A}_n \end{bmatrix} \in \mathbb{R}^{3n \times 15}, \text{ where } \mathbf{A}_i = [\mathbf{A}_{i,R} \quad \mathbf{A}_{i,F} \quad \mathbf{A}_{i,0}]$$

$$\mathbf{A}_{i,R} = \begin{bmatrix} (^s\mathbf{F}_i)^T & 0 & 0 \\ 0 & (^s\mathbf{F}_i)^T & 0 \\ 0 & 0 & (^s\mathbf{F}_i)^T \end{bmatrix}, \quad \mathbf{A}_{i,F} = -^b_e\mathbf{R}_i, \quad \mathbf{A}_{i,0} = -\mathbf{I}_3$$

#### 1.4 Define Orthogonality Constraint

The columns of a rotation matrix are orthogonal and have unit length, meaning $\|^e_s\mathbf{R}^T\|_F^2 = 3$. This is reformulated as a unit-norm constraint on the rotational part of $\mathbf{x}$:

$$\min \|\mathbf{A}\mathbf{x}\|_F^2 \quad \text{s.t.} \quad \|\mathbf{B}\mathbf{x}\|_F^2 = 1$$

Where $\mathbf{B} = [\frac{1}{\sqrt{3}}\mathbf{I}_9 \quad \mathbf{0}_{9 \times 6}]$ isolates and normalizes $\mathbf{x}_9$.

#### 1.5 Solve for $\mathbf{x}$ via Eigenvalue Decomposition

This constrained problem can be simplified by substituting out the unconstrained part ($\mathbf{x}_6$) and solving for the rest.

1. **Variable Transformation:** Define a new variable $\mathbf{y} = \frac{1}{\sqrt{3}}\mathbf{x}_9$ so the constraint becomes $\|\mathbf{y}\|_2 = 1$.

2. **Substitution:** Solve for $\mathbf{x}_6$ in terms of $\mathbf{y}$. This gives
   $$\mathbf{x}_6 = -(\mathbf{A}_6^T \mathbf{A}_6)^{-1} \mathbf{A}_6^T \mathbf{A}_9 (\sqrt{3}\mathbf{y})$$

3. **Simplify:** Substitute $\mathbf{x}_6$ back into the cost function to get the form $\min \|\mathbf{H}\mathbf{y}\|_F^2$ s.t. $\|\mathbf{y}\|_F^2 = 1$, where
   $$\mathbf{H} = (\mathbf{I} - \mathbf{A}_6(\mathbf{A}_6^T \mathbf{A}_6)^{-1}\mathbf{A}_6^T)\mathbf{A}_9 \sqrt{3}$$

4. **Solve:** The optimal $\mathbf{y}^*$ is the eigenvector corresponding to the minimum eigenvalue of $\mathbf{H}^T\mathbf{H}$  [[rayleigh_quotient]]

5. **Recover:** Back-substitute $\mathbf{y}^*$ to find the full solution vector $\mathbf{x}^*$.

#### 1.6 Extract and Correct $\mathbf{F}^b$

The raw gravitational force is extracted directly from the solution: $\mathbf{F}^b_{\text{raw}} = [x_{10}^*, x_{11}^*, x_{12}^*]^T$. The known installation angle $\alpha$ is then used to correct its sign, as the least-squares solution is ambiguous to direction. For a typical ground-mounted robot ($\alpha \approx 0°$), the z-component should be negative.

$$\hat{\mathbf{F}}^b = \text{sign}(\ldots) \cdot \mathbf{F}^b_{\text{raw}}$$

---

### 2. Estimate Rotation ($^e_s\mathbf{R}$) and Force Bias ($^s\mathbf{F}_0$)

With the gravity vector $\hat{\mathbf{F}}^b$ known, the force model becomes linear in the remaining parameters. This is a classic 3D registration problem, solvable with the  [[procrustes_problem]].

#### 2.1 Center the Data

Define two sets of corresponding points. The "source" points are the gravity vectors transformed by the robot's orientation, and the "target" points are the measured forces.

$$\mathbf{p}_i = ^b_e\mathbf{R}_i \cdot \hat{\mathbf{F}}^b \quad \text{and} \quad \mathbf{q}_i = ^s\mathbf{F}_i$$

The problem is to find the rotation $^e_s\mathbf{R}$ and translation $^s\mathbf{F}_0$ that best align these point sets. Decouple rotation from translation by centering the data:

$$\mathbf{p}_i' = \mathbf{p}_i - \bar{\mathbf{p}} \quad \text{and} \quad \mathbf{q}_i' = \mathbf{q}_i - \bar{\mathbf{q}}$$

#### 2.2 Construct Covariance Matrix ($\mathbf{D}$)

The covariance matrix captures the correlation between the centered point sets:

$$\mathbf{D} = \sum_{i=1}^n \mathbf{q}_i' \cdot (\mathbf{p}_i')^T = \sum_{i=1}^n (^s\mathbf{F}_i - \overline{^s\mathbf{F}}) \cdot ((^b_e\mathbf{R}_i \cdot \hat{\mathbf{F}}^b) - (\overline{^b_e\mathbf{R}} \cdot \hat{\mathbf{F}}^b))^T$$

#### 2.3 Compute $^e_s\mathbf{R}$ via SVD

The optimal rotation is found via Singular Value Decomposition (SVD) of $\mathbf{D} = \mathbf{U}\boldsymbol{\Sigma}\mathbf{V}^T$.

$$\hat{^e_s\mathbf{R}} = \mathbf{U} \cdot \text{diag}(1, 1, \det(\mathbf{U}\mathbf{V}^T)) \cdot \mathbf{V}^T$$

The determinant check ensures the result is a valid right-handed rotation matrix.

#### 2.4 Compute $^s\mathbf{F}_0$

The force bias is the difference between the centroids, transformed by the estimated rotation:

$$\hat{^s\mathbf{F}_0} = \bar{\mathbf{q}} - \hat{^e_s\mathbf{R}} \cdot \bar{\mathbf{p}} = \overline{^s\mathbf{F}} - \hat{^e_s\mathbf{R}} \cdot \overline{^b_e\mathbf{R}} \cdot \hat{\mathbf{F}}^b$$

---

### 3. Estimate Torque Bias ($^s\mathbf{T}_0$) and Center of Gravity ($^g_s\mathbf{P}$)

Now that $\hat{^s\mathbf{F}_0}$ is known, the torque model can be solved using a standard linear least-squares approach.

#### 3.1 Linearize Torque Model

The torque model is rewritten to group the unknown parameters $^g_s\mathbf{P}$ and $^s\mathbf{T}_0$:

$$^s\mathbf{T}_i = -(^s\mathbf{F}_i - \hat{^s\mathbf{F}_0})^\wedge \cdot ^g_s\mathbf{P} + \mathbf{I}_3 \cdot ^s\mathbf{T}_0$$

This is a linear system of the form $\mathbf{b} = \mathbf{C}\mathbf{y}$.

#### 3.2 Construct and Solve the Linear System

For `n` measurements, we build the system $\mathbf{b} = \mathbf{C}\mathbf{y}$ where:

$$\mathbf{b} = \begin{bmatrix} ^s\mathbf{T}_1 \\ \vdots \\ ^s\mathbf{T}_n \end{bmatrix} \in \mathbb{R}^{3n \times 1}$$

$$\mathbf{C} = \begin{bmatrix} -(^s\mathbf{F}_1 - \hat{^s\mathbf{F}_0})^\wedge & \mathbf{I}_3 \\ \vdots & \vdots \\ -(^s\mathbf{F}_n - \hat{^s\mathbf{F}_0})^\wedge & \mathbf{I}_3 \end{bmatrix} \in \mathbb{R}^{3n \times 6}$$

$$\mathbf{y} = \begin{bmatrix} ^g_s\mathbf{P} \\ ^s\mathbf{T}_0 \end{bmatrix} \in \mathbb{R}^{6 \times 1}$$


The solution that minimizes the squared error $\|\mathbf{b} - \mathbf{C}\mathbf{y}\|_2$ is given by the normal equations:

$$\hat{\mathbf{y}} = (\mathbf{C}^T\mathbf{C})^{-1}\mathbf{C}^T\mathbf{b}$$

From $\hat{\mathbf{y}}$, we directly obtain the estimates $\hat{^g_s\mathbf{P}}$ and $\hat{^s\mathbf{T}_0}$.

---

### 4. Decompose Gravitational Force Vector

The estimated vector $\hat{\mathbf{F}}^b$ can be further decomposed to find the tool's scalar mass and the robot's installation angles relative to the true gravity frame.

#### 4.1 Define Installation Model

The gravity vector in the base frame, $\mathbf{F}^b$, is the result of rotating the pure gravity vector $\mathbf{F}_{mg} = [0, 0, -mg]^T$ from the gravity frame {G} to the base frame {B}. This rotation $^g_b\mathbf{R}$ can be modeled using Tait-Bryan angles, primarily pitch ($\beta$) and roll ($\alpha$).

$$\hat{\mathbf{F}}^b = ^g_b\mathbf{R}(\alpha, \beta) \cdot \mathbf{F}_{mg} = -mg \begin{bmatrix} \cos\alpha \sin\beta \\ -\sin\alpha \\ \cos\alpha \cos\beta \end{bmatrix}$$

#### 4.2 Compute Mass and Installation Angles

By inverting the relationship, we can solve for the physical parameters from the components of $\hat{\mathbf{F}}^b = [\hat{f}_{bx}, \hat{f}_{by}, \hat{f}_{bz}]^T$:

- **Tool Mass ($mg$):** The magnitude of the vector is the gravitational force.
  $$mg = \|\hat{\mathbf{F}}^b\|_2 = \sqrt{\hat{f}_{bx}^2 + \hat{f}_{by}^2 + \hat{f}_{bz}^2}$$

- **Installation Roll Angle ($\alpha$):**
  $$\alpha = \arctan2(-\hat{f}_{by}, \sqrt{\hat{f}_{bx}^2 + \hat{f}_{bz}^2})$$

- **Installation Pitch Angle ($\beta$):**
  $$\beta = \arctan2(\hat{f}_{bx}, \hat{f}_{bz})$$

---

### 5. Full Force & Torque Compensation

With all calibration parameters identified, we can now subtract the non-contact components from any new measurement to find the pure contact wrench.

#### 5.1 Apply Compensation

For a new measurement $(^s\mathbf{F}_{\text{meas}}, ^s\mathbf{T}_{\text{meas}})$ at a current robot pose $^b_e\mathbf{R}_{\text{curr}}$, the contact components are:

- **Contact Force:**
  $$^s\mathbf{F}_{\text{contact}} = ^s\mathbf{F}_{\text{meas}} - (\hat{^e_s\mathbf{R}} \cdot ^b_e\mathbf{R}_{\text{curr}} \cdot \hat{\mathbf{F}}^b + \hat{^s\mathbf{F}_0})$$

- **Contact Torque:**
  $$^s\mathbf{T}_{\text{contact}} = ^s\mathbf{T}_{\text{meas}} - (\hat{^g_s\mathbf{P}}^\wedge \cdot (\hat{^e_s\mathbf{R}} \cdot ^b_e\mathbf{R}_{\text{curr}} \cdot \hat{\mathbf{F}}^b) + \hat{^s\mathbf{T}_0})$$

---

### Key Insights

1. **Sequential Solution:** The problem is solved sequentially. The force parameters are found first, which then enables the straightforward, linear solution for the torque parameters.

2. **Linearization is Key:** The LROM method cleverly linearizes the force problem by reformulating the equation and relaxing the rotation constraint, allowing it to be solved efficiently using linear algebra.

3. **Standard Problems:** Once the gravity vector $\mathbf{F}^b$ is estimated, the subsequent steps reduce to well-known problems: the Procrustes problem for rotation/translation and standard least-squares for the torque parameters.

4. **Complete Identification:** This complete, non-iterative process identifies all six bias parameters ($^s\mathbf{F}_0, ^s\mathbf{T}_0$), the sensor-to-tool transform ($^e_s\mathbf{R}, ^g_s\mathbf{P}$), the tool's mass ($mg$), and the robot's installation orientation ($\alpha, \beta$).