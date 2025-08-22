## Force Sensor Calibration

### Problem Statement

A force sensor mounted on a robot wrist measures forces that include both contact forces and non-contact components (gravity and sensor bias). We need to identify calibration parameters to extract pure contact forces.

### Model

Each force measurement satisfies:

$$^s\mathbf{F} = {^s_e}\mathbf{R} \cdot {^e_b}\mathbf{R} \cdot \mathbf{F}_b + {^s}\mathbf{F}_0$$

Where:

- $^s\mathbf{F}_i$: Measured force in sensor frame
- ${^s_e}\mathbf{R}$: Unknown rotation from end-effector to sensor
- ${^e_b}\mathbf{R}_i$: Known rotation from base to end-effector (from kinematics)
- $\mathbf{F}_b$: Unknown gravitational force in base frame (gravity-aligned)
- $^s\mathbf{F}_0$: Unknown force bias in sensor frame

### Given

- n force measurements: $^s\mathbf{F}_i \in \mathbb{R}^3$ (from sensor)
- n robot orientations: ${^e_b}\mathbf{R}_i \in SO(3)$ (from forward kinematics)
- Robot installation angle: $\alpha \in [0°, 360°]$ (determines gravity direction in base frame)

### To Find

- $\mathbf{F}_b \in \mathbb{R}^3$: Gravitational force in robot base frame (properly aligned with gravity)
- ${^s_e}\mathbf{R} \in SO(3)$: Rotation matrix from end-effector to sensor
- $^s\mathbf{F}_0 \in \mathbb{R}^3$: Force bias in sensor frame

---

### 1. LROM - Estimate $\mathbf{F}_b$ with Gravity Alignment

**1.1 Reformulate Model**

 Pre-multiply by ${^s_e}\mathbf{R}^T$:

$${^s_e}\mathbf{R}^T \cdot {^s}\mathbf{F} - {^e_b}\mathbf{R} \cdot \mathbf{F}_b - {^s_e}\mathbf{R}^T \cdot {^s}\mathbf{F}_0 = 0$$

 **1.2 Build Unknown Vector (X)**

$$\mathbf{x} = \begin{bmatrix} \mathbf{x}_9 \\ \mathbf{x}_6 \end{bmatrix} \in \mathbb{R}^{15 \times 1}$$

where $\mathbf{x}_9$ is unknown rotation matrix (${^s_e}\mathbf{R}^T$) and $\mathbf{x}_6$ is unknown gravitational force vector $\mathbf{F}_b$ and the transformed bias term.

$$\mathbf{x}_9 = \begin{bmatrix} \mathbf{r}_1^T \\ \mathbf{r}_2^T \\ \mathbf{r}_3^T \end{bmatrix}$$
$$\mathbf{x}_6 = \begin{bmatrix} \mathbf{F}_b \\ {^s_e}\mathbf{R}^T \cdot {^s}\mathbf{F}_0 \end{bmatrix}$$


**1.3 Construct Coefficient Matrix (A)**

$$\mathbf{A} = \begin{bmatrix}
\mathbf{A}_1 \\
\mathbf{A}_2 \\
\vdots \\
\mathbf{A}_n
\end{bmatrix} = \begin{bmatrix}
\mathbf{A}_{1,R} & \mathbf{A}_{1,F} & \mathbf{A}_{1,0} \\
\mathbf{A}_{2,R} & \mathbf{A}_{2,F} & \mathbf{A}_{2,0} \\
\vdots & \vdots & \vdots \\
\mathbf{A}_{n,R} & \mathbf{A}_{n,F} & \mathbf{A}_{n,0}
\end{bmatrix} \in \mathbb{R}^{3n \times 15}$$

where $\mathbf{A}_i = \begin{bmatrix} \mathbf{A}_{i,R} & \mathbf{A}_{i,F} & \mathbf{A}_{i,0} \end{bmatrix} \in \mathbb{R}^{3 \times 15}$ corresponds to measurement $i$

$$\mathbf{A}_{i,R} = \begin{bmatrix}
\mathbf{s}_{F_i}^T & \mathbf{0}_{1 \times 3} & \mathbf{0}_{1 \times 3} \\
\mathbf{0}_{1 \times 3} & \mathbf{s}_{F_i}^T & \mathbf{0}_{1 \times 3} \\
\mathbf{0}_{1 \times 3} & \mathbf{0}_{1 \times 3} & \mathbf{s}_{F_i}^T
\end{bmatrix} \in \mathbb{R}^{3 \times 9} $$

$$\mathbf{A}_{i,F} = -{}^{e}\mathbf{R}_i \in \mathbb{R}^{3 \times 3}$$

$$\mathbf{A}_{i,0} = -\mathbf{I}_{3 \times 3} \in \mathbb{R}^{3 \times 3}$$


 **1.4 Define Constraint**

The rotation part  ($x_9$) of the unknown parameter vector $x$  must satisfy the orthogonality property, which means $||{^s_e}\mathbf{R}^T||_F^2 = 3$. This is rephrased as a unit-norm constraint $||\mathbf{B} \cdot \mathbf{x}||_F^2 = 1$ to simplify the optimization.

$$\min ||A\mathbf{x}||_F^2 \quad \text{s.t.} \quad ||B\mathbf{x}||_F^2 = 1$$

where: $$\mathbf{B} = \begin{bmatrix} \frac{1}{\sqrt{3}} \mathbf{I}_9 & \mathbf{0}_{9 \times 6} \end{bmatrix} \in \mathbb{R}^{9 \times 15}$$

Verification: $$||\mathbf{B} \cdot \mathbf{x}||_F^2 = \left|\left|\frac{1}{\sqrt{3}} \mathbf{x}_9\right|\right|_F^2 = \frac{1}{3}||\mathbf{x}_9||_F^2 = 1 \implies ||\mathbf{x}_9||_F^2 = 3$$


**1.5 Variable Transformation**

To make the constraint even simpler, a new variable $y$ is introduced such that the constraint becomes $||y||_F^2 = 1$.

- The transformation is defined as: $$y = \frac{\sqrt{3}}{3}\mathbf{I}_9 \cdot \mathbf{x}_9$$
    
- The inverse transformation, used for recovering the final result, is: $$\mathbf{x}_9 = \sqrt{3} \cdot \mathbf{I}_9 \cdot y$$


**1.6 Solve  $\mathbf{x}_6$**

With the rotation part of the problem now expressed in terms of $\mathbf{y}$, the optimal value for the remaining parameter vector, $\mathbf{x}_6$, can be solved for directly as a unconstrained least-square problem. For any given $\mathbf{y}$, the $\mathbf{x}_6$ that minimizes the cost function is:

$$\mathbf{x}_6 = -(\mathbf{A}_6^T \mathbf{A}_6)^{-1} \mathbf{A}_6^T \mathbf{A}_9 (\sqrt{3} \cdot \mathbf{I}_9) \mathbf{y}$$


 **1.7 Sub $x_{6}$ to simplify**

By substituting the optimal expression for $x_6$ back into the original cost function $||A\mathbf{x}||^2$, the problem is reduced to an optimization over the single variable $y$:

$$\min ||Hy||_F^2 \quad \text{s.t.} \quad ||y||_F^2 = 1$$

Where the matrix $H$ is defined as:

$$H = (I - A_6(A_6^T A_6)^{-1}A_6^T)A_9(\sqrt{3} \cdot \mathbf{I}_9)$$


 **1.8 Solve Eigenvalue Problem**
The solution to above simplified problem  ($\mathbf{y}^*$) is the eigenvector corresponding to the minimum eigenvalue of $\mathbf{H}^T\mathbf{H}$. This is a standard result for minimizing $||\mathbf{H}\mathbf{y}||^2$ subject to the unit-norm constraint $||\mathbf{y}||^2 = 1$, a principle known as the [[rayleigh_quotient]]



 **1.9 Extract Raw $\mathbf{F}_b$**

Once $\mathbf{y}^*$ is found, substitute it back into the equations for $\mathbf{x}_9$ and $\mathbf{x}_6$ to recover the full optimal vector $\mathbf{x}^*$.

$$F_{b,\text{raw}} = \begin{bmatrix} x_{10}^* \\ x_{11}^* \\ x_{12}^* \end{bmatrix}$$

 **1.10 Apply Gravity-Based Sign Correction**

The final vector $\hat{\mathbf{F}}_b$ is determined by correcting the sign of $\mathbf{F}_{b,\text{raw}}$ based on the installation angle $\alpha$. The least-squares solution correctly finds the axis of the gravity vector, but its direction is ambiguous. The known installation angle $\alpha$ resolves this sign ambiguity. For upright ground-mounted robots ($\alpha \in (0°, 80°] \cup [280°, 360°]$), ensure $\hat{F}_{b,z} < 0$ (pointing down).
    



### 2. Estimate ${^s_e}\mathbf{R}$ and $^s\mathbf{F}_0$

With $\hat{\mathbf{F}}_b$ known, the problem reduces to finding the optimal rotation and translation between two sets of corresponding points. This is a classic 3D registration task known as  [[procrustes_problem]].

**2.1 Calculate Mean Values**

$$\bar{^s\mathbf{F}} = \frac{1}{n} \sum {^s}\mathbf{F}_i$$

$$\bar{^e_b\mathbf{R}} = \frac{1}{n} \sum {^e_b}\mathbf{R}_i$$

 **2.2 Construct the covariance matrix $\mathbf{D}$ from the centered data**

$$\mathbf{D} = \sum_{i=1}^{n} ({^s}\mathbf{F}_i - \bar{^s\mathbf{F}}) \cdot (({^e_b}\mathbf{R}_i - \bar{^e_b\mathbf{R}}) \cdot \hat{\mathbf{F}}_b)^T$$


-  The matrix $\mathbf{D}$ is a **correlation matrix** . Its structure, $\sum (\text{target point}) \cdot (\text{source point})^T$, is designed to find the optimal rotation between two sets of corresponding points.
-   By subtracting the mean from both the measured sensor forces ($^s\mathbf{F}_i - \bar{^s\mathbf{F}}$) and the transformed gravitational forces ($(^{e}\mathbf{R}_i - \bar{^e_b\mathbf{R}}) \cdot \hat{\mathbf{F}}_b$), the translational component (in this context, the sensor bias $^s\mathbf{F}_0$) is decoupled from the rotational component (${^s_e}\mathbf{R}$). This allows the rotation to be solved independently and robustly.

 **2.3 SVD of D**

$$\mathbf{D} = \mathbf{U} \mathbf{\Sigma} \mathbf{V}^T$$

 **2.4 Compute ${^s_e}\mathbf{R}$**

The optimal rotation matrix $\hat{^s_e\mathbf{R}}$ is found using the components of the SVD. A special check on the determinant ensures the result is a valid right-handed rotation matrix. Check [[procrustes_problem]] for further details.

$$\hat{^s_e\mathbf{R}} = \mathbf{U} \cdot \text{diag}(1, 1, \det(\mathbf{U})\det(\mathbf{V})) \cdot \mathbf{V}^T \quad \text{if} \quad \text{rank}(\mathbf{D}) \geq 1$$

 **2.5 Compute $^s\mathbf{F}_0$**

The force bias is calculated using the mean values and the estimated rotation:

$$\hat{^s\mathbf{F}}_0 = \bar{^s\mathbf{F}} - \hat{^s_e\mathbf{R}} \cdot \bar{^e_b\mathbf{R}} \cdot \hat{\mathbf{F}}_b$$

---

### 3. Force Compensation

 **3.1 Apply Compensation**

For any new measurement ${^s}\mathbf{F}_{\text{measured}}$ at a current robot pose ${^e_b}\mathbf{R}_{\text{current}}$, the pure contact force is found by subtracting the calculated gravity and bias components:

$${^s}\mathbf{F}_{\text{contact}} = {^s}\mathbf{F}_{\text{measured}} - (\hat{^s_e\mathbf{R}} \cdot {^e_b}\mathbf{R}_{\text{current}} \cdot \hat{\mathbf{F}}_b + \hat{^s\mathbf{F}}_0)$$

---

### Key Insights

1. **LROM** cleverly avoids directly optimizing over the non-linear rotation constraints of $SO(3)$ by temporarily relaxing them and solving a linear system.
    
2. Once $\mathbf{F}_b$ is known, finding ${^s_e}\mathbf{R}$ becomes a standard, solvable 3D registration problem (Wahba's problem).
    
3. The force bias $^s\mathbf{F}_0$ falls out naturally from the mean values once the other parameters are known.
    
4. The entire process requires only linear algebra operations (matrix inversions, eigenvalue decomposition, and SVD), making it efficient and non-iterative.