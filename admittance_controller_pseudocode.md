
- **Step 0**: read the **wrench** from the robot and **convert to the ODE’s body basis** $B_{\text{des}}$ (this is new and explicit).
- **Step 1**: run the **admittance ODE in** $B_{\text{des}}$ to get the pose offset and its rate.
- **Step 2**: build the **factored adjoints** from the **desired pose** and use them to convert the **offset** and the **offset rate** to **world**.
- **Step 3**: compose the **commanded pose in world**.
- **Step 4**: compute **pose error in world** (measured → commanded).
- **Step 5**: form the **world twist command**.
- **Step 6 (mandatory)**: solve for **joint velocities** and **send** them to the robot.

All quantities carry an explicit time index. Every step shows the matrices you asked to keep separate (translation first, then rotation). No code imports; just math‑style pseudocode.

---

## Notation & helpers (once)

- Pose of tool $B$ w.r.t. world $W$: $X_{WB}=(R_{WB},\,p_{WBo})$.
- Small pose offset $\delta X=[\delta r;\, \delta p]$ (rot first, then trans).
- Hat map for cross product:

$\widehat{u}=\begin{bmatrix}0&-u_z&u_y\\u_z&0&-u_x\\-u_y&u_x&0\end{bmatrix}$.

**Factored adjoints (translation → rotation):**

- Pose‑offset re‑expression (first‑order, no coupling)

$\mathrm{Ad}_{\text{pose,trans}}= \mathrm{diag}(I_3,I_3), \qquad \mathrm{Ad}_{\text{pose,rot}}(R)= \mathrm{diag}(R,R), \qquad \mathrm{Ad}_{\text{pose}}(R,p)=\mathrm{Ad}_{\text{pose,trans}}\;\mathrm{Ad}_{\text{pose,rot}}(R)=\mathrm{diag}(R,R)$.
- Twist / rate re‑expression (full spatial adjoint)
$\mathrm{Ad}_{\text{twist,trans}}(p)= \begin{bmatrix}I_3&0\\ \widehat{p}&I_3\end{bmatrix},\qquad \mathrm{Ad}_{\text{twist,rot}}(R)=\mathrm{diag}(R,R), \qquad \mathrm{Ad}_{\text{twist}}(R,p)=\mathrm{Ad}_{\text{twist,trans}}(p)\;\mathrm{Ad}_{\text{twist,rot}}(R)= \begin{bmatrix}R&0\\ \widehat{p}\,R&R\end{bmatrix}$.

- **Force re‑expression** (for wrenches): use the **dual** of the twist adjoint

$\mathrm{Ad}_{\text{force}}(R,p)=\mathrm{Ad}_{\text{twist}}(R,p)^{\top}$.

To re‑express a wrench $F$ from frame $A$ to frame $B$:  
$F_B=\mathrm{Ad}_{\text{twist}}(X_{BA})^{\top} F_A$, with $X_{BA}=(R_{BA},p_{BA})$.

- Exponential / logarithm on $SO(3)$: $\mathrm{Exp3}(\cdot)$, $\mathrm{Log3}(\cdot)$.
- Spatial Jacobian $J_s(q)$ (maps joint rates to **world‑expressed** tool twist at $B_o$).

---

## Initialization (time $k=0$)

- Control period: $\Delta t$.
- Gains: $M,B,K\in\mathbb{R}^{6\times6}$ (admittance); $K_p^W\in\mathbb{R}^{6\times6}$ (world P gain).
- State (admittance, in $B_{\text{des}}$ axes):

$\delta X_{B_{\text{des}}}^{(0)}=\mathbf{0}_6,\qquad \dot{\delta X}_{B_{\text{des}}}^{(0)}=\mathbf{0}_6$.

- Known at $k=0$: desired pose $X_{WB,\text{des}}^{(0)}=(R_{\text{des}}^{(0)},p_{\text{des}}^{(0)})$, measured pose $X_{WB,\text{meas}}^{(0)}=(R_{\text{meas}}^{(0)},p_{\text{meas}}^{(0)})$, desired twist $V_{WB,\text{des}}^{W,(0)}$ (often $\mathbf{0}$).

---

## Per‑tick loop (compute $k\to k{+}1$)

### **Step 0 — Acquire wrench and re‑express to $B_{\text{des}}$**

1. Read sensor wrench $F_S^{(k)}$ (expressed in the sensor frame $S$).
2. Known transforms at $k$: $X_{WS}^{(k)}$ (sensor in world), $X_{WB,\text{des}}^{(k)}$ (desired tool pose).
3. Build the transform from $S$ to $B_{\text{des}}$:

$X_{B_{\text{des}}S}^{(k)} = \big(X_{WB,\text{des}}^{(k)}\big)^{-1}\,X_{WS}^{(k)}$.

4. **Wrench in $B_{\text{des}}$ axes** (what the ODE needs):

$F_{B\!ext,\,B_{\text{des}}}^{(k)} \;=\; \mathrm{Ad}_{\text{twist}}\!\big(X_{B_{\text{des}}S}^{(k)}\big)^{\top}\; F_S^{(k)}.$

(If your wrench already arrives in world or in $B_{\text{meas}}$, use the same formula with the appropriate $X$.)

---

### **Step 1 — Admittance ODE (in $B_{\text{des}}$ axes)**

$M\,\ddot{\delta X}_{B_{\text{des}}}^{(k)}+B\,\dot{\delta X}_{B_{\text{des}}}^{(k)}+K\,\delta X_{B_{\text{des}}}^{(k)} =F_{B\!ext,\,B_{\text{des}}}^{(k)}$.

Semi‑implicit Euler:

$\dot{\delta X}_{B_{\text{des}}}^{(k+1)}=\dot{\delta X}_{B_{\text{des}}}^{(k)}+\ddot{\delta X}_{B_{\text{des}}}^{(k)}\,\Delta t,\qquad \delta X_{B_{\text{des}}}^{(k+1)}=\delta X_{B_{\text{des}}}^{(k)}+\dot{\delta X}_{B_{\text{des}}}^{(k+1)}\,\Delta t$.

---

### **Step 2 — Build factored adjoints from the desired pose and convert to world**

- (Optionally) roll the desired pose with the planner’s twist:


$X_{WB,\text{des}}^{(k+1)} = \mathrm{Exp6}\big(V_{WB,\text{des}}^{W,(k)}\,\Delta t\big)\;X_{WB,\text{des}}^{(k)}$.

- Construct adjoints **from** $X_{WB,\text{des}}^{(k+1)}=(R_{\text{des}}^{(k+1)},p_{\text{des}}^{(k+1)})$:

$\mathrm{Ad}_{\text{pose}}^{(k+1)}=\mathrm{diag}\big(R_{\text{des}}^{(k+1)},R_{\text{des}}^{(k+1)}\big), \qquad \mathrm{Ad}_{\text{twist}}^{(k+1)}= \begin{bmatrix} R_{\text{des}}^{(k+1)} & 0\\[2pt] \widehat{p_{\text{des}}^{(k+1)}}\,R_{\text{des}}^{(k+1)} & R_{\text{des}}^{(k+1)} \end{bmatrix}$.

- Convert **offset** and **offset rate** to **world**:


$\delta X_W^{(k+1)}=\mathrm{Ad}_{\text{pose}}^{(k+1)}\;\delta X_{B_{\text{des}}}^{(k+1)}, \qquad \dot{\delta X}_W^{(k+1)}=\mathrm{Ad}_{\text{twist}}^{(k+1)}\;\dot{\delta X}_{B_{\text{des}}}^{(k+1)}$.

---

### **Step 3 — Compose the commanded pose in world**

$R_{WB,\text{cmd}}^{(k+1)}=\mathrm{Exp3}\big(\delta r_W^{(k+1)}\big)\;R_{WB,\text{des}}^{(k+1)},\qquad p_{WBo,\text{cmd}}^{(k+1)}=p_{WBo,\text{des}}^{(k+1)}+\delta p_W^{(k+1)}$.

Denote $X_{WB,\text{cmd}}^{(k+1)}=(R_{WB,\text{cmd}}^{(k+1)},p_{WBo,\text{cmd}}^{(k+1)})$.

---

### **Step 4 — Pose error in world (measured → commanded)**

Use measurements at $k$: $X_{WB,\text{meas}}^{(k)}=(R_{WB,\text{meas}}^{(k)},p_{WBo,\text{meas}}^{(k)})$.

$e_{R_W}^{(k)}=\mathrm{Log3}\Big(R_{WB,\text{cmd}}^{(k+1)}\;R_{WB,\text{meas}}^{(k)\,\top}\Big),\qquad e_{p_W}^{(k)}=p_{WBo,\text{cmd}}^{(k+1)}-p_{WBo,\text{meas}}^{(k)}, \qquad e_{X_W}^{(k)}=\begin{bmatrix}e_{R_W}^{(k)}\\ e_{p_W}^{(k)}\end{bmatrix}$.

---

### **Step 5 — World twist command (the task‑space command)**

$V_{WB,\text{cmd}}^{W,(k+1)}=V_{WB,\text{des}}^{W,(k)}\;+\;\dot{\delta X}_W^{(k+1)}\;+\;K_p^W\,e_{X_W}^{(k)}$.

(This is the twist your resolved‑rates step will track.)

---

### **Step 6 — Joint velocities (mandatory) and send to robot**

Compute the **spatial Jacobian** at the current configuration $q^{(k)}$: $J_s(q^{(k)})\in\mathbb{R}^{6\times n}$ such that  
$V_{WB}^{W}=J_s(q)\,\dot{q}$.

Solve for $\dot{q}^{(k)}$ (QP form shown; use bounds/weights as needed):

$\dot{q}^{(k)}=\arg\min_{\dot{q}}\;\big\|J_s(q^{(k)})\,\dot{q}-V_{WB,\text{cmd}}^{W,(k+1)}\big\|^2+\lambda\|\dot{q}\|^2 \quad\text{s.t.}\quad \dot{q}_{\min}\le \dot{q}\le \dot{q}_{\max}$.

Send $\dot{q}^{(k)}$ to the robot actuators.

---

### **Commit state for the next tick**

Set $\delta X_{B_{\text{des}}}^{(k+1)}$, $\dot{\delta X}_{B_{\text{des}}}^{(k+1)}$, and $X_{WB,\text{des}}^{(k+1)}$ as the new state; then increment $k\leftarrow k+1$.

---

### Why this is consistent and easy to read

- The **ODE** runs where compliance is defined: $B_{\text{des}}$.
- **Wrench** is explicitly re‑expressed into $B_{\text{des}}$ using the proper **force adjoint** (Step 0).
- **Offsets** and **rates** are converted to **world** with **factored adjoints** (Step 2).
- **Composition, error, and command** are all in **world**, matching your Jacobian and your driver.
- Time indexing is clear: inputs at $k$ → compute $X_{\text{cmd}}^{(k+1)}$, $V_{\text{cmd}}^{W,(k+1)}$ → solve $\dot{q}^{(k)}$ → send.