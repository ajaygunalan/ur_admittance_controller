## Force & Torque Sensor Filtering

### Problem Statement

A force/torque (F/T) sensor provides raw measurements that contain both signal and noise components. Even after gravity compensation and bias removal, residual noise, sensor drift, and measurement artifacts remain in the processed wrench data. We need to apply appropriate filtering techniques to extract clean force and torque signals while preserving contact dynamics.

### Model

Each filtered measurement satisfies the following signal processing chain when applied to 6-DOF wrench data:

$$\mathbf{W}_{\text{filtered}} = \mathcal{D}(\mathcal{C}(\mathcal{L}(\mathbf{W}_{\text{raw}})))$$

Where:
- $\mathbf{W}_{\text{raw}} = [f_x, f_y, f_z, \tau_x, \tau_y, \tau_z]^T$: Raw 6-DOF wrench measurements from the sensor.
- $\mathcal{L}(\cdot)$: Low-pass filtering operation applied **before** compensation.
- $\mathcal{C}(\cdot)$: Gravity and bias compensation operation.
- $\mathcal{D}(\cdot)$: Deadband filtering operation applied **after** compensation.
- $\mathbf{W}_{\text{filtered}}$: Final processed wrench ready for control input.

### Given

- Raw sensor measurements: $\mathbf{W}_{\text{raw},i} \in \mathbb{R}^6$ at discrete time intervals.
- Calibration parameters: Gravity vector, bias offsets, rotation matrices from previous calibration.
- Filter design requirements: Noise suppression while preserving contact dynamics.

### To Find

- $\alpha \in [0,1]$: Low-pass filter coefficient for optimal noise-signal tradeoff.
- $\mathbf{T}_{\text{deadband}} \in \mathbb{R}^6$: Deadband threshold vector for each wrench component.
- Complete filtering pipeline that integrates with existing compensation.

---

### 1. Low-Pass Filtering - Signal Conditioning

Applied to raw sensor data before compensation to remove high-frequency noise using a first-order IIR filter.

$$H(z) = \frac{\alpha}{1 - (1-\alpha)z^{-1}} \quad \text{where}$$


Default Parameters:
- cutoff frequency $f_c = 200$ Hz 
- angular cutoff frequency $\omega_c = 2\pi f_c = 1256.64$ rad/s
- sampling period $\Delta T = 0.002$ s
- filter coefficient $\alpha = \frac{\omega_c \Delta T}{1 + \omega_c \Delta T} \approx 0.715$ 



Implementation:
$$\mathbf{W}_{\text{lpf}}[n] = \alpha \mathbf{W}_{\text{raw}}[n] + (1-\alpha) \mathbf{W}_{\text{lpf}}[n-1]$$

---

### 2. Deadband Filtering - Contact Detection

Applied to compensated wrench data after gravity and bias removal to suppress residual errors and provide clean zero-force detection.

$$\mathcal{D}(w_i, \tau_i) = \begin{cases} 
0 & \text{if } |w_i| \leq \tau_i \\
w_i & \text{if } |w_i| > \tau_i 
\end{cases}$$

Default Thresholds: $\tau_f = 0.5$ N (forces), $\tau_\tau = 0.01$ Nm (torques)

$$\mathbf{W}_{\text{filtered}} = \begin{bmatrix} \mathcal{D}(\mathbf{f}, \tau_f) \\ \mathcal{D}(\boldsymbol{\tau}, \tau_\tau) \end{bmatrix}$$

