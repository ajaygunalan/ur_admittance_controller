# UR Admittance Controller Architecture

## Why This Controller Exists

Universal Robots only accept **position commands**, not force/torque commands. But many tasks require force control:
- **Assembly**: Insert parts without jamming
- **Polishing**: Maintain constant contact force
- **Human collaboration**: Safe physical interaction

**Solution**: Admittance control - convert forces into compliant motion.

## Core Concept

```
External Force → Virtual Mass-Spring-Damper → Motion
     F(t)      →    M·ẍ + D·ẋ + K·x = F   →   Δx(t)
```

The robot behaves as if it has adjustable:
- **Mass (M)**: How quickly it responds to forces
- **Damping (D)**: How smoothly it moves (no oscillations)
- **Stiffness (K)**: Whether it returns to position (spring behavior)

## System Architecture

```
┌─────────────┐     ┌──────────────────┐     ┌─────────────────┐     ┌───────┐
│ F/T Sensor  │────►│ Admittance Ctrl  │────►│ Joint Traj Ctrl │────►│ Robot │
└─────────────┘     └──────────────────┘     └─────────────────┘     └───────┘
    6D Wrench         Joint References        Position Commands        Motion
    @ 100-200Hz       (chainable i/f)         @ 125Hz streaming      
```

### Why Controller Chaining?
- **Direct memory sharing**: No serialization overhead
- **Deterministic timing**: Guaranteed data delivery
- **Zero-copy**: References point to same memory

## Control Algorithm

### 1. Force Acquisition & Transform
```
                  Transform only if needed
F_tool ─────────────────────────────────────► F_base
         │                                         │
         │ if (ft_frame == base_link)             │
         └────────────────────────────────────────┘
                    Skip transform
```

**Why?** Forces must be in robot base frame for correct motion direction.

### 2. Admittance Dynamics

**Pure Admittance Mode (K=0)**:
```
F_ext ──► [1/M] ──► ∫∫ ──► Δx
            │
            └── [D] ──┘
               Damping
```
Robot moves freely, stays where pushed.

**Impedance Mode (K>0)**:
```
         ┌── [K] ←── (x - x_desired)
         │
F_ext ───┴──► [1/M] ──► ∫∫ ──► Δx
               │
               └── [D] ──┘
```
Robot returns to desired position like a spring.

### 3. Safety Pipeline
```
Cartesian    Apply         Convert        Apply          Output
Velocity ──► Limits ────► to Joints ───► Joint ────────► References
   ẋ         ±0.5m/s        IK          Limits            q_ref
```

## Key Parameters Explained

### Virtual Mass `[X,Y,Z,Rx,Ry,Rz]`
- **Units**: kg for translation, kg·m² for rotation
- **Effect**: Lower = faster response, Higher = slower/stable
- **Example**: `[5,5,5,0.5,0.5,0.5]` - Light and responsive

### Damping Ratio `ζ`
- **Units**: Dimensionless (typically 0.7-1.2)
- **Effect**: 
  - ζ < 1: Underdamped (oscillates)
  - ζ = 1: Critically damped (fastest, no overshoot)
  - ζ > 1: Overdamped (slow, no oscillation)

### Stiffness `K`
- **Units**: N/m (translation), Nm/rad (rotation)
- **Effect**: Spring constant - how strongly robot returns to position
- **Special**: K=0 disables position control entirely

## Real-Time Considerations

### Why Pre-allocated Memory?
```cpp
// BAD - causes allocation in RT
joint_positions_.push_back(value);  // ❌

// GOOD - pre-sized in configure()
joint_positions_[i] = value;        // ✅
```
Dynamic allocation is non-deterministic, can cause control loop timing violations.

### Transform Caching Strategy
```
Non-RT Thread                    RT Thread
─────────────                    ─────────
TF Lookup (may block)            Read cached transform
     ↓                                ↑
Update atomic cache ─────────────────►│
```
TF lookups can block indefinitely. Cache ensures RT thread never waits.

### Lock-Free Publishing
```cpp
if (publisher->trylock()) {      // Non-blocking
    publisher->msg_ = data;
    publisher->unlockAndPublish();
}
// Continue regardless - control > monitoring
```

## Common Use Cases

### Assembly (Peg-in-Hole)
**Problem**: Rigid position control causes jamming  
**Solution**: 
- XY stiffness = 50 N/m (gentle centering)
- Z stiffness = 0 (free insertion)
- Low mass for quick compliance

### Surface Following  
**Problem**: Maintain contact without excessive force  
**Solution**:
- XY stiffness = 0 (follow surface contours)
- Z stiffness = 150 N/m (maintain height)
- Very low Z mass (1 kg) for sensitivity

### Human Collaboration
**Problem**: Safety during physical interaction  
**Solution**:
- Moderate stiffness (80 N/m) - gentle return
- High damping (ζ=1.0) - no oscillation
- Velocity limits for safety

## Implementation Insights

### Why RK4 Integration?
Simple Euler integration accumulates error. RK4 provides:
- 4th order accuracy
- Numerical stability
- Smooth velocity profiles

### Why Gradual Stiffness Engagement?
```
Stiffness
    ↑
  K │      ╱─────── Target
    │    ╱
    │  ╱ ← Ramp over 2s
  0 │╱
    └────────────► Time
```
Prevents sudden forces when switching modes. Critical for safety.

### Drift Prevention
When velocity < threshold for 1s → reset integrators.  
Prevents accumulation of sensor noise causing unwanted motion.

## Performance Profile

The controller must complete within the control period (e.g., 5ms @ 200Hz):
1. **Sensor Read**: Direct array access - negligible
2. **Transform**: Matrix multiply (only if needed) - ~0.1ms
3. **Control Law**: 6x6 matrix ops + RK4 - ~0.2ms
4. **Kinematics**: Plugin-dependent - ~0.5ms typical
5. **Safety Checks**: Simple comparisons - negligible

Total: ~1ms typical, well within 5ms budget.

## Summary

This controller enables force-sensitive tasks on position-controlled robots by implementing a virtual mass-spring-damper system. Key insights:

1. **Admittance vs Impedance**: Same math, different perspective
2. **Real-time safety**: Every design choice prioritizes deterministic timing
3. **Flexibility**: Per-axis control enables task-specific behaviors
4. **Safety first**: Multiple layers of limits and checks

The result: A UR robot that can "feel" and respond to forces naturally.