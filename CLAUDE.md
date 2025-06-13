# UR Admittance Controller - Development Notes

## Recent Critical Fixes (December 2024)

### 1. Frame Mismatch in Velocity IK (FIXED)
**Problem**: Velocity was computed in base frame but treated as tool frame in IK solver
- Cart velocity `V_tcp_base_commanded_` was in BASE frame
- Code incorrectly treated it as TOOL frame before IK
- Caused wrong motion directions, especially in Y/Z axes

**Fix**: Properly transform TCP velocity to wrist_3 velocity while keeping base frame
```cpp
// Now correctly computes wrist_3 velocity from TCP velocity (both in base frame)
KDL::Vector p_diff = tcp_frame.p - wrist3_frame.p;
twist_wrist3_base.vel = twist_tcp_base.vel - twist_tcp_base.rot * p_diff;
```

### 2. Unlimited Rotational Motion (FIXED)
**Problem**: Only translational acceleration/velocity were limited
- Forces at TCP create torques due to tool offset (Ä = d × F)
- No limits on rotational acceleration/velocity
- Orientation terms dominated motion when pushing off-axis

**Fix**: Added rotational limits
- Max rotational acceleration: 2.0 rad/s²
- Max rotational velocity: 3.0 rad/s
- Prevents violent rotations from tool offset torques

### 3. Quaternion Convention (FIXED)
**Problem**: Used negative-w quaternion [-0.00, -0.71, 0.71, 0.00]
- Mathematically valid but non-standard
- Could cause interpolation issues

**Fix**: Changed to positive-w convention [0.00, 0.71, -0.71, 0.00]

### 4. Log Spam (FIXED)
**Problem**: INFO messages flooded terminal at 100Hz
**Fix**: Changed to INFO_ONCE and INFO_THROTTLE

### 5. Uniform Parameters (UPDATED)
**Change**: Made all admittance parameters uniform
- K = [10,10,10,10,10,10] N/m (was 20 for Y)
- D = [10,10,10,10,10,10] Ns/m (was 12 for translation)
- M = [2,2,2,2,2,2] kg remains same
- Robot now behaves identically in all directions

## Architecture Decisions

### Why Velocity-Based Control
- UR robots accept joint velocity commands via `/forward_velocity_controller/commands`
- 100Hz control loop matches ros2_control expectations
- Simple integration: acceleration ’ velocity ’ position

### Frame Conventions
- All computations in base frame for consistency
- KDL chain: base_link ’ wrist_3_link (6 joints)
- Tool offset: wrist_3_link ’ tool_payload (fixed transform)
- Wrench input expected in base frame at TCP

### Safety Philosophy
- Hard workspace limits with immediate velocity zeroing
- IK failure triggers immediate stop (industry standard)
- Acceleration and velocity limits on all 6 DOF
- No defensive programming where guarantees exist (487Hz joint states)

### Parameter Tuning Guidelines
- Mass: Higher = slower response (typically 1-5 kg for translation)
- Damping: Higher = more resistance (typically 5-20 Ns/m)
- Stiffness: Higher = stronger return to equilibrium (0 = pure admittance)
- Rotational mass should be ~0.1-1.0 kgÅm² (currently 2.0)

## Known Issues & Future Work

1. **Workspace Limiter**: Currently hard clips velocity at boundaries
   - Could implement soft boundaries with smooth deceleration
   - Prevents "sliding along wall" effect

2. **Tool Offset Torques**: Pure forces create torques due to lever arm
   - Consider separate admittance ratios for force vs torque
   - Or transform wrench to point closer to wrist_3

3. **Parameter Units**: Rotational mass (kgÅm²) may be too high
   - Consider reducing to 0.5-1.0 for more responsive rotation

## Testing Checklist
- [ ] Push in +X/-X: Should move smoothly along X
- [ ] Push in +Y/-Y: Should move along Y (not X or diagonal)
- [ ] Push in +Z/-Z: Should move along Z
- [ ] Push near workspace boundary: Should decelerate and stop
- [ ] Apply pure torque: Should rotate without translation
- [ ] Push off-center: Should see limited rotation due to caps

## Debug Commands
```bash
# Run with debug logging
ros2 run ur_admittance_controller admittance_node --ros-args --log-level debug

# Monitor forces and velocities
ros2 topic echo /wrench_tcp_base
ros2 topic echo /forward_velocity_controller/commands

# Check current parameters
ros2 param get /admittance_node admittance.mass
ros2 param get /admittance_node admittance.damping
ros2 param get /admittance_node admittance.stiffness
```