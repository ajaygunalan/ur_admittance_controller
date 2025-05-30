# Architecture

## Concept
**Convert forces into compliant robot motion using admittance control**

```
Force → Virtual Mass-Spring-Damper → Motion
F(t)  →    M·ẍ + D·ẋ + K·x = F    →  Δx(t)
```

## System Pipeline
```
[F/T Sensor] → [Admittance Node] → [Trajectory Controller] → [Robot]
     ↓               ↓                      ↓                   ↓
  /wrist_ft      Force→Velocity      Position Commands      Joint Motion
```

## Implementation

### Node Structure
```
src/
├── admittance_node.cpp          # Main node + control loop
├── admittance_computations.cpp  # Core dynamics M·ẍ + D·ẋ + K·x = F  
└── sensor_handling.cpp          # F/T processing + transforms
```

### Data Flow
1. **Sensor**: F/T data → transform to base frame → filter
2. **Compute**: Admittance law → Cartesian velocity
3. **Output**: Inverse kinematics → joint trajectory commands

### Control Loop
```cpp
void controlLoop() {
  updateSensorData();      // 1. Read F/T sensor
  computeAdmittance();     // 2. M·ẍ + D·ẋ + K·x = F
  publishCommands();       // 3. Send joint trajectory
}
```

## Key Features
- **Real-time**: 500Hz control thread, pre-allocated messages
- **Safety**: Velocity limits, deadband filtering, graceful fallbacks  
- **Integration**: Works with ur_simulation_gz + scaled_joint_trajectory_controller
- **Parameters**: Dynamic reconfiguration via generate_parameter_library

## Performance
- **Frequency**: 500Hz (2ms period)
- **Latency**: <2ms force-to-motion
- **Memory**: Stack allocation, no dynamic allocation in RT loop