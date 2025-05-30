# Dual-Mode F/T Sensor Implementation Plan for UR Admittance Controller

## Executive Summary

This document outlines the implementation plan for adding dual-mode force/torque sensor support to the UR Admittance Controller, enabling seamless operation with both hardware interfaces (real robots) and topic-based sensors (Gazebo simulation).

## Architecture Overview

The implementation follows established ROS2 Control patterns while maintaining real-time safety through:
- Pre-allocated memory buffers with TLSF allocator
- Lock-free RealtimeBuffer communication
- Comprehensive timestamp validation
- Cached transform data in non-RT callbacks
- Configurable fallback strategies

## Key Design Decisions

1. **Single Controller, Dual Mode**: Maintains one controller that adapts based on parameters
2. **Triple Buffering**: Maximizes decoupling between RT and non-RT threads
3. **Transform Caching**: Eliminates RT-unsafe tf2 lookups in control loop
4. **Graceful Degradation**: Multiple fallback strategies for robustness

## Implementation Components

### 1. Enhanced Parameter Structure

Parameters are organized into logical groups:
- `sensor_interface`: Core mode selection and topic configuration
- `fallback_strategy`: Behavior on data loss or staleness
- Runtime validation and bounds checking

### 2. Memory Management

- Custom TLSF allocator for deterministic behavior
- Pre-allocated message buffers (3x for triple buffering)
- Fixed-capacity string allocation
- Zero dynamic allocation after initialization

### 3. Data Flow Architecture

```
Topic Mode:
/wrist_ft_sensor → Subscriber Callback → Transform Cache → RealtimeBuffer → RT Update Loop

Hardware Mode:
Hardware Interface → State Interfaces → RT Update Loop
```

### 4. Timing Synchronization

- Sequence number tracking for missed updates
- Timestamp validation with configurable timeout
- Consecutive miss counting for safety triggers
- Clock source consistency (use_sim_time)

### 5. Transform Handling

- Transforms cached in subscriber callback
- Adjoint matrix pre-computation
- Frame ID override capability
- Fallback to existing RT transform mechanism

### 6. Fallback Strategies

Three configurable strategies on data loss:
- `hold_position`: Stop motion, maintain current position
- `use_last`: Continue with previous valid data
- `zero_force`: Assume no external forces

### 7. Lifecycle Management

- Mode-specific initialization in `on_configure()`
- Clean state reset in `on_deactivate()`
- Runtime parameter update notifications
- Graceful mode switching support

## Safety Considerations

1. **RT Thread Protection**: No blocking operations, no memory allocation
2. **Data Freshness**: Continuous validation of sensor timestamps
3. **Transform Validity**: Cached transforms with validity flags
4. **Fallback Activation**: Automatic safety responses on failures
5. **Monitoring**: RT-safe logging of anomalies

## Testing Strategy

1. **Unit Tests**: Mock publishers, data flow validation
2. **Integration Tests**: Full system with Gazebo
3. **RT Performance**: ros2_tracing validation
4. **Timing Tests**: Clock synchronization, latency measurement
5. **Failure Tests**: Fallback strategy verification

## Performance Targets

- Additional latency: <2ms in topic mode
- Memory overhead: ~10KB for buffers
- CPU overhead: <1% for callbacks
- Determinism: No RT violations

## Migration Path

1. Parameters default to hardware mode (backward compatible)
2. Existing configurations work unchanged
3. Topic mode activated by single parameter change
4. No API changes for downstream users

## Future Extensions

- Hot-swapping between modes
- Multiple sensor fusion
- Network transport optimization
- Hardware interface wrapper option

## References

- ROS2 Control documentation
- PickNik Robotics topic_based_ros2_control
- Universal Robots driver patterns
- Real-time systems best practices