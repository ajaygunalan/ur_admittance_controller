# Dual-Mode F/T Sensor Implementation Task List

## Phase 1: Preparation and Setup

### 1.1 Environment Setup
- [ ] Install TLSF allocator: `sudo apt install ros-${ROS_DISTRO}-tlsf-cpp`
- [ ] Install ros2_tracing tools: `sudo apt install ros-${ROS_DISTRO}-ros2trace ros-${ROS_DISTRO}-tracetools`
- [x] Verify Gazebo simulation publishes to `/wrist_ft_sensor` ✓
  - Topic: `/wrist_ft_sensor`
  - Type: `geometry_msgs/msg/WrenchStamped`
  - Frame ID: `ur/ft_sensor_joint/tcp_fts_sensor`
- [x] Document current F/T sensor interface names and frames ✓

### 1.2 Create Feature Branch
- [x] Create git branch: `feature/dual-mode-ft-sensor` ✓
- [ ] Update README with feature description
- [ ] Create test checklist document

## Phase 2: Parameter System Updates

### 2.1 Update Parameter Configuration
- [x] ~~Backup current `admittance_config.yaml`~~ (Using Git version control)
- [x] Add `sensor_interface` parameter group: ✓
  - [x] Add `mode` parameter with validation ✓
  - [x] Add `topic_config` sub-group ✓
  - [x] Add `fallback_strategy` group ✓
- [ ] Update parameter descriptions and constraints
- [ ] Test parameter loading with new structure

### 2.2 Generate Parameter Library
- [x] Run `colcon build` to regenerate parameter headers ✓
- [x] Verify generated header includes new parameters ✓
- [x] Check for compilation errors ✓ (Fixed validation syntax)

## Phase 3: Header File Modifications

### 3.1 Update admittance_types.hpp
- [x] Add `FTSensorData` structure definition ✓
- [x] Add constants for buffer sizes ✓
- [ ] Add fallback strategy enum
- [x] Include necessary headers (WrenchStamped) ✓

### 3.2 Update admittance_controller.hpp
- [x] Add private members: ✓
  - [x] `ft_topic_sub_` subscriber ✓
  - [x] `ft_buffer_storage_` array ✓
  - [x] `rt_ft_buffer_` RealtimeBuffer ✓
  - [x] Atomic tracking variables ✓
  - [x] `ft_sub_options_` for custom allocator ✓
- [x] Add method declarations: ✓
  - [x] `ftSensorCallback()` ✓
  - [x] `handleFallbackStrategy()` ✓
  - [x] `validateSensorData()` ✓
- [ ] Update `updateSensorData()` declaration (no change needed)

## Phase 4: Core Implementation

### 4.1 Implement Memory Pre-allocation
- [x] In `on_init()`: ✓
  - [x] Initialize buffer storage array ✓
  - [x] Pre-allocate string capacities ✓
  - [x] Setup atomic variables ✓
- [ ] Create TLSF allocator setup method (postponed - not critical)
- [x] Implement buffer warm-up routine ✓

### 4.2 Implement Topic Subscriber
- [x] In `on_configure()`: ✓
  - [x] Check sensor mode parameter ✓
  - [x] Create subscriber with custom allocator ✓
  - [x] Configure QoS settings (SensorDataQoS) ✓
  - [x] Initialize RealtimeBuffer ✓
- [x] Implement `ftSensorCallback()`: ✓
  - [x] Copy data to pre-allocated buffer ✓
  - [x] Lookup and cache transform ✓
  - [x] Update sequence number ✓
  - [x] Write to RealtimeBuffer ✓

### 4.3 Modify Sensor Data Reading
- [x] Update `updateSensorData()` in sensor_interface.cpp: ✓
  - [x] Add mode branching logic ✓
  - [x] Implement topic mode reading ✓
  - [x] Add timestamp validation ✓
  - [x] Add sequence number checking ✓
  - [x] Integrate transform caching ✓
- [x] Preserve existing hardware interface code ✓
- [x] Add comprehensive error handling ✓

### 4.4 Implement Fallback Strategies
- [x] Create `handleFallbackStrategy()` method: ✓
  - [x] Implement "hold_position" strategy ✓
  - [x] Implement "use_last" strategy ✓
  - [x] Implement "zero_force" strategy ✓
- [x] Add strategy selection logic ✓
- [x] Integrate with main update loop ✓

## Phase 5: Transform and Timing Management

### 5.1 Transform Caching Implementation
- [x] Add transform caching to callback ✓
- [x] Implement adjoint matrix computation ✓
- [x] Add frame ID override logic ✓
- [x] Handle transform failures gracefully ✓

### 5.2 Timing Synchronization
- [x] Implement timestamp validation ✓
- [x] Add configurable timeout checking ✓
- [x] Track consecutive missed updates ✓
- [x] Add RT-safe logging for timing issues ✓

## Phase 6: Lifecycle and Safety

### 6.1 Update Lifecycle Methods
- [x] Modify `on_configure()`: ✓
  - [x] Add mode-specific initialization ✓
  - [x] Skip F/T interface caching in topic mode ✓
- [x] Modify `on_deactivate()`: ✓
  - [x] Reset topic-specific state ✓
  - [x] Clean up subscribers ✓
- [ ] Update `on_cleanup()`:
  - [ ] Release allocated resources

### 6.2 Safety Features
- [x] Implement maximum missed update checking ✓
- [x] Add automatic safety stop triggers ✓
- [x] Ensure zero-velocity on errors ✓
- [ ] Add diagnostic status publishing (future enhancement)

## Phase 7: Build System Updates

### 7.1 Update CMakeLists.txt
- [ ] Add TLSF dependency: `find_package(tlsf_cpp REQUIRED)` (postponed - using standard allocator)
- [ ] Add tracetools dependency (optional)
- [ ] Update target link libraries

### 7.2 Update package.xml
- [ ] Add `<depend>tlsf_cpp</depend>` (postponed)
- [ ] Add `<test_depend>ros2trace</test_depend>`
- [ ] Update package version

## Phase 8: Testing Implementation

### 8.1 Unit Tests
- [ ] Create test/test_dual_mode_sensor.cpp:
  - [ ] Test parameter loading
  - [ ] Test mode switching logic
  - [ ] Test fallback strategies
  - [ ] Test buffer operations
- [ ] Create mock F/T publisher node
- [ ] Test timestamp validation logic

### 8.2 Integration Tests
- [ ] Create launch/test_topic_mode.launch.py:
  - [ ] Launch controller in topic mode
  - [ ] Launch mock F/T publisher
  - [ ] Verify data flow
- [ ] Test with actual Gazebo simulation
- [ ] Test mode transitions

### 8.3 Performance Tests
- [ ] Create scripts/test_rt_performance.py:
  - [ ] Use ros2_tracing to measure latency
  - [ ] Check for RT violations
  - [ ] Measure callback execution time
- [ ] Document performance baselines
- [ ] Create automated regression tests

## Phase 9: Documentation and Examples

### 9.1 Update Documentation
- [ ] Update README.md with dual-mode instructions
- [ ] Create docs/DUAL_MODE_SETUP.md guide
- [ ] Update admittance_config.yaml with examples
- [ ] Add troubleshooting section

### 9.2 Create Example Configurations
- [ ] Create config/admittance_sim_mode.yaml
- [ ] Create config/admittance_hw_mode.yaml
- [ ] Add launch file examples for both modes

### 9.3 Migration Guide
- [ ] Document parameter changes
- [ ] Create migration checklist
- [ ] Add compatibility notes

## Phase 10: Validation and Integration

### 10.1 System Testing
- [ ] Test with UR5e in Gazebo
- [ ] Verify force/torque response
- [ ] Test all fallback strategies
- [ ] Validate transform accuracy

### 10.2 Code Review Preparation
- [ ] Run clang-format on modified files
- [ ] Run clang-tidy for static analysis
- [ ] Update copyright headers
- [ ] Create pull request description

### 10.3 CI/CD Updates
- [ ] Update GitHub Actions workflow
- [ ] Add topic mode tests to CI
- [ ] Ensure all tests pass
- [ ] Check code coverage

## Phase 11: Future Enhancements (Optional)

### 11.1 Advanced Features
- [ ] Implement runtime mode switching
- [ ] Add sensor fusion capability
- [ ] Create hardware interface wrapper
- [ ] Add network transport optimization

### 11.2 Monitoring and Diagnostics
- [ ] Add ROS2 diagnostics integration
- [ ] Create performance monitoring tools
- [ ] Add data quality metrics
- [ ] Implement anomaly detection

## Completion Checklist

- [ ] All unit tests passing
- [ ] Integration tests verified
- [ ] No RT violations detected
- [ ] Documentation complete
- [ ] Code review approved
- [ ] Performance targets met
- [ ] Backward compatibility confirmed
- [ ] Migration guide tested

## Notes

- Each task should be tested individually before moving to the next
- Commit frequently with descriptive messages
- Keep the original functionality intact
- Document any deviations from the plan
- Run tests after each major phase