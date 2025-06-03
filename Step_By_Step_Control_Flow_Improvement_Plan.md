# Step-by-Step Control Flow Improvement Plan

## Goal
Transform our complex control_cycle() with multiple checks into a clean, ROS1-style control loop by moving all initialization to the startup phase.

## Current State Analysis

### Problems in control_cycle():
1. Checks kinematics_initialized_ every cycle
2. Checks joint_states_received_ every cycle  
3. Tries to initialize_desired_pose() every cycle until successful
4. Does TF lookup for TCP pose every cycle
5. Mixed initialization logic with control logic

## Implementation Plan

### Phase 1: Create Initialization Infrastructure (30 min)

#### Step 1.1: Add wait_for_robot_ready() method
```cpp
// In admittance_node.hpp (private section):
void wait_for_robot_ready();

// In admittance_node.cpp:
void AdmittanceNode::wait_for_robot_ready() {
  RCLCPP_INFO(get_logger(), "Waiting for robot to be ready...");
  
  // Wait for joint states
  while (!joint_states_received_ && rclcpp::ok()) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Waiting for joint states...");
    rclcpp::spin_some(shared_from_this());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  
  RCLCPP_INFO(get_logger(), "Joint states received");
}
```

#### Step 1.2: Add wait_for_kinematics() method
```cpp
// In admittance_node.hpp (private section):
void wait_for_kinematics();

// In admittance_node.cpp:
void AdmittanceNode::wait_for_kinematics() {
  RCLCPP_INFO(get_logger(), "Waiting for kinematics initialization...");
  
  while (!kinematics_initialized_ && rclcpp::ok()) {
    if (load_kinematics()) {
      kinematics_initialized_ = true;
      RCLCPP_INFO(get_logger(), "Kinematics initialized successfully");
      break;
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Waiting for robot_description...");
    rclcpp::spin_some(shared_from_this());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}
```

#### Step 1.3: Modify wait_for_transformations() 
```cpp
// Already exists but needs enhancement
void AdmittanceNode::wait_for_transformations() {
  RCLCPP_INFO(get_logger(), "Waiting for required transforms...");
  
  // Wait for base->tip transform
  while (!tf_buffer_->canTransform(params_.base_link, params_.tip_link, 
                                   tf2::TimePointZero) && rclcpp::ok()) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Waiting for transform: %s -> %s",
                         params_.base_link.c_str(), params_.tip_link.c_str());
    rclcpp::spin_some(shared_from_this());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  
  RCLCPP_INFO(get_logger(), "All transforms ready");
}
```

#### Step 1.4: Add wait_for_initial_pose() method
```cpp
// In admittance_node.hpp (private section):
void wait_for_initial_pose();

// In admittance_node.cpp:
void AdmittanceNode::wait_for_initial_pose() {
  RCLCPP_INFO(get_logger(), "Initializing reference pose...");
  
  while (!desired_pose_initialized_ && rclcpp::ok()) {
    if (initialize_desired_pose()) {
      RCLCPP_INFO(get_logger(), "Reference pose initialized successfully");
      break;
    }
    rclcpp::spin_some(shared_from_this());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}
```

### Phase 2: Move Initialization to Main (20 min)

#### Step 2.1: Create initialize() method
```cpp
// In admittance_node.hpp (public section):
bool initialize();

// In admittance_node.cpp:
bool AdmittanceNode::initialize() {
  RCLCPP_INFO(get_logger(), "=== Starting Admittance Controller Initialization ===");
  
  // Step 1: Wait for robot
  wait_for_robot_ready();
  if (!rclcpp::ok()) return false;
  
  // Step 2: Initialize kinematics
  wait_for_kinematics();
  if (!rclcpp::ok()) return false;
  
  // Step 3: Wait for transforms
  wait_for_transformations();
  if (!rclcpp::ok()) return false;
  
  // Step 4: Initialize reference pose
  wait_for_initial_pose();
  if (!rclcpp::ok()) return false;
  
  RCLCPP_INFO(get_logger(), "=== Initialization Complete - Ready for Control ===");
  return true;
}
```

#### Step 2.2: Update main() to call initialize()
```cpp
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<ur_admittance_controller::AdmittanceNode>();
  
  // Initialize and wait for robot to be ready
  if (!node->initialize()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize admittance controller");
    rclcpp::shutdown();
    return 1;
  }
  
  rclcpp::Rate rate(100);  // 100Hz control loop
  RCLCPP_INFO(node->get_logger(), "Starting control loop at 100Hz...");
  
  // Now run clean control loop
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    node->control_cycle();
    rate.sleep();
  }
  
  rclcpp::shutdown();
  return 0;
}
```

### Phase 3: Clean Up control_cycle() (15 min)

#### Step 3.1: Remove all initialization checks
```cpp
void AdmittanceNode::control_cycle() {
  // DELETE these lines:
  // if (!kinematics_initialized_) return;
  // if (!joint_states_received_) return;
  // if (!desired_pose_initialized_ && !initialize_desired_pose()) return;
  
  // Keep only control logic:
  getEndEffectorPose(X_tcp_base_current_);
  
  if (!compute_admittance()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Admittance computation failed");
    return;
  }
  
  limit_to_workspace();
  limit_joint_velocities();
  send_commands_to_robot();
}
```

### Phase 4: Add Transform Caching (Optional Optimization) (20 min)

#### Step 4.1: Add transform cache variables
```cpp
// In admittance_node.hpp (private section):
// Transform caching
Eigen::Isometry3d cached_base_to_tip_;
std::chrono::steady_clock::time_point last_transform_update_;
const std::chrono::milliseconds transform_cache_duration_{10}; // 10ms cache
```

#### Step 4.2: Create cached transform getter
```cpp
// In admittance_node.hpp (private section):
void updateCachedTransforms();

// In admittance_node.cpp:
void AdmittanceNode::updateCachedTransforms() {
  auto now = std::chrono::steady_clock::now();
  auto elapsed = now - last_transform_update_;
  
  // Update cache if expired
  if (elapsed > transform_cache_duration_) {
    getEndEffectorPose(cached_base_to_tip_);
    last_transform_update_ = now;
  }
  
  // Use cached transform
  X_tcp_base_current_ = cached_base_to_tip_;
}
```

#### Step 4.3: Use cached transform in control_cycle()
```cpp
void AdmittanceNode::control_cycle() {
  // Use cached transform instead of lookup every cycle
  updateCachedTransforms();
  
  if (!compute_admittance()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Admittance computation failed");
    return;
  }
  
  limit_to_workspace();
  limit_joint_velocities();
  send_commands_to_robot();
}
```

### Phase 5: Add Debug/Status Information (10 min)

#### Step 5.1: Add initialization status tracking
```cpp
// In admittance_node.hpp (private section):
struct InitializationStatus {
  bool robot_ready = false;
  bool kinematics_ready = false;
  bool transforms_ready = false;
  bool pose_initialized = false;
  
  bool is_complete() const {
    return robot_ready && kinematics_ready && 
           transforms_ready && pose_initialized;
  }
} init_status_;
```

#### Step 5.2: Update status in wait methods
```cpp
// In each wait method, update the corresponding status:
void AdmittanceNode::wait_for_robot_ready() {
  // ... existing code ...
  init_status_.robot_ready = true;
}
// Repeat for other wait methods
```

### Phase 6: Testing and Validation (30 min)

#### Step 6.1: Test initialization sequence
1. Launch the node and verify initialization messages appear in order
2. Test with robot_description not available (should wait)
3. Test with robot not spawned (should wait for joint states)
4. Test normal startup (should initialize and start control)

#### Step 6.2: Verify control cycle performance
1. Add timing measurements to control_cycle()
2. Verify 100Hz is maintained
3. Check CPU usage is reduced (no repeated checks)

#### Step 6.3: Test error recovery
1. Test what happens if robot disconnects
2. Test transform failures during operation
3. Ensure graceful shutdown on Ctrl+C

## Implementation Order

1. **Day 1 (1-2 hours)**:
   - Implement Phase 1 (initialization methods)
   - Implement Phase 2 (main initialization)
   - Test basic initialization flow

2. **Day 2 (1 hour)**:
   - Implement Phase 3 (clean control_cycle)
   - Test control loop performance
   - Add Phase 5 (debug info)

3. **Day 3 (Optional, 1 hour)**:
   - Implement Phase 4 (transform caching)
   - Performance testing and optimization
   - Documentation updates

## Expected Results

### Before:
```cpp
control_cycle() {
  if (!kinematics_initialized_) return;     // Check 1
  if (!joint_states_received_) return;      // Check 2  
  if (!desired_pose_initialized_ && ...) return; // Check 3
  getEndEffectorPose(...);                   // TF lookup
  compute_admittance();                      // Finally!
  // ...
}
```

### After:
```cpp
control_cycle() {
  updateCachedTransforms();  // Or just getEndEffectorPose()
  compute_admittance();      // Direct control!
  limit_to_workspace();
  limit_joint_velocities();
  send_commands_to_robot();
}
```

## Success Metrics

1. **Code Simplicity**: control_cycle() reduced from ~25 lines to ~10 lines
2. **Performance**: Reduced CPU usage (no repeated initialization checks)
3. **Startup Time**: Clear initialization sequence with status messages
4. **Robustness**: Proper error handling during initialization
5. **Maintainability**: Clear separation of initialization and control

## Risk Mitigation

1. **Blocking in main**: Add timeout to initialization (e.g., 30 seconds)
2. **Lost transforms**: Add periodic transform validation
3. **State changes**: Consider adding a "reinitialization" mechanism if needed

This plan transforms our control flow to match the clean ROS1 architecture while maintaining ROS2 best practices.