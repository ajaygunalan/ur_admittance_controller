# ROS2 Control Loop Modernization Plan

## Current Implementation Analysis

### Current Approach (ROS1-style)
```cpp
void AdmittanceNode::run() {
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(shared_from_this());
  
  while (rclcpp::ok()) {
    executor.spin_some();    // Process callbacks
    control_cycle();         // Run control logic
    std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 100Hz
  }
}
```

**Issues:**
- Mixing executor spinning with control logic
- Using `sleep_for` instead of proper ROS2 timing mechanisms
- Not leveraging ROS2's timer-based architecture
- Potential timing jitter from sleep-based approach

## Modernization Options

### Option 1: Timer-Based Approach (RECOMMENDED) ✅

Convert the control loop to use ROS2 timers, which is the most idiomatic approach:

```cpp
class AdmittanceNode : public rclcpp::Node {
public:
  AdmittanceNode(const rclcpp::NodeOptions& options) 
  : Node("admittance_node", options) {
    // ... existing initialization ...
    
    // Create 100Hz timer for control loop
    control_timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&AdmittanceNode::control_cycle, this));
  }
  
private:
  rclcpp::TimerBase::SharedPtr control_timer_;
};

// Main becomes simple
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ur_admittance_controller::AdmittanceNode>());
  rclcpp::shutdown();
  return 0;
}
```

**Advantages:**
- Fully integrated with ROS2 executor system
- Better timing consistency
- Cleaner separation of concerns
- No manual sleep or spin_some needed
- Works well with single/multi-threaded executors

### Option 2: MultiThreadedExecutor with Callback Groups

For better real-time performance and separation of control vs sensor callbacks:

```cpp
class AdmittanceNode : public rclcpp::Node {
public:
  AdmittanceNode(const rclcpp::NodeOptions& options) 
  : Node("admittance_node", options) {
    // Create callback groups
    control_cb_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    sensor_cb_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    
    // Timer in control callback group
    control_timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&AdmittanceNode::control_cycle, this),
      control_cb_group_);
    
    // Subscriptions in sensor callback group
    auto sub_opts = rclcpp::SubscriptionOptions();
    sub_opts.callback_group = sensor_cb_group_;
    
    wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/wrench_tcp_base", rclcpp::SensorDataQoS(), 
      std::bind(&AdmittanceNode::wrench_callback, this, std::placeholders::_1),
      sub_opts);
  }
  
private:
  rclcpp::CallbackGroup::SharedPtr control_cb_group_;
  rclcpp::CallbackGroup::SharedPtr sensor_cb_group_;
};

// Main with MultiThreadedExecutor
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<ur_admittance_controller::AdmittanceNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}
```

**Advantages:**
- Parallel processing of sensor data and control
- Better real-time characteristics
- Control loop not blocked by sensor callbacks

### Option 3: Real-Time Executor (Future-Proof)

For true real-time performance (when available):

```cpp
// Use StaticSingleThreadedExecutor for deterministic behavior
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<ur_admittance_controller::AdmittanceNode>();
  
  // Static executor for more deterministic behavior
  rclcpp::executors::StaticSingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}
```

## Implementation Plan

### Phase 1: Basic Timer Implementation (Quick Win)
1. Remove `run()` method
2. Convert `control_cycle()` to timer callback
3. Update main to use `rclcpp::spin()`
4. Test timing consistency

### Phase 2: Callback Group Optimization (Optional)
1. Add callback groups for control vs sensors
2. Move to MultiThreadedExecutor
3. Profile performance improvements

### Phase 3: Additional Improvements
1. Add timing diagnostics
2. Implement overrun detection
3. Add configurable control frequency parameter

## Code Changes Required

### 1. Update Constructor
```cpp
AdmittanceNode::AdmittanceNode(const rclcpp::NodeOptions& options)
: Node("admittance_node", options) {
  // ... existing initialization ...
  
  // Add control timer (100Hz)
  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&AdmittanceNode::control_cycle, this));
    
  RCLCPP_INFO(get_logger(), "Control timer started at 100Hz");
}
```

### 2. Remove run() Method
Delete the entire `run()` method - no longer needed.

### 3. Update Main
```cpp
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<ur_admittance_controller::AdmittanceNode>();
  
  RCLCPP_INFO(node->get_logger(), "Starting admittance controller...");
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}
```

### 4. Add Timer Member Variable
```cpp
// In admittance_node.hpp
private:
  rclcpp::TimerBase::SharedPtr control_timer_;
```

## Benefits of Modernization

1. **Better Integration**: Fully leverages ROS2 executor system
2. **Improved Timing**: Timer-based approach provides better consistency
3. **Cleaner Code**: Removes manual executor management
4. **Scalability**: Easy to add more timers or switch to multi-threaded
5. **Future-Proof**: Compatible with upcoming real-time executors
6. **Diagnostics**: Easier to add timing statistics and overrun detection

## Testing Strategy

1. **Timing Verification**:
   - Log timestamps to verify 100Hz rate
   - Check for timing jitter
   - Monitor CPU usage

2. **Functional Testing**:
   - Verify all callbacks still work
   - Test parameter updates
   - Check control performance

3. **Performance Comparison**:
   - Compare with old implementation
   - Measure latency improvements
   - Check resource usage

## Recommendation

Start with **Option 1 (Timer-Based Approach)** as it provides:
- Immediate modernization benefits
- Minimal code changes
- Easy to implement and test
- Foundation for future improvements

This approach is simple, effective, and follows ROS2 best practices while maintaining the same functionality as the current implementation.