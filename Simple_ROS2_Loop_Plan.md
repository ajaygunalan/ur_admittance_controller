# Simple ROS2 Control Loop Plan

## Objective
Create the simplest possible 100Hz control loop in ROS2 C++ without timers or complex executors. Just a basic while loop with rate control.

## Current Complex Implementation
```cpp
void AdmittanceNode::run() {
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(shared_from_this());
  
  while (rclcpp::ok()) {
    executor.spin_some();
    control_cycle();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}
```

## Simplified Implementation Options

### Option 1: Basic Rate Object (Simplest) ✅
```cpp
void AdmittanceNode::run() {
  RCLCPP_INFO(get_logger(), "Starting simple control loop at 100Hz...");
  
  rclcpp::Rate rate(100);  // 100Hz
  
  while (rclcpp::ok()) {
    // Process any pending callbacks
    rclcpp::spin_some(shared_from_this());
    
    // Run control logic
    control_cycle();
    
    // Sleep to maintain rate
    rate.sleep();
  }
}
```

### Option 2: Even Simpler - No Executor
```cpp
void AdmittanceNode::run() {
  RCLCPP_INFO(get_logger(), "Starting control loop...");
  
  auto next_cycle = std::chrono::steady_clock::now();
  const auto period = std::chrono::milliseconds(10);  // 100Hz
  
  while (rclcpp::ok()) {
    // Process callbacks without executor
    rclcpp::spin_some(shared_from_this());
    
    // Run control
    control_cycle();
    
    // Simple rate control
    next_cycle += period;
    std::this_thread::sleep_until(next_cycle);
  }
}
```

### Option 3: Absolute Minimum
```cpp
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<ur_admittance_controller::AdmittanceNode>();
  
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    node->control_cycle();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  
  rclcpp::shutdown();
  return 0;
}
```

## Recommended Implementation (Option 1)

### Changes Required:

1. **Update run() method in admittance_node.cpp:**
```cpp
void AdmittanceNode::run() {
  RCLCPP_INFO(get_logger(), "Starting simple control loop at 100Hz...");
  
  // Create rate object for 100Hz
  rclcpp::Rate rate(100);
  
  while (rclcpp::ok()) {
    // Process callbacks (subscriptions, services)
    rclcpp::spin_some(shared_from_this());
    
    // Run control computation
    control_cycle();
    
    // Sleep to maintain 100Hz rate
    rate.sleep();
  }
}
```

2. **Keep main() as is:**
```cpp
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<ur_admittance_controller::AdmittanceNode>();
  node->run();
  
  rclcpp::shutdown();
  return 0;
}
```

## Why This Works

1. **rclcpp::Rate(100)** creates a rate limiter for 100Hz
2. **rclcpp::spin_some()** processes pending callbacks without blocking
3. **rate.sleep()** sleeps for the right amount to maintain frequency
4. **No executors needed** - we handle spinning manually

## Benefits

✅ **Dead simple** - Anyone can understand it  
✅ **No timers** - Just a while loop  
✅ **No executor complexity** - Manual spin control  
✅ **Works exactly like ROS1** - Familiar pattern  
✅ **Good enough for 100Hz** - No precision timing needed  

## What We're Removing

❌ SingleThreadedExecutor  
❌ executor.add_node()  
❌ executor.spin_some()  
❌ Complex timer callbacks  
❌ Callback groups  
❌ Any "proper" ROS2 patterns  

## Testing

```bash
# Build
cd /home/ajay/ros2_ws
colcon build --packages-select ur_admittance_controller

# Run
source install/setup.bash
ros2 run ur_admittance_controller admittance_node

# Check rate (in another terminal)
ros2 topic hz /forward_velocity_controller/commands
```

## Summary

This is the simplest possible ROS2 control loop:
- Uses `rclcpp::Rate` for timing (like ROS1)
- Manual `spin_some()` for callbacks
- Basic while loop structure
- No fancy ROS2 features
- Perfect for simple control at 100Hz