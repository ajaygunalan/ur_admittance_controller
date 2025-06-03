# Direct Main Loop Implementation Plan

## Goal
Move the control loop directly into main() like standard ROS2 packages do - no separate run() method.

## Current Implementation (BAD)
```cpp
// In class
void AdmittanceNode::run() {
  // Control loop here
}

// In main
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ur_admittance_controller::AdmittanceNode>();
  node->run();  // <-- This is non-standard!
  rclcpp::shutdown();
  return 0;
}
```

## New Implementation (STANDARD ROS2 PATTERN)

### Option 1: Simple Direct Loop in Main
```cpp
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<ur_admittance_controller::AdmittanceNode>();
  rclcpp::Rate rate(100);  // 100Hz
  
  RCLCPP_INFO(node->get_logger(), "Starting admittance control at 100Hz...");
  
  while (rclcpp::ok()) {
    // Process callbacks
    rclcpp::spin_some(node);
    
    // Run control cycle
    node->control_cycle();
    
    // Maintain rate
    rate.sleep();
  }
  
  rclcpp::shutdown();
  return 0;
}
```

### Option 2: With Basic Error Handling
```cpp
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<ur_admittance_controller::AdmittanceNode>();
    rclcpp::Rate rate(100);
    
    while (rclcpp::ok()) {
      rclcpp::spin_some(node);
      node->control_cycle();
      rate.sleep();
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Error: %s", e.what());
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}
```

### Option 3: Minimal Version
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

## Required Changes

### 1. Remove run() method from AdmittanceNode class
Delete this entire method - we don't need it!

### 2. Make control_cycle() public
In `admittance_node.hpp`:
```cpp
public:  // Add this section if not already there
  void control_cycle();  // Move from private to public
```

### 3. Update main() function
Replace the entire main function with one of the options above.

## Why This Is Better

1. **Standard ROS2 Pattern**: This is how ROS2 examples and tutorials do it
2. **No Hidden Logic**: Everything is visible in main()
3. **Easier to Understand**: No need to look for run() method
4. **More Flexible**: Easy to add startup/shutdown logic
5. **Less Code**: Removes unnecessary abstraction

## Examples from Official ROS2 Packages

### From ros2/tutorials (publisher example):
```cpp
int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("talker");
  auto publisher = node->create_publisher<std_msgs::msg::String>("topic", 10);
  rclcpp::Rate loop_rate(2);
  
  while (rclcpp::ok()) {
    // Publish message
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  
  return 0;
}
```

### From ros2_control examples:
```cpp
int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto controller = std::make_shared<MyController>();
  
  while (rclcpp::ok()) {
    rclcpp::spin_some(controller);
    controller->update();
    // Rate control here
  }
  
  rclcpp::shutdown();
  return 0;
}
```

## Implementation Steps

1. **Delete** the `run()` method from `admittance_node.cpp`
2. **Move** `control_cycle()` to public section in header
3. **Replace** main() with the new implementation
4. **Test** that everything still works

## Final Clean Main Function

```cpp
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<ur_admittance_controller::AdmittanceNode>();
  rclcpp::Rate rate(100);  // 100Hz control loop
  
  RCLCPP_INFO(node->get_logger(), "UR Admittance Controller started at 100Hz");
  
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    node->control_cycle();
    rate.sleep();
  }
  
  RCLCPP_INFO(node->get_logger(), "Shutting down UR Admittance Controller");
  rclcpp::shutdown();
  return 0;
}
```

This is how real ROS2 packages do it - simple, direct, and in main()!