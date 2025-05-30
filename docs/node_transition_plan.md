# Step-by-Step Transition Plan: Controller → Simple Node

## Overview
Transform `ur_admittance_controller` from ROS2 Control ChainableControllerInterface to standalone `admittance_node` in 5 incremental steps over ~1.5 weeks.

## Step 1: Create Basic Node Structure (Day 1)

### Tasks:
1. **Create new files alongside existing controller** (no deletion yet!):
   ```bash
   touch src/admittance_node.cpp
   touch include/admittance_node.hpp
   touch launch/admittance_node.launch.py
   ```

2. **Basic node skeleton**:
   ```cpp
   // include/admittance_node.hpp
   class AdmittanceNode : public rclcpp::Node {
   public:
     AdmittanceNode();
   private:
     // Core components (to be ported)
     void controlLoop();
     
     // Subscriptions
     rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
     rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
     
     // Publishers
     rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
     
     // Timer
     rclcpp::TimerBase::SharedPtr control_timer_;
     
     // Parameters (reuse existing param structure)
     std::shared_ptr<ur_admittance_controller::ParamListener> param_listener_;
     ur_admittance_controller::Params params_;
   };
   ```

3. **Update CMakeLists.txt** to build both:
   ```cmake
   # Keep existing controller build
   # Add new node executable
   add_executable(admittance_node src/admittance_node.cpp)
   ament_target_dependencies(admittance_node 
     rclcpp geometry_msgs sensor_msgs trajectory_msgs tf2_ros)
   ```

### Validation:
- New node compiles and runs (does nothing yet)
- Existing controller still works unchanged
- Can run `ros2 run ur_admittance_controller admittance_node`

## Step 2: Port Core Algorithm (Days 2-3)

### Tasks:
1. **Copy algorithm files to preserve originals**:
   ```bash
   cp src/realtime_computations.cpp src/admittance_computations.cpp
   cp include/admittance_types.hpp include/admittance_node_types.hpp
   ```

2. **Remove ROS2 Control dependencies from copies**:
   - Remove `#include <controller_interface/*>`
   - Remove `hardware_interface` references
   - Keep pure algorithm code

3. **Port transform handling**:
   ```cpp
   // In admittance_node.cpp - reuse existing transform logic
   tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
   tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
   
   // Copy waitForTransforms() logic (already tested!)
   ```

4. **Port F/T sensor handling**:
   ```cpp
   // Direct topic subscription (reuse existing callback!)
   wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
     "/wrist_ft_sensor", rclcpp::SensorDataQoS(),
     [this](geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
       // Use existing ftSensorCallback logic
     });
   ```

### Validation:
- Algorithm computes correct Cartesian velocities given test forces
- Transform system initializes properly
- F/T data flows through correctly (use `ros2 topic echo`)

## Step 3: Add I/O Interfaces (Days 4-5)

### Tasks:
1. **Joint state subscription**:
   ```cpp
   joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
     "/joint_states", 10,
     [this](sensor_msgs::msg::JointState::SharedPtr msg) {
       // Update current joint positions
       for (size_t i = 0; i < params_.joints.size(); ++i) {
         auto it = std::find(msg->name.begin(), msg->name.end(), params_.joints[i]);
         if (it != msg->name.end()) {
           size_t idx = std::distance(msg->name.begin(), it);
           joint_positions_[i] = msg->position[idx];
         }
       }
     });
   ```

2. **Trajectory publisher setup**:
   ```cpp
   trajectory_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
     "/scaled_joint_trajectory_controller/joint_trajectory", 1);
   ```

3. **Control loop implementation** (20-30Hz):
   ```cpp
   control_timer_ = create_wall_timer(
     std::chrono::milliseconds(40),  // 25Hz
     [this]() { controlLoop(); });
   
   void AdmittanceNode::controlLoop() {
     // 1. Get current wrench (already handled by callback)
     // 2. Apply admittance law (reuse computeAdmittanceDynamics())
     // 3. Convert to joint velocities (reuse inverse kinematics)
     // 4. Create trajectory message
     trajectory_msgs::msg::JointTrajectory traj;
     traj.header.stamp = now();
     traj.joint_names = params_.joints;
     
     // Single point trajectory for continuous streaming
     trajectory_msgs::msg::JointTrajectoryPoint point;
     point.positions = joint_positions_;  // Current positions
     point.velocities = joint_velocities_; // From admittance
     point.time_from_start = rclcpp::Duration::from_seconds(0.1);
     
     traj.points.push_back(point);
     trajectory_pub_->publish(traj);
   }
   ```

4. **Parameter loading** (reuse existing system):
   ```cpp
   param_listener_ = std::make_shared<ur_admittance_controller::ParamListener>(
     shared_from_this());
   params_ = param_listener_->get_params();
   ```

### Validation:
- Joint states update correctly
- Trajectory messages published at correct rate
- Parameters load from existing config files

## Step 4: Integration Testing (Days 6-7)

### Tasks:
1. **Create test launch file**:
   ```python
   # launch/test_admittance_node.launch.py
   def generate_launch_description():
     return LaunchDescription([
       # Load parameters
       Node(
         package='ur_admittance_controller',
         executable='admittance_node',
         parameters=[admittance_config],
         remappings=[
           ('/wrist_ft_sensor', '/wrist_ft_sensor'),
           ('/joint_states', '/joint_states'),
         ]
       )
     ])
   ```

2. **Side-by-side testing**:
   - Run original controller in one terminal
   - Run new node in another terminal
   - Compare outputs with same force inputs

3. **Create validation script**:
   ```python
   # scripts/validate_node_transition.py
   - Subscribe to both controller outputs
   - Apply test forces
   - Compare Cartesian velocities
   - Verify trajectory generation
   ```

4. **Safety validation**:
   - Test emergency stop behavior
   - Verify force limits respected
   - Check transform failure handling

### Validation:
- Node produces same behavior as original controller
- No performance degradation
- Safety features work correctly

## Step 5: Cleanup and Optimization (Day 8)

### Tasks:
1. **Remove unused code**:
   - Delete ROS2 Control interface methods
   - Remove hardware interface dependencies
   - Clean up includes and dependencies

2. **Optimize performance**:
   - Pre-allocate trajectory messages
   - Use const references where possible
   - Profile and optimize hot paths

3. **Update documentation**:
   - Update README with new launch instructions
   - Document parameter changes (if any)
   - Add migration guide for users

4. **Final cleanup**:
   ```bash
   # After validation, remove old controller files
   git rm src/admittance_controller.cpp
   git rm src/controller_integration.cpp
   git rm include/admittance_controller.hpp
   ```

5. **Update package.xml**:
   ```xml
   <!-- Remove -->
   <depend>controller_interface</depend>
   <depend>hardware_interface</depend>
   
   <!-- Keep -->
   <depend>tf2_ros</depend>
   <depend>kinematics_interface</depend>
   ```

### Validation:
- Clean build with no warnings
- All tests pass
- Documentation updated

## Risk Mitigation

1. **Keep both implementations during transition**
2. **Test each step independently**
3. **Use version control for easy rollback**
4. **Validate in simulation before hardware**

## Expected Outcome

- **Simpler codebase**: ~50% less code
- **Easier debugging**: Standard ROS2 node patterns
- **Better stability**: No controller lifecycle issues
- **Maintained functionality**: Same admittance behavior
- **Improved maintainability**: Clear separation of concerns

## Quick Reference Commands

```bash
# Build both during transition
colcon build --packages-select ur_admittance_controller

# Test new node
ros2 run ur_admittance_controller admittance_node

# Compare outputs
ros2 topic echo /admittance_node/trajectory_cmd
ros2 topic echo /scaled_joint_trajectory_controller/joint_trajectory

# Launch with robot
ros2 launch ur_admittance_controller test_admittance_node.launch.py
```

This incremental approach ensures a **smooth, testable transition** while maintaining the ability to rollback at any step.