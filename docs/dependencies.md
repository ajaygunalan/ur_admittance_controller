# Dependencies

### Core ROS2 Framework
- [ROS2 Jazzy](https://github.com/ros2/ros2) - Base framework
- [rclcpp](https://github.com/ros2/rclcpp) - Core C++ client library
- [rclcpp_action](https://github.com/ros2/rclcpp) - Action client/server for trajectory execution
- [tf2/tf2_ros/tf2_eigen](https://github.com/ros2/geometry2) - Transform management between coordinate frames

### ROS2 Message/Service Types
- [geometry_msgs](https://github.com/ros2/common_interfaces) - WrenchStamped, PoseStamped
- [sensor_msgs](https://github.com/ros2/common_interfaces) - JointState  
- [std_msgs](https://github.com/ros2/common_interfaces) - Float64MultiArray, String
- [control_msgs](https://github.com/ros-controls/control_msgs) - FollowJointTrajectory action
- [controller_manager_msgs](https://github.com/ros-controls/ros2_control) - SwitchController service
- [trajectory_msgs](https://github.com/ros2/common_interfaces) - JointTrajectory
- [std_srvs](https://github.com/ros2/common_interfaces) - Standard services

### Kinematics Library
- [Orocos KDL](https://github.com/orocos/orocos_kinematics_dynamics) v1.5.1 - Kinematics and Dynamics Library
  - [kdl_parser](https://github.com/ros/robot_model) - URDF to KDL conversion
  - ChainFkSolverPos_recursive - Forward kinematics
  - ChainIkSolverVel_wdls - Weighted damped least squares velocity IK

### Math Libraries
- [Eigen3](https://gitlab.com/libeigen/eigen) - Linear algebra and transformations
  - Matrix/vector operations
  - Isometry3d transforms
  - SVD decomposition

### C++ Utilities
- [yaml-cpp](https://github.com/jbeder/yaml-cpp) - Configuration file I/O
- [fmt](https://github.com/fmtlib/fmt) - String formatting (header-only in `include/fmt/`)
- [tl::expected](https://github.com/TartanLlama/expected) - Result<T>/Status error handling (header-only in `include/tl/`)

### Build Tools
- [generate_parameter_library](https://github.com/PickNikRobotics/generate_parameter_library) - Auto-generates parameter handling from YAML
- [urdf](https://github.com/ros2/urdf) - Robot model parsing