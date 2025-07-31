# Technology Stack

## Core Framework
- **ROS2 Jazzy**: Base robotics framework
- **CMake**: Build system (minimum version 3.16)
- **C++20**: Programming language standard

## ROS2 Packages
- **rclcpp**: Core C++ client library
- **rclcpp_action**: Action client/server for trajectory execution
- **pluginlib**: Plugin loading
- **generate_parameter_library**: Auto-generates parameter handling from YAML

## Message Types
- geometry_msgs (WrenchStamped, PoseStamped)
- sensor_msgs (JointState)
- std_msgs (Float64MultiArray, String)
- control_msgs (FollowJointTrajectory action)
- controller_manager_msgs (SwitchController service)
- trajectory_msgs (JointTrajectory)

## Kinematics & Math
- **Orocos KDL** v1.5.1: Kinematics and Dynamics Library
- **kdl_parser**: URDF to KDL conversion
- **Eigen3**: Linear algebra, transformations, matrix operations

## Transform Management
- tf2/tf2_ros/tf2_eigen: Transform management between coordinate frames

## Utilities
- **yaml-cpp**: Configuration file I/O
- **fmt**: String formatting (header-only, included in project)
- **tl::expected**: Result<T>/Status error handling (header-only, included in project)

## Build Tools
- **ament_cmake**: ROS2 build system
- **colcon**: Build tool for ROS2 packages

## Testing (configured but not implemented)
- ament_cmake_gmock
- controller_manager
- ros2_control_test_assets