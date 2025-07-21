# ==============================================================================
# UR Admittance Node - CMake Build Configuration
# ==============================================================================
# This CMakeLists.txt defines the build configuration for the UR admittance
# node package. It builds a standalone ROS2 node for force-compliant control,
# parameter generation, and installation of all necessary files.
#
# Author: UR Robotics Team
# Date: 2024-2025
# ==============================================================================

cmake_minimum_required(VERSION 3.16)
project(ur_admittance_controller VERSION 2.0.0 LANGUAGES CXX)

# C++ Standard Configuration
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# ==============================================================================
# Find Dependencies
# ==============================================================================

# Core ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Parameter generation and plugin loading
find_package(pluginlib REQUIRED)
find_package(generate_parameter_library REQUIRED)

# Message packages
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Additional packages for init_robot and verify_poses
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

# Transform packages
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

# Math and kinematics packages
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)

# YAML for calibration file I/O
find_package(yaml-cpp REQUIRED)
# ==============================================================================
# Generate Parameter Library
# ==============================================================================
# Generate C++ header from YAML parameter definition
generate_parameter_library(
  ur_admittance_controller_parameters
  config/admittance_config.yaml
)

# ==============================================================================
# Node Dependencies
# ==============================================================================
set(NODE_DEPENDENCIES
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  tf2
  tf2_ros
  tf2_eigen
  orocos_kdl
  urdf
  kdl_parser
)

# ==============================================================================
# Build Gravity Compensation Library
# ==============================================================================




# ==============================================================================
# Build Admittance Node Executable (New Standalone Implementation)
# ==============================================================================
add_executable(admittance_node
  src/admittance_node.cpp
  src/admittance_computations.cpp
)

# ==============================================================================
# Build Wrench Node
# ==============================================================================
add_executable(wrench_node
  src/wrench_node.cpp
)

# ==============================================================================
# Build Wrench Calibration Node
# ==============================================================================
add_executable(wrench_calibration_node
  src/wrench_calibration_node.cpp
  src/wrench_calibration_algorithm.cpp
)

# ==============================================================================
# Build Init Robot Executable
# ==============================================================================
add_executable(init_robot
  src/init_robot.cpp
)

# ==============================================================================
# Build Verify Poses Executable
# ==============================================================================
add_executable(verify_poses
  src/verify_poses.cpp
)

# ==============================================================================
# Build Test IK Executable
# ==============================================================================
add_executable(test_ik
  src/test_ik.cpp
)

# Include directories for nodes
target_include_directories(admittance_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(wrench_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(init_robot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(verify_poses PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(test_ik PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(wrench_calibration_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link parameter library
target_link_libraries(admittance_node
  ur_admittance_controller_parameters
  Eigen3::Eigen
)

target_link_libraries(wrench_node
  Eigen3::Eigen
  yaml-cpp
)

target_link_libraries(init_robot
  Eigen3::Eigen
  yaml-cpp
)

target_link_libraries(verify_poses
  Eigen3::Eigen
)

target_link_libraries(test_ik
  Eigen3::Eigen
  yaml-cpp
)

target_link_libraries(wrench_calibration_node
  Eigen3::Eigen
  yaml-cpp
)

# Note: Removed PACKAGE_SOURCE_DIR definitions - now using ament_index_cpp for proper installation support

ament_target_dependencies(admittance_node ${NODE_DEPENDENCIES})
ament_target_dependencies(wrench_node rclcpp geometry_msgs std_srvs tf2 tf2_ros tf2_eigen ament_index_cpp)

# Additional dependencies for init_robot and verify_poses
set(INIT_ROBOT_DEPENDENCIES
  rclcpp
  rclcpp_action
  control_msgs
  controller_manager_msgs
  sensor_msgs
  trajectory_msgs
  rcl_interfaces
  orocos_kdl
  kdl_parser
  urdf
)

set(VERIFY_POSES_DEPENDENCIES
  rclcpp
  sensor_msgs
  rcl_interfaces
  orocos_kdl
  kdl_parser
  urdf
)

ament_target_dependencies(init_robot ${INIT_ROBOT_DEPENDENCIES})
ament_target_dependencies(verify_poses ${VERIFY_POSES_DEPENDENCIES})
ament_target_dependencies(test_ik ${INIT_ROBOT_DEPENDENCIES})

# Dependencies for wrench_calibration_node
set(CALIBRATION_DEPENDENCIES
  rclcpp
  rclcpp_action
  control_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_eigen
  ament_index_cpp
)

ament_target_dependencies(wrench_calibration_node ${CALIBRATION_DEPENDENCIES})

# ==============================================================================
# Installation
# ==============================================================================
# Install parameter library
install(
  TARGETS ur_admittance_controller_parameters
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# Install the node executables
install(
  TARGETS admittance_node wrench_node wrench_calibration_node init_robot verify_poses test_ik
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install configuration and launch files
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Ensure config directory structure is maintained for calibration file output
install(
  FILES config/wrench_calibration.yaml
  DESTINATION share/${PROJECT_NAME}/config
  OPTIONAL  # File may not exist initially
)

# ==============================================================================
# Ament Package Configuration
# ==============================================================================
# Export targets
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Export dependencies
ament_export_dependencies(${NODE_DEPENDENCIES})
# ==============================================================================
# Testing Configuration
# ==============================================================================
if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)
  
  # Add test executables here when implemented
  # ament_add_gmock(test_admittance_controller test/test_admittance_controller.cpp)
  # target_link_libraries(test_admittance_controller ${PROJECT_NAME})
endif()

# ==============================================================================
# Finalize Package
# ==============================================================================
ament_package()