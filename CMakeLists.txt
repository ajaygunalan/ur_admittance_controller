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
set(CMAKE_CXX_STANDARD 17)
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

# Parameter generation and plugin loading
find_package(pluginlib REQUIRED)
find_package(generate_parameter_library REQUIRED)

# Message packages
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Transform packages
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

# Math and kinematics packages
find_package(Eigen3 REQUIRED)
find_package(kinematics_interface REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
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
  kinematics_interface
  pluginlib
  urdf
  kdl_parser
)

# ==============================================================================
# Build Admittance Node Executable (New Standalone Implementation)
# ==============================================================================
add_executable(admittance_node
  src/admittance_node.cpp
  src/admittance_computations.cpp
  src/sensor_handling.cpp
)

# Include directories for node
target_include_directories(admittance_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link parameter library
target_link_libraries(admittance_node
  ur_admittance_controller_parameters
  Eigen3::Eigen
)

ament_target_dependencies(admittance_node ${NODE_DEPENDENCIES})

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

# Install the new admittance node executable
install(
  TARGETS admittance_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install configuration, launch files, and scripts
install(
  DIRECTORY config launch scripts
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts as executables
install(PROGRAMS
  scripts/test_suite.py
  scripts/validate.py
  DESTINATION lib/${PROJECT_NAME}
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