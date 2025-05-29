# ==============================================================================
# UR Admittance Controller - CMake Build Configuration
# ==============================================================================
# This CMakeLists.txt defines the build configuration for the UR admittance
# controller package. It handles compilation of the controller library,
# parameter generation, and installation of all necessary files.
#
# Author: UR Robotics Team
# Date: 2024
# ==============================================================================

cmake_minimum_required(VERSION 3.16)
project(ur_admittance_controller VERSION 1.0.0 LANGUAGES CXX)

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
find_package(rclcpp_lifecycle REQUIRED)

# ROS2 Control packages
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(pluginlib REQUIRED)
find_package(generate_parameter_library REQUIRED)

# Message packages
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Transform packages
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
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
# Build Controller Library
# ==============================================================================
# Create shared library for the admittance controller
add_library(${PROJECT_NAME} SHARED
  src/admittance_controller.cpp
  src/realtime_computations.cpp
  src/sensor_interface.cpp
  src/controller_integration.cpp
  src/utilities.cpp
  src/plugin_export.cpp
)

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link libraries
target_link_libraries(${PROJECT_NAME}
  ur_admittance_controller_parameters
  Eigen3::Eigen
)

# Specify dependencies for ament
set(DEPENDENCIES
  controller_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  pluginlib
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  std_msgs
  control_msgs
  std_srvs
  kinematics_interface
  kdl_parser
  urdf
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
)

ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

# Export compile definitions
target_compile_definitions(${PROJECT_NAME} PRIVATE "UR_ADMITTANCE_CONTROLLER_BUILDING_DLL")
# ==============================================================================
# Plugin Export
# ==============================================================================
# Export plugin description for ros2_control
pluginlib_export_plugin_description_file(
  controller_interface
  ur_admittance_controller.xml
)

# ==============================================================================
# Installation
# ==============================================================================
# Install libraries
install(
  TARGETS ${PROJECT_NAME} ur_admittance_controller_parameters
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
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
  scripts/ur_admittance_tests.py
  scripts/ur_admittance_utils.py
  scripts/validate_notation.py
  scripts/safe_startup_test.py
  scripts/system_status.py
  DESTINATION lib/${PROJECT_NAME}
)

# ==============================================================================
# Ament Package Configuration
# ==============================================================================
# Export targets
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Export dependencies
ament_export_dependencies(${DEPENDENCIES})
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