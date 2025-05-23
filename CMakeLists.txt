# UR Admittance Controller - Force-Compliant Control for Universal Robots
# CMake configuration for building the controller, tools, and examples

cmake_minimum_required(VERSION 3.16)
project(ur_admittance_controller LANGUAGES CXX VERSION 1.0.0)

# Compiler settings
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# Use C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies - Organized by category

# ROS2 core dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# ROS2 controller dependencies
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(realtime_tools REQUIRED)

# Message dependencies
find_package(control_msgs REQUIRED)     # For joint trajectory actions
find_package(geometry_msgs REQUIRED)     # For wrench and twist messages
find_package(trajectory_msgs REQUIRED)   # For trajectory generation
find_package(std_srvs REQUIRED)          # For trigger services

# Kinematics dependencies
find_package(kinematics_interface REQUIRED)
find_package(kinematics_interface_kdl REQUIRED)
find_package(kdl_parser REQUIRED)

# TF dependencies
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Other dependencies
find_package(pluginlib REQUIRED)                # For controller plugins
find_package(generate_parameter_library REQUIRED) # For parameter handling
find_package(Eigen3 REQUIRED)                   # For matrix/vector operations

#################################################
# Parameters
#################################################
generate_parameter_library(
  ur_admittance_controller_parameters
  src/admittance_controller_parameters.yaml
)

#################################################
# Main Controller Library
#################################################
add_library(ur_admittance_controller SHARED
  src/admittance_controller.cpp
)

# Include directories
target_include_directories(ur_admittance_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link to generated parameter library
target_link_libraries(ur_admittance_controller
  ur_admittance_controller_parameters
)

# Add controller dependencies
ament_target_dependencies(ur_admittance_controller
  # Core dependencies
  controller_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  
  # Message dependencies
  control_msgs
  geometry_msgs
  
  # Kinematics dependencies
  kinematics_interface
  kinematics_interface_kdl
  kdl_parser
  
  # TF dependencies
  tf2_ros
  tf2_geometry_msgs
  
  # Plugin system
  pluginlib
)

# Configure visibility macros for library symbols
target_compile_definitions(ur_admittance_controller PRIVATE "UR_ADMITTANCE_CONTROLLER_BUILDING_DLL")

# Register controller as a pluginlib plugin
pluginlib_export_plugin_description_file(controller_interface ur_admittance_controller.xml)

#################################################
# Utility Executables
#################################################

# Wrench Signal Generator - Simulates F/T sensor signals
add_executable(wrench_signal_generator src/wrench_signal_generator.cpp)
target_include_directories(wrench_signal_generator PRIVATE include)
ament_target_dependencies(wrench_signal_generator 
  # Core ROS2
  rclcpp
  # Messages
  geometry_msgs std_srvs
  # TF
  tf2_ros tf2
  # Math
  Eigen3
)

# Joint Motion Example - Sends trajectories to UR robots
add_executable(joint_motion_example src/joint_motion_example.cpp)
target_include_directories(joint_motion_example PRIVATE include)
ament_target_dependencies(joint_motion_example 
  # Core ROS2
  rclcpp rclcpp_action 
  # Messages
  control_msgs trajectory_msgs std_srvs
)

#################################################
# Installation
#################################################

# Install header files
install(
  DIRECTORY include/
  DESTINATION include
)

# Install configuration and launch files
install(
  DIRECTORY 
    launch    # Launch files
    config    # Configuration files
  DESTINATION share/${PROJECT_NAME}
)

# Install libraries and executables
install(
  TARGETS 
    # Main controller library
    ur_admittance_controller
    ur_admittance_controller_parameters
    # Utility executables
    wrench_signal_generator
    joint_motion_example
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}  # For executables
  ARCHIVE DESTINATION lib                 # For static libraries
  LIBRARY DESTINATION lib                 # For shared libraries
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  controller_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  kinematics_interface
)

# Tests disabled until test files are created
# if(BUILD_TESTING)
#   find_package(ament_cmake_gmock REQUIRED)
#   find_package(controller_manager REQUIRED)
#   find_package(ros2_control_test_assets REQUIRED)
#
#   ament_add_gmock(test_ur_admittance_controller test/test_ur_admittance_controller.cpp)
#   target_link_libraries(test_ur_admittance_controller
#     ur_admittance_controller
#   )
# endif()

ament_package()
