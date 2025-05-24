# UR Admittance Controller - Force-Compliant Control for Universal Robots
# Clean, professional CMake configuration for the complete controller package

cmake_minimum_required(VERSION 3.16)
project(ur_admittance_controller LANGUAGES CXX VERSION 1.0.0)

# Compiler settings
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# Use C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies - Core ROS2 Control and supporting packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(realtime_tools REQUIRED)

# Message and communication packages
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Kinematics and math packages
find_package(kinematics_interface REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Plugin and parameter system
find_package(pluginlib REQUIRED)
find_package(generate_parameter_library REQUIRED)

#################################################
# Parameter Generation
#################################################
generate_parameter_library(
  ur_admittance_controller_parameters
  config/admittance_config.yaml
)

#################################################
# Main Controller Library - Single clean implementation
#################################################
add_library(ur_admittance_controller SHARED
  src/admittance_controller.cpp
)

# Include directories - clean single header approach
target_include_directories(ur_admittance_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link to generated parameter library
target_link_libraries(ur_admittance_controller
  ur_admittance_controller_parameters
)

# Controller dependencies - streamlined for clean architecture
ament_target_dependencies(ur_admittance_controller
  # Core ROS2 Control
  controller_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  
  # Messages
  geometry_msgs
  control_msgs
  
  # Kinematics and math
  kinematics_interface
  kdl_parser
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
# Installation - Clean package structure
#################################################

# Install single header file
install(
  DIRECTORY include/
  DESTINATION include
)

# Install configuration, launch, URDF, and scripts
install(
  DIRECTORY 
    urdf       # Robot descriptions with F/T sensor  
    config     # Configuration files (3 clean configs)
    launch     # Launch files (3 purpose-specific launchers)
    scripts    # Status and utility scripts
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts as executables
install(
  PROGRAMS 
    scripts/system_status.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install main controller library
install(
  TARGETS 
    ur_admittance_controller
    ur_admittance_controller_parameters
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# Export package for other packages to use
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  controller_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  geometry_msgs
  control_msgs
  kinematics_interface
  pluginlib
)

# Testing (disabled until implemented)
# if(BUILD_TESTING)
#   find_package(ament_cmake_gmock REQUIRED)
#   find_package(controller_manager REQUIRED)
#   find_package(ros2_control_test_assets REQUIRED)
#
#   ament_add_gmock(test_ur_admittance_controller test/test_ur_admittance_controller.cpp)
#   target_link_libraries(test_ur_admittance_controller ur_admittance_controller)
# endif()

ament_package()
