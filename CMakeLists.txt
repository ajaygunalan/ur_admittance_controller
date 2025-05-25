cmake_minimum_required(VERSION 3.16)
project(ur_admittance_controller VERSION 1.0.0 LANGUAGES CXX)

# C++17 is required
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(pluginlib REQUIRED)
find_package(generate_parameter_library REQUIRED)

# Messages
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Transforms and services
find_package(geometry_msgs REQUIRED)

# Transforms and Math
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

# Kinematics and URDF
find_package(kinematics_interface REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)

# Generate parameter library
generate_parameter_library(
  ur_admittance_controller_parameters
  config/admittance_config.yaml
)

# Create controller library
add_library(${PROJECT_NAME} SHARED
  src/admittance_controller.cpp
  src/communication_interface.cpp  # Add external communication handlers
  src/controller_setup.cpp  # Add lifecycle management and configuration
)

# Set include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link parameter library
target_link_libraries(${PROJECT_NAME}
  ur_admittance_controller_parameters
)

# Specify all dependencies
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

# Eigen requires special handling
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

# Configure visibility macros
target_compile_definitions(${PROJECT_NAME} PRIVATE "UR_ADMITTANCE_CONTROLLER_BUILDING_DLL")

# Plugin export
pluginlib_export_plugin_description_file(controller_interface ur_admittance_controller.xml)

# Install targets
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

# Install other resources
install(
  DIRECTORY config launch urdf scripts
  DESTINATION share/${PROJECT_NAME}
)

# Install Python executables
install(
  PROGRAMS 
    scripts/system_status.py
    scripts/test_safe_startup.py
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)
  
  # Add tests here when implemented
endif()

ament_package()