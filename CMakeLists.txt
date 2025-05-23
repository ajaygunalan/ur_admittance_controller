cmake_minimum_required(VERSION 3.16)
project(ur_admittance_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(kinematics_interface REQUIRED)
find_package(kinematics_interface_kdl REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(kdl_parser REQUIRED)

generate_parameter_library(
  ur_admittance_controller_parameters
  src/admittance_controller_parameters.yaml
)

add_library(ur_admittance_controller SHARED
  src/admittance_controller.cpp
)

target_include_directories(ur_admittance_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(ur_admittance_controller
  ur_admittance_controller_parameters
)

ament_target_dependencies(ur_admittance_controller
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  control_msgs
  geometry_msgs
  kinematics_interface
  kinematics_interface_kdl
  tf2_ros
  tf2_geometry_msgs
  kdl_parser
)

# Causes the visibility macros to use dllexport rather than dllimport
target_compile_definitions(ur_admittance_controller PRIVATE "UR_ADMITTANCE_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(controller_interface ur_admittance_controller.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Add utility executables
add_executable(wrench_signal_generator src/wrench_signal_generator.cpp)
target_include_directories(wrench_signal_generator PRIVATE include)
ament_target_dependencies(wrench_signal_generator 
  rclcpp geometry_msgs std_srvs tf2_ros tf2 Eigen3)

# Joint motion example utility
add_executable(joint_motion_example src/joint_motion_example.cpp)
target_include_directories(joint_motion_example PRIVATE include)
ament_target_dependencies(joint_motion_example 
  rclcpp rclcpp_action control_msgs trajectory_msgs std_srvs)

install(
  TARGETS 
    ur_admittance_controller
    ur_admittance_controller_parameters
    wrench_signal_generator
    joint_motion_example
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
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
