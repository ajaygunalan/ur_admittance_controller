cmake_minimum_required(VERSION 3.16)
project(ur_admittance_controller VERSION 2.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Wunused-result)
endif()

add_compile_definitions(FMT_HEADER_ONLY=1)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(yaml-cpp REQUIRED)
# Generate parameter library
generate_parameter_library(
  ur_admittance_controller_parameters
  config/admittance_config.yaml
)

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
  yaml-cpp
)

# Build executables
add_executable(admittance_node
  src/admittance_node.cpp
  src/admittance_computations.cpp
  src/utilities/conversions.cpp
  src/utilities/file_io.cpp
  src/utilities/kinematics.cpp
)

add_executable(wrench_node
  src/wrench_node.cpp
  src/utilities/conversions.cpp
  src/utilities/file_io.cpp
)

add_executable(wrench_calibration_node
  src/wrench_calibration_node.cpp
  src/wrench_calibration_algorithm.cpp
)

add_executable(init_robot
  src/init_robot.cpp
  src/utilities/file_io.cpp
  src/utilities/kinematics.cpp
)

# Include directories
foreach(target admittance_node wrench_node init_robot wrench_calibration_node)
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
endforeach()

# Link parameter library
target_link_libraries(admittance_node
  ur_admittance_controller_parameters
  Eigen3::Eigen
  yaml-cpp
  ${orocos_kdl_LIBRARIES}
)

target_link_libraries(wrench_node
  Eigen3::Eigen
  yaml-cpp
)

target_link_libraries(init_robot
  Eigen3::Eigen
  yaml-cpp
  ${orocos_kdl_LIBRARIES}
)

target_link_libraries(wrench_calibration_node
  Eigen3::Eigen
  yaml-cpp
)

ament_target_dependencies(admittance_node ${NODE_DEPENDENCIES})
ament_target_dependencies(wrench_node rclcpp geometry_msgs sensor_msgs std_srvs tf2 tf2_ros tf2_eigen)
ament_target_dependencies(init_robot rclcpp rclcpp_action control_msgs controller_manager_msgs 
  sensor_msgs trajectory_msgs rcl_interfaces orocos_kdl kdl_parser urdf)
ament_target_dependencies(wrench_calibration_node rclcpp rclcpp_action control_msgs 
  sensor_msgs geometry_msgs tf2 tf2_ros tf2_eigen)

# Installation
install(
  TARGETS ur_admittance_controller_parameters
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(
  TARGETS admittance_node wrench_node wrench_calibration_node init_robot
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(
  FILES config/wrench_calibration.yaml
  DESTINATION share/${PROJECT_NAME}/config
  OPTIONAL
)

# Export configuration
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${NODE_DEPENDENCIES})

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)
endif()

ament_package()