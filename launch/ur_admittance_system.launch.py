#!/usr/bin/env python3
"""
Complete UR Admittance Control System Launch File
Launches complete system for both Gazebo simulation and real robot

Usage:
  # Complete simulation system
  ros2 launch ur_admittance_controller ur_admittance_system.launch.py

  # Real robot system  
  ros2 launch ur_admittance_controller ur_admittance_system.launch.py use_sim:=false robot_ip:=192.168.1.100

  # Add to existing simulation
  ros2 launch ur_admittance_controller ur_admittance_system.launch.py add_to_existing:=true

  # Different UR model
  ros2 launch ur_admittance_controller ur_admittance_system.launch.py ur_type:=ur10e
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim",
            default_value="true",
            description="Use Gazebo simulation (true) or real robot (false)"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="UR robot type (ur3, ur3e, ur5, ur5e, ur10, ur10e, etc.)"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.100",
            description="IP address of real UR robot"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz for visualization"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "add_to_existing",
            default_value="false",
            description="Add to existing simulation/robot instead of launching new system"
        )
    )

    # Configuration
    use_sim = LaunchConfiguration("use_sim")
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    launch_rviz = LaunchConfiguration("launch_rviz")
    add_to_existing = LaunchConfiguration("add_to_existing")

    # Package paths
    ur_admittance_pkg = FindPackageShare("ur_admittance_controller")
    ur_simulation_pkg = FindPackageShare("ur_simulation_gz")
    ur_robot_driver_pkg = FindPackageShare("ur_robot_driver")

    # Configuration files
    complete_config_file = PathJoinSubstitution([
        ur_admittance_pkg, "config", "ur_complete_system.yaml"
    ])

    # OPTION 1: Complete Gazebo Simulation System
    gazebo_simulation_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([ur_simulation_pkg, "launch", "ur_sim_control.launch.py"])
        ]),
        launch_arguments={
            "ur_type": ur_type,
            "description_package": "ur_admittance_controller",
            "description_file": "ur5e_admittance_sim.urdf.xacro",
            "controllers_file": complete_config_file,
            "launch_rviz": launch_rviz,
        }.items(),
        condition=IfCondition(use_sim),
        condition_if=UnlessCondition(add_to_existing)
    )

    # OPTION 2: Real Robot System
    real_robot_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([ur_robot_driver_pkg, "launch", "ur_control.launch.py"])
        ]),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "launch_rviz": launch_rviz,
            "controllers_file": complete_config_file,
        }.items(),
        condition=UnlessCondition(use_sim),
        condition_if=UnlessCondition(add_to_existing)
    )

    # OPTION 3: Add-on Components (for existing systems)
    addon_ft_sensor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([ur_admittance_pkg, "launch", "ft_sensor_broadcaster.launch.py"])
        ]),
        launch_arguments={
            "use_sim": use_sim,
        }.items(),
        condition=IfCondition(add_to_existing)
    )

    addon_admittance_controller = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([ur_admittance_pkg, "launch", "admittance_controller.launch.py"])
                ]),
                launch_arguments={
                    "use_sim": use_sim,
                    "ur_type": ur_type,
                    "start_active": "true",
                }.items()
            )
        ],
        condition=IfCondition(add_to_existing)
    )

    # System status and monitoring
    system_monitor = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="ur_admittance_controller",
                executable="system_status.py",
                name="ur_admittance_system_monitor",
                output="screen",
                parameters=[{
                    "use_sim": use_sim,
                    "ur_type": ur_type,
                    "expected_controllers": [
                        "scaled_joint_trajectory_controller",
                        "joint_state_broadcaster",
                        "force_torque_sensor_broadcaster",
                        "ur_admittance_controller"
                    ]
                }]
            )
        ]
    )

    # Usage instructions
    usage_instructions = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "bash", "-c",
                    "echo '' && "
                    "echo '🎯 UR Admittance Control System Ready!' && "
                    "echo '===========================================' && "
                    "echo '' && "
                    "echo '📊 Monitoring:' && "
                    "echo '  F/T Sensor: ros2 topic echo /ft_sensor_readings --once' && "
                    "echo '  Admittance: ros2 topic echo /ur_admittance_controller/cartesian_velocity_command' && "
                    "echo '  Controllers: ros2 control list_controllers' && "
                    "echo '' && "
                    "echo '🧪 Testing:' && "
                    "if [ '$use_sim' = 'true' ]; then "
                    "echo '  1. In Gazebo, enable Force mode (press F key)' && "
                    "echo '  2. Click and drag robot end-effector' && "
                    "echo '  3. Robot should move compliantly in drag direction'; "
                    "else "
                    "echo '  1. Gently apply force to robot end-effector' && "
                    "echo '  2. Robot should move compliantly in force direction' && "
                    "echo '  3. Robot should stop when force is removed'; "
                    "fi && "
                    "echo '' && "
                    "echo '⚙️  Live Tuning:' && "
                    "echo '  ros2 param set /ur_admittance_controller mass.0 5.0' && "
                    "echo '  ros2 param set /ur_admittance_controller damping_ratio.0 0.9' && "
                    "echo ''"
                ],
                output="screen"
            )
        ]
    )

    return LaunchDescription(declared_arguments + [
        # Core systems
        gazebo_simulation_system,
        real_robot_system,
        
        # Add-on components
        addon_ft_sensor,
        addon_admittance_controller,
        
        # Monitoring and instructions
        system_monitor,
        usage_instructions
    ])