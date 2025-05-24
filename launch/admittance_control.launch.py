#!/usr/bin/env python3
"""
UR Admittance Controller Launch File
Launches the admittance controller for both simulation and real robot

Usage:
  # For simulation (Gazebo)
  ros2 launch ur_admittance_controller admittance_controller.launch.py

  # For real robot
  ros2 launch ur_admittance_controller admittance_controller.launch.py use_sim:=false

  # Start inactive (for chaining setup)
  ros2 launch ur_admittance_controller admittance_controller.launch.py start_active:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
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
            description="Use simulation (true) or real robot (false)"
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
            "start_active",
            default_value="true",
            description="Start controller in active state (true) or inactive (false)"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_name",
            default_value="ur_admittance_controller",
            description="Name of the admittance controller"
        )
    )

    # Configuration
    use_sim = LaunchConfiguration("use_sim")
    ur_type = LaunchConfiguration("ur_type")
    start_active = LaunchConfiguration("start_active")
    controller_name = LaunchConfiguration("controller_name")

    # Configuration file
    config_file = PathJoinSubstitution([
        FindPackageShare("ur_admittance_controller"),
        "config",
        "admittance_config.yaml"
    ])

    # Load Admittance Controller (inactive)
    load_admittance_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            controller_name,
            "--controller-manager", "/controller_manager",
            "--param-file", config_file,
            "--inactive",
            "--controller-manager-timeout", "300"
        ],
        output="screen",
        parameters=[{
            "use_sim": use_sim,
            "ur_type": ur_type
        }]
    )

    # Activate Admittance Controller (if requested)
    activate_admittance_controller = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    controller_name,
                    "--controller-manager", "/controller_manager",
                    "--activate",
                    "--controller-manager-timeout", "300"
                ],
                output="screen"
            )
        ],
        condition=IfCondition(start_active)
    )

    # Status information
    status_info = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="ur_admittance_controller",
                executable="system_status.py",  # ✅ Updated to use consolidated status script
                name="admittance_status",
                output="screen",
                parameters=[{
                    "controller_name": controller_name,
                    "expected_state": "active" if start_active else "inactive"
                }]
            )
        ]
    )

    return LaunchDescription(declared_arguments + [
        load_admittance_controller,
        activate_admittance_controller,
        status_info
    ])