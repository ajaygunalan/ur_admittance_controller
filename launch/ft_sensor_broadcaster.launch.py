#!/usr/bin/env python3
"""
Force/Torque Sensor Broadcaster Launch File
Launches F/T sensor broadcaster for both simulation and real robot

Usage:
  # For simulation (Gazebo)
  ros2 launch ur_admittance_controller ft_sensor_broadcaster.launch.py

  # For real robot  
  ros2 launch ur_admittance_controller ft_sensor_broadcaster.launch.py use_sim:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
            "sensor_name",
            default_value="wrist_ft_sensor",  # FIXED: matches URDF
            description="Name of the F/T sensor"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "frame_id",
            default_value="tool0",
            description="Frame ID for F/T sensor data"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "topic_name",
            default_value="ft_sensor_readings",
            description="Output topic for F/T data"
        )
    )

    # Configuration
    use_sim = LaunchConfiguration("use_sim")
    sensor_name = LaunchConfiguration("sensor_name")
    frame_id = LaunchConfiguration("frame_id")
    topic_name = LaunchConfiguration("topic_name")

    # Configuration file
    config_file = PathJoinSubstitution([
        FindPackageShare("ur_admittance_controller"),
        "config",
        "ur_complete_system.yaml"  # Consolidated configuration
    ])

    # Force/Torque Sensor Broadcaster
    ft_sensor_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "force_torque_sensor_broadcaster",
            "--controller-manager", "/controller_manager",
            "--param-file", config_file,
            "--controller-manager-timeout", "300"
        ],
        output="screen"
    )

    return LaunchDescription(declared_arguments + [
        ft_sensor_broadcaster
    ])