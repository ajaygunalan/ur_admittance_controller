#!/usr/bin/env python3
"""
Test launch file for topic-based F/T sensor mode

This launch file demonstrates how to use the admittance controller
with topic-based force/torque sensor data from Gazebo simulation.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "controller_name",
            default_value="ur_admittance_controller",
            description="Name of the admittance controller"
        ),
    ]
    
    controller_name = LaunchConfiguration("controller_name")
    
    # Path to config file with topic mode enabled
    config_file = PathJoinSubstitution([
        FindPackageShare("ur_admittance_controller"),
        "config",
        "admittance_sim_mode.yaml"
    ])
    
    # Controller spawner node
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            controller_name,
            "--controller-manager", "/controller_manager",
            "--param-file", config_file,
        ],
        output="screen"
    )
    
    # Optional: Mock F/T publisher for testing without Gazebo
    mock_ft_publisher = Node(
        package="ur_admittance_controller",
        executable="mock_ft_publisher.py",
        name="mock_ft_publisher",
        output="screen",
        parameters=[{
            "publish_rate": 100.0,
            "topic_name": "/wrist_ft_sensor",
            "frame_id": "ur/ft_sensor_joint/tcp_fts_sensor",
            "force_amplitude": 10.0,
            "force_frequency": 0.5,
        }]
    )
    
    return LaunchDescription(declared_arguments + [
        controller_spawner,
        # Uncomment to use mock publisher:
        # mock_ft_publisher,
    ])