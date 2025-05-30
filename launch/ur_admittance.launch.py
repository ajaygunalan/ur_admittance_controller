#!/usr/bin/env python3
"""
UR Admittance Node - Launch File

This launch file starts the standalone UR admittance node with appropriate
configurations for simulation or real robot operation.

USAGE:
    # Simulation mode (default)
    ros2 launch ur_admittance_controller ur_admittance.launch.py

    # Real robot mode
    ros2 launch ur_admittance_controller ur_admittance.launch.py use_sim:=false

    # Custom node name
    ros2 launch ur_admittance_controller ur_admittance.launch.py node_name:=my_admittance_node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the launch description for the UR admittance node."""
    
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim",
            default_value="true",
            description="Use simulation mode (true) or real robot (false)"
        ),
        DeclareLaunchArgument(
            "node_name", 
            default_value="admittance_node",
            description="Name of the admittance node"
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value="admittance_config.yaml",
            description="Configuration file name"
        ),
    ]
    
    # Get launch configurations
    use_sim = LaunchConfiguration("use_sim")
    node_name = LaunchConfiguration("node_name")
    config_file = LaunchConfiguration("config_file")
    
    # Path to config file
    config_path = PathJoinSubstitution([
        FindPackageShare("ur_admittance_controller"),
        "config",
        config_file
    ])
    
    # Simulation-specific config overlay
    sim_config_path = PathJoinSubstitution([
        FindPackageShare("ur_admittance_controller"),
        "config",
        "admittance_sim_mode.yaml"
    ])
    
    # Create the admittance node
    admittance_node = Node(
        package="ur_admittance_controller",
        executable="admittance_node",
        name=node_name,
        parameters=[
            config_path,
            # Override sensor mode for simulation
            {"sensor_interface.topic_config.topic_name": "/wrist_ft_sensor"},
            # Node namespace to avoid conflicts
            {"use_sim_time": use_sim}
        ],
        output="screen",
        emulate_tty=True,
    )
    
    # Build and return the launch description
    return LaunchDescription(declared_arguments + [admittance_node])