#!/usr/bin/env python3
"""UR Admittance Node Launch"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('ur_admittance_controller')
    
    # Declare launch arguments for runtime parameter customization
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'node_name',
            default_value='admittance_node',
            description='Name of the admittance control node'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',  # Default to true for simulation
            description='Use simulation time'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level (debug, info, warn, error)'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'startup_delay',
            default_value='2.0',  # Wait for robot_state_publisher
            description='Delay before starting admittance node (seconds)'
        )
    )
    
    # Main node with generate_parameter_library support
    # Add a timer to wait for robot_state_publisher to be ready
    admittance_node = TimerAction(
        period=LaunchConfiguration('startup_delay'),
        actions=[
            Node(
                package="ur_admittance_controller",
                executable="admittance_node",
                name=LaunchConfiguration('node_name'),
                output="screen",
                parameters=[
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}
                ],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                emulate_tty=True
            )
        ]
    )
    
    return LaunchDescription(declared_arguments + [admittance_node])