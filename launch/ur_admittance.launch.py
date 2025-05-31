#!/usr/bin/env python3
"""UR Admittance Node Launch"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('ur_admittance_controller')
    
    # Declare launch arguments for runtime parameter customization
    return LaunchDescription([
        DeclareLaunchArgument(
            'node_name',
            default_value='admittance_node',
            description='Name of the admittance control node'
        ),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level (debug, info, warn, error)'
        ),
        
        # Main node with generate_parameter_library support
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
    ])