#!/usr/bin/env python3
"""UR Admittance Node Launch with Zero-Error Initialization"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('ur_admittance_controller')
    
    # Navigate up 4 levels to workspace root, then to logs/
    # From: install/ur_admittance_controller/share/ur_admittance_controller
    # To: ros2_ws/logs/
    log_dir = os.path.join(pkg_dir, '..', '..', '..', '..', 'logs')
    log_dir = os.path.normpath(log_dir)  # Clean up the path
    os.makedirs(log_dir, exist_ok=True)
    os.environ['ROS_LOG_DIR'] = log_dir
    
    # Declare launch arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
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
    
    # Robot state publisher (should already be running from ur_sim_control.launch.py)
    # Joint state broadcaster (should already be running)
    # robot_description already loaded by ur_simulation_gz into /robot_state_publisher
    
    
    # Trajectory controller removed - robot equilibrium handled by init_robot.py script
    
    # Wrench controller spawner (can start immediately)
    wrench_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['wrench_controller', '-c', '/controller_manager'],
        output='screen',
    )
    
    # Admittance node
    admittance_node = TimerAction(
        period=2.0,  # Wait for controllers to be ready
        actions=[
            Node(
                package="ur_admittance_controller",
                executable="admittance_node",
                name="admittance_node",
                output="screen",
                parameters=[
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}
                ],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                emulate_tty=True
            )
        ]
    )
    
    # Wrench node
    wrench_node = TimerAction(
        period=2.0,  # Start with admittance node
        actions=[
            Node(
                package="ur_admittance_controller",
                executable="wrench_node",
                name="wrench_node",
                output="screen",
                parameters=[
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}
                ],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                emulate_tty=True
            )
        ]
    )
    
    return LaunchDescription(declared_arguments + [
        # Start controllers
        wrench_controller_spawner,
        
        # Start nodes
        admittance_node,
        wrench_node,
    ])