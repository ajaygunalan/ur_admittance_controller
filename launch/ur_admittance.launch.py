#!/usr/bin/env python3
"""UR Admittance Node Launch with Zero-Error Initialization"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('ur_admittance_controller')
    
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
    
    # Get URDF from ur_description package
    ur_description_path = FindPackageShare('ur_description')
    urdf_file = PathJoinSubstitution([
        ur_description_path, 'urdf', 'ur.urdf.xacro'
    ])
    
    # Process xacro to get robot description
    robot_description = Command([
        'xacro ', urdf_file,
        ' name:=ur',
        ' ur_type:=ur3',  # or ur5, ur10, etc.
        ' tf_prefix:=',
        ' simulation_controllers:=', PathJoinSubstitution([
            FindPackageShare('ur_simulation_gz'), 'config', 'ur_controllers.yaml'
        ])
    ])
    
    # Run initialization script to position robot at equilibrium
    init_script = ExecuteProcess(
        cmd=['ros2', 'run', 'ur_admittance_controller', 'init_admittance.py'],
        name='admittance_initializer',
        output='screen'
    )
    
    # Delay controller spawner to ensure initialization completes
    delayed_controller_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['forward_velocity_controller', '-c', '/controller_manager'],
                output='screen',
            )
        ]
    )
    
    # Wrench controller spawner (can start immediately)
    wrench_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['wrench_controller', '-c', '/controller_manager'],
        output='screen',
    )
    
    # Admittance node (start after initialization)
    admittance_node = TimerAction(
        period=4.0,  # Wait for initialization and controller to be ready
        actions=[
            Node(
                package="ur_admittance_controller",
                executable="admittance_node",
                name="admittance_node",
                output="screen",
                parameters=[
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                    {'robot_description': robot_description}
                ],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                emulate_tty=True
            )
        ]
    )
    
    # Wrench node
    wrench_node = TimerAction(
        period=4.0,  # Start with admittance node
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
        # Initialize robot position first
        init_script,
        
        # Then start controllers
        delayed_controller_spawner,
        wrench_controller_spawner,
        
        # Finally start nodes
        admittance_node,
        wrench_node,
    ])