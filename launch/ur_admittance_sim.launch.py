#!/usr/bin/env python3
"""
Launch file for UR Admittance Controller in simulation mode.
This file properly manages controller switching to avoid resource conflicts.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import time

def launch_setup(context, *args, **kwargs):
    controller_manager_timeout = LaunchConfiguration('controller_manager_timeout')
    
    # List of nodes to start
    nodes_to_start = []
    
    # First, stop any conflicting controllers (e.g., scaled_joint_trajectory_controller)
    stop_default_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "scaled_joint_trajectory_controller", 
            "--controller-manager", "/controller_manager",
            "--stop",
            "--controller-manager-timeout", controller_manager_timeout
        ],
        output="screen",
        name="stop_default_controller"
    )
    nodes_to_start.append(stop_default_controller)
    
    # Load the admittance controller (without activating)
    load_admittance_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "ur_admittance_controller",
            "--controller-manager", "/controller_manager",
            "--load-only",
            "--param-file",
            "/home/ajay/ros2_ws/install/ur_admittance_controller/share/ur_admittance_controller/config/ur_admittance_controllers.yaml",
            "--controller-manager-timeout", controller_manager_timeout
        ],
        output="screen",
        name="load_admittance_controller"
    )
    
    # Activate the admittance controller after default controller is stopped
    activate_admittance_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "ur_admittance_controller",
            "--controller-manager", "/controller_manager",
            "--activate",
            "--controller-manager-timeout", controller_manager_timeout
        ],
        output="screen",
        name="activate_admittance_controller"
    )
    
    # Chain the actions: stop default -> load admittance -> activate admittance
    stop_then_load = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=stop_default_controller,
            on_exit=[load_admittance_controller],
        )
    )
    nodes_to_start.append(stop_then_load)
    
    load_then_activate = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_admittance_controller,
            on_exit=[activate_admittance_controller],
        )
    )
    nodes_to_start.append(load_then_activate)
    
    # Monitor status
    system_status = Node(
        package='ur_admittance_controller',
        executable='system_status.py',
        name='ur_admittance_status',
        output='screen'
    )
    nodes_to_start.append(system_status)
    
    return nodes_to_start

def generate_launch_description():
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'controller_manager_timeout',
            default_value='10',
            description='Timeout for controller manager operations'
        )
    )
    
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )