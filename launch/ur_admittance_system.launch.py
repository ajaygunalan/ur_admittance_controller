#!/usr/bin/env python3
"""
UR Admittance Control Components Launch File
Launches admittance control components for UR robots
Assumes robot is already running via: ros2 launch ur_simulation_gz ur_sim_control.launch.py

Usage:
  # For simulation (after robot is already running in Gazebo)
  ros2 launch ur_admittance_controller ur_admittance_system.launch.py

  # For real robot (after robot driver is already running)
  ros2 launch ur_admittance_controller ur_admittance_system.launch.py use_sim:=false

  # With custom UR type
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
            "start_active",
            default_value="true",
            description="Start admittance controller in active state"
        )
    )

    # Configuration
    use_sim = LaunchConfiguration("use_sim")
    ur_type = LaunchConfiguration("ur_type")
    start_active = LaunchConfiguration("start_active")

    # Package paths
    ur_admittance_pkg = FindPackageShare("ur_admittance_controller")

    # Launch F/T sensor broadcaster
    ft_sensor_broadcaster = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([ur_admittance_pkg, "launch", "ft_sensor_broadcaster.launch.py"])
        ]),
        launch_arguments={
            "use_sim": use_sim,
        }.items()
    )

    # Launch admittance controller with delay to ensure F/T sensor is ready
    admittance_controller = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([ur_admittance_pkg, "launch", "admittance_control.launch.py"])
                ]),
                launch_arguments={
                    "use_sim": use_sim,
                    "ur_type": ur_type,
                    "start_active": start_active,
                }.items()
            )
        ]
    )

    # System status monitor
    system_monitor = TimerAction(
        period=5.0,
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
        period=7.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "bash", "-c",
                    "echo '' && "
                    "echo '🎯 UR Admittance Control Components Loaded!' && "
                    "echo '===========================================' && "
                    "echo '' && "
                    "echo '⚠️  Prerequisites:' && "
                    "echo '  Robot must already be running via:' && "
                    "echo '  ros2 launch ur_simulation_gz ur_sim_control.launch.py' && "
                    "echo '' && "
                    "echo '📊 Monitoring:' && "
                    "echo '  F/T Sensor: ros2 topic echo /ft_sensor_readings --once' && "
                    "echo '  Admittance: ros2 topic echo /ur_admittance_controller/cartesian_velocity_command' && "
                    "echo '  Controllers: ros2 control list_controllers' && "
                    "echo '' && "
                    "echo '🧪 Testing in Gazebo:' && "
                    "echo '  1. In Gazebo, enable Force mode (press F key)' && "
                    "echo '  2. Click and drag robot end-effector' && "
                    "echo '  3. Robot should move compliantly in drag direction' && "
                    "echo '' && "
                    "echo '⚙️  Live Parameter Tuning:' && "
                    "echo '  ros2 param set /ur_admittance_controller mass.0 5.0' && "
                    "echo '  ros2 param set /ur_admittance_controller damping_ratio.0 0.9' && "
                    "echo ''"
                ],
                output="screen"
            )
        ]
    )

    return LaunchDescription(declared_arguments + [
        # Core admittance components
        ft_sensor_broadcaster,
        admittance_controller,
        
        # Monitoring and instructions
        system_monitor,
        usage_instructions
    ])